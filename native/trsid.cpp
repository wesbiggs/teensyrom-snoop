// trsid-native: the teensyrom-snoop SID player in C++ on classic reSID, for computers too slow for
// host/trsid.py (a Raspberry Pi Zero: reSIDfp alone needs more than its one core).
//
//   trsid-native --tcp 192.168.1.50                   play live (ALSA)
//   trsid-native --tcp 192.168.1.50 --no-play --wav x.wav --seconds 60    record live, no sound card
//   trsid-native --port /dev/ttyACM0 --wav x.wav      play and record, over USB serial
//   trsid-native --bench                              time the player on a synthetic tune and an 8 kHz digi
//   trsid-native --bench --replay sid.snoop           ...or on a recording (host/trsnoop.py --raw)
//   trsid-native --replay sid.snoop --wav x.wav       a recording to WAV
//
// The wire format is docs/PROTOCOL.md; rendering and live timing follow host/sidplay.py (SidRenderer,
// LiveClock, LivePlayer) and the command line follows host/trsid.py.  Live play defaults to reSID's
// fast sampling (the only one a Raspberry Pi Zero keeps up with), a recording to interpolate.
// reSID (GPL-2.0-or-later) is fetched and compiled by native/Makefile.

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#if HAVE_ALSA
#include <alsa/asoundlib.h>
#endif

#include "sid.h"

namespace {

const uint32_t PHI2_PAL = 985248;
const double WARMUP = 1.2;          // seconds each SID runs silent first (its DC offset settles)
const int SID_REGS = 0x19;          // $00-$18 writable

// ---- wire format ---------------------------------------------------------------------------------

enum { F_HELLO = 1, F_WRITES = 2, F_STATUS = 3, F_BYE = 4 };

uint16_t crc16(const uint8_t* p, size_t n) {        // CRC-16/CCITT-FALSE
    static uint16_t table[256];
    static bool init = false;
    if (!init) {
        for (int i = 0; i < 256; i++) {
            uint16_t c = (uint16_t)(i << 8);
            for (int k = 0; k < 8; k++) c = (uint16_t)((c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1);
            table[i] = c;
        }
        init = true;
    }
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) crc = (uint16_t)((crc << 8) ^ table[((crc >> 8) ^ p[i]) & 0xFF]);
    return crc;
}

struct Block {
    uint64_t time;
    uint16_t addr;
    std::vector<uint8_t> data;
};

struct Frame {
    int type = 0;
    uint32_t phi2 = 0;              // HELLO
    std::vector<Block> blocks;      // WRITES
    uint16_t seq = 0;               // WRITES: seq; STATUS: next_seq
    uint64_t time = 0;              // STATUS
    uint32_t late = 0, late250 = 0, late1000 = 0;
    int reason = 0;                 // BYE
};

bool varint(const std::vector<uint8_t>& p, size_t& i, uint64_t& v) {
    v = 0;
    for (int shift = 0; shift < 64; shift += 7) {
        if (i >= p.size()) return false;
        uint8_t b = p[i++];
        v |= (uint64_t)(b & 0x7F) << shift;
        if (!(b & 0x80)) return true;
    }
    return false;
}

uint32_t le(const std::vector<uint8_t>& p, size_t i, int n) {
    uint32_t v = 0;
    for (int k = n - 1; k >= 0; k--) v = (v << 8) | p[i + k];
    return v;
}

bool parse(const std::vector<uint8_t>& p, Frame& f) {
    if (p.empty()) return false;
    f = Frame();
    f.type = p[0];
    size_t i;
    switch (f.type) {
    case F_HELLO:
        if (p.size() < 12 || p.size() != 12 + 4 * (size_t)p[11]) return false;
        f.phi2 = le(p, 2, 4);
        return true;
    case F_WRITES: {
        if (p.size() < 4) return false;
        f.seq = (uint16_t)le(p, 1, 2);
        i = 3;
        uint64_t t;
        if (!varint(p, i, t)) return false;
        while (i < p.size()) {
            uint64_t dt;
            if (!varint(p, i, dt) || i + 3 > p.size()) return false;
            Block b;
            t += dt;
            b.time = t;
            b.addr = (uint16_t)le(p, i, 2);
            size_t n = p[i + 2];
            i += 3;
            if (n == 0 || i + n > p.size()) return false;
            b.data.assign(p.begin() + i, p.begin() + i + n);
            i += n;
            f.blocks.push_back(std::move(b));
        }
        return true;
    }
    case F_STATUS:
        if (p.size() < 4) return false;
        f.seq = (uint16_t)le(p, 1, 2);
        i = 3;
        if (!varint(p, i, f.time)) return false;
        i += 14;                                    // dropped, out_dropped, ring_hiwater, writes
        if (p.size() == i + 12) {                   // version 3: the late counters
            f.late = le(p, i, 4);
            f.late250 = le(p, i + 4, 4);
            f.late1000 = le(p, i + 8, 4);
        } else if (p.size() != i) {
            return false;
        }
        return true;
    case F_BYE:
        if (p.size() != 2) return false;
        f.reason = p[1];
        return true;
    }
    return false;
}

// Feed raw channel bytes, take frames.  Text and broken frames between the 0x00 delimiters are skipped.
struct Decoder {
    std::vector<uint8_t> buf;
    uint32_t bad_frames = 0;

    template <class F> void feed(const uint8_t* data, size_t n, F on_frame) {
        for (size_t k = 0; k < n; k++) {
            uint8_t c = data[k];
            if (c != 0) {
                if (buf.size() < 4096) buf.push_back(c);
                continue;
            }
            if (!buf.empty()) {
                Frame f;
                if (decode(f)) on_frame(f);
                else bad_frames++;
                buf.clear();
            }
        }
    }

    bool decode(Frame& f) {
        std::vector<uint8_t> raw;                   // COBS
        size_t i = 0;
        while (i < buf.size()) {
            uint8_t code = buf[i++];
            if (i + code - 1 > buf.size()) return false;
            raw.insert(raw.end(), buf.begin() + i, buf.begin() + i + code - 1);
            i += code - 1;
            if (code != 0xFF && i < buf.size()) raw.push_back(0);
        }
        if (raw.size() < 3) return false;
        size_t n = raw.size() - 2;
        if (crc16(raw.data(), n) != (raw[n] | raw[n + 1] << 8)) return false;
        raw.resize(n);
        return parse(raw, f);
    }
};

// ---- encoding (the synthetic bench streams) ------------------------------------------------------

void put_varint(std::vector<uint8_t>& o, uint64_t v) {
    do {
        uint8_t b = v & 0x7F;
        v >>= 7;
        o.push_back(b | (v ? 0x80 : 0));
    } while (v);
}

void put_le(std::vector<uint8_t>& o, uint32_t v, int n) {
    for (int k = 0; k < n; k++) o.push_back((uint8_t)(v >> (8 * k)));
}

void put_frame(std::vector<uint8_t>& out, std::vector<uint8_t> p) {
    uint16_t crc = crc16(p.data(), p.size());
    p.push_back(crc & 0xFF);
    p.push_back(crc >> 8);
    out.push_back(0);
    size_t code_at = out.size();
    out.push_back(0);
    uint8_t code = 1;
    for (uint8_t b : p) {
        if (b == 0) {
            out[code_at] = code;
            code_at = out.size();
            out.push_back(0);
            code = 1;
        } else {
            out.push_back(b);
            if (++code == 0xFF) {
                out[code_at] = code;
                code_at = out.size();
                out.push_back(0);
                code = 1;
            }
        }
    }
    out[code_at] = code;
    out.push_back(0);
}

// The same streams as host/sidplay.py bench_stream: a 50 Hz tune (~1250 writes/s), plus with
// digi_hz a $D418 write digi_hz times a second.
std::vector<uint8_t> bench_stream(double seconds, int digi_hz, uint16_t base, uint32_t phi2) {
    std::vector<Block> blocks;
    uint64_t frame_cycles = phi2 / 50;
    for (int n = 0; n < (int)(seconds * 50); n++) {
        uint64_t t = 1000 + n * frame_cycles;
        for (int v = 0; v < 3; v++) {
            int f = 2000 + 300 * v + 37 * (n % 32);
            blocks.push_back({t + 40 * (uint64_t)v, (uint16_t)(base + 7 * v),
                              {(uint8_t)(f & 0xFF), (uint8_t)(f >> 8), 0x00, 0x08,
                               (uint8_t)(n % 8 ? 0x41 : 0x40), 0x09, 0xA9}});
        }
        blocks.push_back({t + 200, (uint16_t)(base + 0x15), {0x00, (uint8_t)(0x20 + n % 64), 0xF7, 0x1F}});
    }
    if (digi_hz) {
        double step = (double)phi2 / digi_hz;
        for (long k = 0; k * step < seconds * phi2; k++)
            blocks.push_back({1500 + (uint64_t)(k * step), (uint16_t)(base + 0x18), {(uint8_t)(k % 16)}});
    }
    std::stable_sort(blocks.begin(), blocks.end(), [](const Block& a, const Block& b) { return a.time < b.time; });
    uint64_t end = 1000 + (uint64_t)(seconds * phi2);
    std::vector<uint8_t> w, p;
    p = {F_HELLO, 3};
    put_le(p, phi2, 4);
    put_le(p, 600000000, 4);
    p.push_back(0);
    p.push_back(1);
    put_le(p, base, 2);
    put_le(p, base + 0x1C, 2);
    put_frame(w, p);
    uint16_t seq = 0;
    for (size_t i = 0; i < blocks.size(); i += 20, seq++) {
        p = {F_WRITES};
        put_le(p, seq, 2);
        uint64_t prev = blocks[i].time;
        put_varint(p, prev);
        for (size_t k = i; k < blocks.size() && k < i + 20; k++) {
            put_varint(p, blocks[k].time - prev);
            put_le(p, blocks[k].addr, 2);
            p.push_back((uint8_t)blocks[k].data.size());
            p.insert(p.end(), blocks[k].data.begin(), blocks[k].data.end());
            prev = blocks[k].time;
        }
        put_frame(w, p);
    }
    p = {F_STATUS};
    put_le(p, seq, 2);
    put_varint(p, end);
    for (int k = 0; k < 14 + 12; k++) p.push_back(0);
    put_frame(w, p);
    put_frame(w, {F_BYE, 0});
    return w;
}

// ---- rendering -----------------------------------------------------------------------------------

struct Options {
    std::vector<uint16_t> bases;
    int rate = 48000;
    bool m8580 = false;
    bool stereo = false;
    bool filter = true;
    reSID::sampling_method sampling = reSID::SAMPLE_INTERPOLATE;
};

const char* sampling_name(reSID::sampling_method m) {
    switch (m) {
    case reSID::SAMPLE_FAST: return "fast";
    case reSID::SAMPLE_INTERPOLATE: return "interpolate";
    case reSID::SAMPLE_RESAMPLE: return "resample";
    default: return "resample-fastmem";
    }
}

struct Write {
    uint64_t time;
    uint8_t sid, reg, value;
};

// One reSID per SID base address, clocked on the C64's timeline; writes land on their exact cycle.
// Mono mixes the SIDs; stereo puts the odd-numbered ones (1st, 3rd...) left, the others right.
class Renderer {
public:
    explicit Renderer(const Options& o) : opt(o) { make(PHI2_PAL); }

    void make(uint32_t phi2_hz) {
        phi2 = phi2_hz;
        sids.clear();
        chans.clear();
        for (size_t i = 0; i < opt.bases.size(); i++) {
            std::unique_ptr<reSID::SID> s(new reSID::SID());
            s->set_chip_model(opt.m8580 ? reSID::MOS8580 : reSID::MOS6581);
            s->enable_filter(opt.filter);
            if (!s->set_sampling_parameters(phi2, opt.sampling, opt.rate)) {
                fprintf(stderr, "reSID refused %d Hz with %s sampling\n", opt.rate, sampling_name(opt.sampling));
                exit(1);
            }
            s->clock((reSID::cycle_count)(phi2 * WARMUP));
            sids.push_back(std::move(s));
            chans.push_back(std::vector<short>());
        }
        cursor = 0;
        queue.clear();
    }

    void push(const Frame& f) {
        if (f.type == F_HELLO) {
            make(f.phi2);
        } else if (f.type == F_WRITES) {
            for (const Block& b : f.blocks)
                for (size_t k = 0; k < b.data.size(); k++) write(b.time, b.addr + k, b.data[k]);
        }
    }

    void write(uint64_t t, unsigned addr, uint8_t value) {
        for (size_t i = 0; i < opt.bases.size(); i++) {
            unsigned reg = addr - opt.bases[i];
            if (addr >= opt.bases[i] && reg < (unsigned)SID_REGS) {
                if (t < cursor) {
                    late++;
                    t = cursor;
                }
                queue.push_back({t, (uint8_t)i, (uint8_t)reg, value});
                return;
            }
        }
        ignored++;
    }

    // Clock every SID up to C64 time t, applying the queued writes on their cycle; the samples are
    // appended to out (interleaved when stereo).
    void render_to(uint64_t t, std::vector<int16_t>& out) {
        while (!queue.empty() && queue.front().time <= t) {
            Write w = queue.front();
            queue.pop_front();
            clock(w.time);
            sids[w.sid]->write(w.reg, w.value);
        }
        clock(t);
        mix(out);
    }

    uint32_t phi2 = PHI2_PAL;
    uint64_t cursor = 0;
    uint32_t late = 0, ignored = 0;
    std::deque<Write> queue;

private:
    void clock(uint64_t t) {
        if (t <= cursor) return;
        for (size_t i = 0; i < sids.size(); i++) {
            reSID::cycle_count d = (reSID::cycle_count)(t - cursor);
            short buf[1024];
            while (d > 0) {
                int n = sids[i]->clock(d, buf, 1024);
                chans[i].insert(chans[i].end(), buf, buf + n);
            }
        }
        cursor = t;
    }

    void mix(std::vector<int16_t>& out) {
        size_t n = chans[0].size();
        for (auto& c : chans) n = std::min(n, c.size());
        if (sids.size() == 1 && !opt.stereo) {
            out.insert(out.end(), chans[0].begin(), chans[0].begin() + n);
        } else {
            for (size_t k = 0; k < n; k++) {
                int l = 0, r = 0, nl = 0, nr = 0;
                for (size_t i = 0; i < chans.size(); i++) {
                    if (i % 2 == 0) { l += chans[i][k]; nl++; }
                    else { r += chans[i][k]; nr++; }
                }
                if (!opt.stereo) {
                    out.push_back((int16_t)((l + r) / (int)chans.size()));
                } else {
                    l /= nl;
                    r = nr ? r / nr : l;
                    out.push_back((int16_t)l);
                    out.push_back((int16_t)r);
                }
            }
        }
        for (auto& c : chans) c.erase(c.begin(), c.begin() + n);
    }

    Options opt;
    std::vector<std::unique_ptr<reSID::SID>> sids;
    std::vector<std::vector<short>> chans;
};

double cpu_seconds() {
    timespec ts;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

std::vector<Frame> decode_all(const std::vector<uint8_t>& data, Decoder& dec) {
    std::vector<Frame> frames;
    dec.feed(data.data(), data.size(), [&](const Frame& f) { frames.push_back(f); });
    return frames;
}

// Render a stream the way live play will (decode, then render every step_ms of C64 time, feeding the
// frames that have "arrived" by then) as fast as possible; and the same span with no writes.
void bench(const char* name, const std::vector<uint8_t>& data, const Options& o) {
    Renderer r(o), e(o);                            // made outside the timing (warm-up)
    double t0 = cpu_seconds();
    Decoder dec;
    std::vector<Frame> frames = decode_all(data, dec);
    double t_dec = cpu_seconds() - t0;
    uint64_t end = 0, writes = 0;
    for (const Frame& f : frames) {
        if (f.type == F_WRITES && !f.blocks.empty()) {
            end = std::max(end, f.blocks.back().time);
            for (const Block& b : f.blocks) writes += b.data.size();
        } else if (f.type == F_STATUS) {
            end = std::max(end, f.time);
        }
    }
    if (!end) {
        printf("%s: no writes or STATUS in the stream\n", name);
        return;
    }
    uint64_t step = r.phi2 * 4 / 1000;
    std::vector<int16_t> out;
    out.reserve(o.rate / 100 * 2);
    t0 = cpu_seconds();
    size_t i = 0;
    for (uint64_t t = step; t < end + step; t += step) {
        while (i < frames.size() && !(frames[i].type == F_WRITES && !frames[i].blocks.empty()
                                      && frames[i].blocks[0].time > t)) {
            if (frames[i].type != F_HELLO) r.push(frames[i]);
            i++;
        }
        out.clear();
        r.render_to(std::min(t, end), out);
    }
    double t_play = cpu_seconds() - t0 + t_dec;
    t0 = cpu_seconds();
    for (uint64_t t = step; t < end + step; t += step) {
        out.clear();
        e.render_to(std::min(t, end), out);
    }
    double t_emu = cpu_seconds() - t0;
    double c64 = (double)end / r.phi2;
    printf("%s: %.1f s of C64 time, %.0f writes/s: %.0f%% of one core (reSID, silent: %.0f%%)\n", name, c64,
           writes / c64, 100 * t_play / c64, 100 * t_emu / c64);
    fflush(stdout);
}

bool read_file(const char* path, std::vector<uint8_t>& data) {
    FILE* fh = fopen(path, "rb");
    if (!fh) return false;
    uint8_t buf[65536];
    size_t n;
    while ((n = fread(buf, 1, sizeof buf, fh)) > 0) data.insert(data.end(), buf, buf + n);
    fclose(fh);
    return true;
}

bool write_wav(const char* path, const std::vector<int16_t>& s, int rate, int channels) {
    FILE* fh = fopen(path, "wb");
    if (!fh) return false;
    std::vector<uint8_t> h;
    uint32_t bytes = (uint32_t)(s.size() * 2);
    h.insert(h.end(), {'R', 'I', 'F', 'F'});
    put_le(h, 36 + bytes, 4);
    h.insert(h.end(), {'W', 'A', 'V', 'E', 'f', 'm', 't', ' '});
    put_le(h, 16, 4);
    put_le(h, 1, 2);
    put_le(h, channels, 2);
    put_le(h, rate, 4);
    put_le(h, rate * channels * 2, 4);
    put_le(h, channels * 2, 2);
    put_le(h, 16, 2);
    h.insert(h.end(), {'d', 'a', 't', 'a'});
    put_le(h, bytes, 4);
    fwrite(h.data(), 1, h.size(), fh);
    for (int16_t v : s) {                           // little-endian whatever the host
        uint8_t b[2] = {(uint8_t)(v & 0xFF), (uint8_t)((uint16_t)v >> 8)};
        fwrite(b, 1, 2, fh);
    }
    return fclose(fh) == 0;
}

// A whole recording on the C64's own timeline, ending at the last time it knows (as render_stream).
void replay(const std::vector<uint8_t>& data, const Options& o, const char* wav) {
    Renderer r(o);
    Decoder dec;
    std::vector<int16_t> out;
    uint64_t end = 0;
    bool started = false;
    dec.feed(data.data(), data.size(), [&](const Frame& f) {
        if (f.type == F_HELLO && started) {
            r.render_to(end, out);
            end = 0;
        }
        started = true;
        r.push(f);
        if (f.type == F_WRITES && !f.blocks.empty()) end = std::max(end, f.blocks.back().time);
        else if (f.type == F_STATUS) end = std::max(end, f.time);
        if (!r.queue.empty() && r.queue.back().time - r.cursor > r.phi2)
            r.render_to(r.queue.back().time - r.phi2 / 2, out);
    });
    r.render_to(end, out);
    int ch = o.stereo ? 2 : 1;
    printf("# %.2f s, %u writes outside the SIDs, %u late, %u bad frames\n", (double)out.size() / ch / o.rate,
           r.ignored, r.late, dec.bad_frames);
    if (wav) {
        if (!write_wav(wav, out, o.rate, ch)) {
            fprintf(stderr, "cannot write %s\n", wav);
            exit(1);
        }
        printf("# wrote %s\n", wav);
    }
}

// ---- live play -----------------------------------------------------------------------------------

double wall_seconds() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Where "now" is on the C64's timeline.  Each frame says the TeensyROM had reached at least its time
// when it was sent, so time/phi2 - arrival is at most the true offset of the two clocks: keep the
// largest of the last `window` seconds (least transport delay) and extrapolate with the nominal phi2.
struct LiveClock {
    double phi2;
    double window = 4.0;
    std::deque<std::pair<double, double>> marks;    // (wall, offset seconds); marks[0] is the best

    explicit LiveClock(double hz) : phi2(hz) {}

    void seen(uint64_t c64, double wall) {
        double off = c64 / phi2 - wall;
        while (!marks.empty() && marks.back().second <= off) marks.pop_back();
        marks.push_back({wall, off});
        trim(wall);
    }

    void trim(double wall) {
        while (marks.size() > 1 && wall - marks.front().first > window) marks.pop_front();
    }

    bool now(double wall, int64_t& c64) {
        if (marks.empty()) return false;
        trim(wall);
        c64 = (int64_t)((marks.front().second + wall) * phi2);
        return true;
    }
};

// Renderer + clock + a sample FIFO with slow drift correction.  feed() from the network thread,
// pump() from the render thread, fill() from the audio thread (or drain() when only recording).
class LivePlayer {
public:
    const double AUTO_START = 0.06, AUTO_MIN = 0.015, AUTO_MAX = 0.5, AUTO_MARGIN = 0.010;
    const double AUTO_WINDOW = 6.0, AUTO_UP = 0.05, AUTO_DOWN = 0.010;

    LivePlayer(const Options& o, double latency, double buffer)
        : r(o), clock(PHI2_PAL), auto_latency(latency < 0), latency(latency < 0 ? AUTO_START : latency),
          channels(o.stereo ? 2 : 1), target((size_t)(buffer * o.rate)), last(2, 0) {}

    void feed(const Frame& f) {
        std::lock_guard<std::mutex> g(lock);
        double wall = wall_seconds();
        if (f.type == F_HELLO) {
            r.push(f);
            clock = LiveClock(f.phi2);
            clock.seen(0, wall);
            fifo.clear();
            started = false;
            return;
        }
        r.push(f);
        uint64_t oldest;
        if (f.type == F_WRITES && !f.blocks.empty()) {
            clock.seen(f.blocks.back().time, wall);
            oldest = f.blocks.front().time;
        } else if (f.type == F_STATUS) {
            clock.seen(f.time, wall);
            oldest = f.time;
        } else {
            return;
        }
        int64_t now = 0;
        clock.now(wall, now);
        delays.push_back({wall, std::max(0.0, (now - (double)oldest) / r.phi2)});
        while (!delays.empty() && wall - delays.front().first > AUTO_WINDOW) delays.pop_front();
    }

    // Render what is due.  Call every few ms.
    void pump() {
        std::lock_guard<std::mutex> g(lock);
        double wall = wall_seconds();
        if (auto_latency) adjust_latency(wall);
        int64_t now;
        if (!clock.now(wall, now)) return;
        int64_t t = now - (int64_t)(latency * r.phi2);
        if (t <= (int64_t)r.cursor) return;
        if ((t - (int64_t)r.cursor) > (int64_t)r.phi2) {      // a stall (or the start): jump, keep the state right
            scratch.clear();
            r.render_to(t - r.phi2 / 5, scratch);
        }
        size_t before = fifo.size();
        r.render_to(t, fifo);
        if (fifo.size() > before && fifo.size() / channels >= target) started = true;
    }

    // n frames for the sound card, interleaved.  Holds the FIFO near `buffer` by taking one frame more
    // or less per call (the C64's crystal against the sound card's: ~100 ppm).
    void fill(int16_t* out, size_t n) {
        std::lock_guard<std::mutex> g(lock);
        if (!started) {
            std::fill(out, out + n * channels, 0);
            return;
        }
        size_t have = fifo.size() / channels;
        size_t want = n;
        if (have > target * 5 / 4) {
            want = n + 1;
            dropped++;
        } else if (have < target * 3 / 4) {
            want = n - 1;
            repeated++;
        }
        size_t got = std::min(want, have);
        if (got < want) underruns++;
        size_t skip = (want == n + 1 && got == want) ? 1 : 0;
        size_t use = std::min(n, got - skip);
        std::copy(fifo.begin() + skip * channels, fifo.begin() + (skip + use) * channels, out);
        fifo.erase(fifo.begin(), fifo.begin() + got * channels);
        if (use) std::copy(out + (use - 1) * channels, out + use * channels, last.begin());
        for (size_t k = use; k < n; k++) std::copy(last.begin(), last.begin() + channels, out + k * channels);
    }

    // Everything rendered so far, no drift correction (recording without a sound card).
    void drain(std::vector<int16_t>& out) {
        std::lock_guard<std::mutex> g(lock);
        out.insert(out.end(), fifo.begin(), fifo.end());
        fifo.clear();
    }

    Renderer r;
    LiveClock clock;
    bool auto_latency;
    double latency;
    uint32_t underruns = 0, dropped = 0, repeated = 0;
    std::mutex lock;

private:
    void adjust_latency(double wall) {
        if (last_adjust < 0) {
            last_adjust = wall;
            return;
        }
        double dt = wall - last_adjust;
        if (dt < 0.1 || delays.size() < 20) return;
        last_adjust = wall;
        std::vector<double> d;
        for (auto& x : delays) d.push_back(x.second);
        size_t k = std::min(d.size() - 1, (size_t)(0.99 * d.size()));
        std::nth_element(d.begin(), d.begin() + k, d.end());
        double goal = std::min(AUTO_MAX, std::max(AUTO_MIN, d[k] + AUTO_MARGIN));
        if (goal > latency) latency = std::min(goal, latency + AUTO_UP * dt);
        else latency = std::max(goal, latency - AUTO_DOWN * dt);
    }

    size_t channels, target;
    std::vector<int16_t> fifo, scratch, last;
    std::deque<std::pair<double, double>> delays;   // (wall, seconds a frame's oldest write had waited)
    double last_adjust = -1;
    bool started = false;
};

// The TeensyROM's channel: TCP (the snoop data port 2113, else 2112) or USB serial.
struct Channel {
    int fd = -1;

    bool open_tcp(const char* host, int port) {
        int ports[2] = {port ? port : 2113, port ? 0 : 2112};
        for (int p : ports) {
            if (!p) break;
            addrinfo hints = {}, *res = nullptr;
            hints.ai_family = AF_UNSPEC;
            hints.ai_socktype = SOCK_STREAM;
            char ps[8];
            snprintf(ps, sizeof ps, "%d", p);
            if (getaddrinfo(host, ps, &hints, &res) != 0) {
                fprintf(stderr, "cannot resolve %s\n", host);
                return false;
            }
            int err = 0;
            for (addrinfo* a = res; a; a = a->ai_next) {
                fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
                if (fd >= 0 && connect(fd, a->ai_addr, a->ai_addrlen) == 0) break;
                err = errno;
                if (fd >= 0) close(fd);
                fd = -1;
            }
            freeaddrinfo(res);
            if (fd >= 0) {
                int one = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
                return true;
            }
            if (p == 2113 && err == ECONNREFUSED && !port) {
                fprintf(stderr, "# port 2113 refused (firmware before 0.9-sn4?): using 2112, which other tools "
                                "then cannot use during the capture\n");
                continue;
            }
            fprintf(stderr, "cannot connect to %s:%d: %s\n", host, p, strerror(err));
            return false;
        }
        return false;
    }

    bool open_serial(const char* path) {
        fd = open(path, O_RDWR | O_NOCTTY);
        if (fd < 0) {
            fprintf(stderr, "cannot open %s: %s\n", path, strerror(errno));
            return false;
        }
        termios t;
        if (tcgetattr(fd, &t) == 0) {
            cfmakeraw(&t);
            cfsetispeed(&t, B115200);                // USB CDC ignores it
            cfsetospeed(&t, B115200);
            t.c_cflag |= CLOCAL | CREAD;
            tcsetattr(fd, TCSANOW, &t);
        }
        return true;
    }

    bool write_all(const std::vector<uint8_t>& b) {
        size_t off = 0;
        while (off < b.size()) {
            ssize_t n = ::write(fd, b.data() + off, b.size() - off);
            if (n <= 0) return false;
            off += n;
        }
        return true;
    }

    // Up to n bytes; 0 after timeout_ms with nothing; -1 when the channel is gone.
    ssize_t read_some(uint8_t* buf, size_t n, int timeout_ms) {
        pollfd p = {fd, POLLIN, 0};
        int k = poll(&p, 1, timeout_ms);
        if (k < 0) return errno == EINTR ? 0 : -1;
        if (k == 0) return 0;
        ssize_t got = ::read(fd, buf, n);
        return got > 0 ? got : -1;
    }
};

std::vector<uint8_t> start_command(const std::vector<uint16_t>& bases, int send_ms) {
    std::vector<uint8_t> c = {0x64, 0x53, 0x01, (uint8_t)bases.size()};
    for (uint16_t b : bases) {
        put_le(c, b, 2);
        put_le(c, b + 0x1C, 2);
    }
    c.push_back(0);                                 // gap 0: every write its own exact time
    c.push_back((uint8_t)send_ms);
    return c;
}

// Read until AckToken (CC 64) or FailToken (7F 9B).  rest: what followed the Ack (the stream's start).
bool wait_ack(Channel& ch, std::vector<uint8_t>& rest) {
    std::vector<uint8_t> buf;
    double end = wall_seconds() + 3.0;
    uint8_t tmp[4096];
    while (wall_seconds() < end) {
        ssize_t n = ch.read_some(tmp, sizeof tmp, 100);
        if (n < 0) break;
        buf.insert(buf.end(), tmp, tmp + n);
        for (size_t i = 0; i + 1 < buf.size(); i++) {
            if (buf[i] == 0xCC && buf[i + 1] == 0x64) {
                rest.assign(buf.begin() + i + 2, buf.end());
                return true;
            }
            if (buf[i] == 0x7F && buf[i + 1] == 0x9B) {
                n = ch.read_some(tmp, sizeof tmp, 200);
                if (n > 0) buf.insert(buf.end(), tmp, tmp + n);
                std::string msg(buf.begin() + i + 2, buf.end());
                fprintf(stderr, "TeensyROM refused: %s\n", msg.c_str());
                return false;
            }
        }
    }
    fprintf(stderr, "no answer from the TeensyROM (firmware without teensyrom-snoop?)\n");
    return false;
}

std::atomic<bool> g_stop(false);

void on_signal(int) { g_stop = true; }

struct LiveArgs {
    const char* tcp = nullptr;
    const char* serial = nullptr;
    int tcp_port = 0;
    int send_ms = 0;
    double latency = -1;                            // < 0: automatic
    double buffer = 0.03;
    double seconds = 0;
    bool play = true;
    bool isr = false;
    const char* wav = nullptr;
    const char* device = "default";
};

#if HAVE_ALSA
snd_pcm_t* open_alsa(const LiveArgs& a, const Options& o) {
    snd_pcm_t* pcm = nullptr;
    int err = snd_pcm_open(&pcm, a.device, SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        fprintf(stderr, "cannot open sound device %s: %s (record only: --no-play --wav)\n", a.device, snd_strerror(err));
        return nullptr;
    }
    err = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16, SND_PCM_ACCESS_RW_INTERLEAVED, o.stereo ? 2 : 1, o.rate, 1,
                             40000);           // 40 ms of sound card buffer
    if (err < 0) {
        fprintf(stderr, "sound device %s: %s\n", a.device, snd_strerror(err));
        snd_pcm_close(pcm);
        return nullptr;
    }
    return pcm;
}
#endif

int live(const LiveArgs& a, const Options& o) {
    Channel ch;
    if (a.tcp ? !ch.open_tcp(a.tcp, a.tcp_port) : !ch.open_serial(a.serial)) return 1;
#if HAVE_ALSA
    snd_pcm_t* pcm = nullptr;
    if (a.play && !(pcm = open_alsa(a, o))) return 1;
#else
    if (a.play) {
        fprintf(stderr, "built without ALSA (install libasound2-dev and rebuild): record with --no-play --wav\n");
        return 1;
    }
#endif
    if (!ch.write_all(start_command(o.bases, a.send_ms))) {
        fprintf(stderr, "cannot send the start command\n");
        return 1;
    }
    std::vector<uint8_t> rest;
    if (!wait_ack(ch, rest)) return 1;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);
    LivePlayer player(o, a.latency, a.buffer);
    int channels = o.stereo ? 2 : 1;
    std::vector<int16_t> recorded;
    std::mutex rec_lock;
    std::atomic<bool> stop_threads(false);

    std::thread pump([&]() {
        while (!stop_threads) {
            player.pump();
            if (!a.play && a.wav) {
                std::lock_guard<std::mutex> g(rec_lock);
                player.drain(recorded);
            } else if (!a.play) {
                std::vector<int16_t> sink;
                player.drain(sink);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    });
#if HAVE_ALSA
    std::thread audio;
    if (pcm) {
        audio = std::thread([&]() {
            std::vector<int16_t> buf(256 * channels);
            while (!stop_threads) {
                player.fill(buf.data(), 256);
                if (a.wav) {
                    std::lock_guard<std::mutex> g(rec_lock);
                    recorded.insert(recorded.end(), buf.begin(), buf.end());
                }
                snd_pcm_sframes_t n = snd_pcm_writei(pcm, buf.data(), 256);
                if (n < 0 && snd_pcm_recover(pcm, (int)n, 1) < 0) {
                    fprintf(stderr, "sound device: %s\n", snd_strerror((int)n));
                    break;
                }
            }
        });
    }
#endif

    Decoder dec;
    Frame last_status;
    bool have_status = false;
    uint32_t lost = 0;
    int next_seq = -1;
    double end = a.seconds > 0 ? wall_seconds() + a.seconds : 0;
    bool bye = false;
    auto on_frame = [&](const Frame& f) {
        if (f.type == F_HELLO) next_seq = 0;
        if (f.type == F_WRITES && next_seq >= 0) {
            lost += (uint16_t)(f.seq - next_seq);
            next_seq = (uint16_t)(f.seq + 1);
        }
        player.feed(f);
        if (f.type == F_STATUS) {
            if (a.isr && have_status) {
                double secs = std::max((double)(f.time - last_status.time) / player.r.phi2, 1e-3);
                printf("# latency %4.0f ms  late bus interrupts/s: %7.1f  >250ns: %6.1f  >1us: %6.1f\n",
                       player.latency * 1000, (f.late - last_status.late) / secs,
                       (f.late250 - last_status.late250) / secs, (f.late1000 - last_status.late1000) / secs);
                fflush(stdout);
            }
            last_status = f;
            have_status = true;
        }
        if (f.type == F_BYE) bye = true;
    };
    dec.feed(rest.data(), rest.size(), on_frame);
    uint8_t buf[4096];
    while (!g_stop && !bye && !(end && wall_seconds() > end)) {
        ssize_t n = ch.read_some(buf, sizeof buf, 200);
        if (n < 0) {
            fprintf(stderr, "# the TeensyROM closed the connection\n");
            break;
        }
        dec.feed(buf, (size_t)n, on_frame);
    }
    ch.write_all({0x64, 0x53, 0x00});              // stop
    stop_threads = true;
    pump.join();
#if HAVE_ALSA
    if (audio.joinable()) audio.join();
    if (pcm) {
        snd_pcm_drop(pcm);
        snd_pcm_close(pcm);
    }
#endif
    close(ch.fd);

    printf("# latency %.0f ms%s, FIFO %.0f ms, sampling %s\n", player.latency * 1000,
           player.auto_latency ? " (automatic)" : "", a.buffer * 1000, sampling_name(o.sampling));
    printf("# writes late %u, underruns %u, drift: dropped %u repeated %u, lost frames %u, bad frames %u\n",
           player.r.late, player.underruns, player.dropped, player.repeated, lost, dec.bad_frames);
    if (have_status) {
        double secs = std::max((double)last_status.time / player.r.phi2, 1e-3);
        printf("# late bus interrupts in %.0f s: %u (>250ns %u, >1us %u)\n", secs, last_status.late,
               last_status.late250, last_status.late1000);
    }
    if (a.wav && !recorded.empty()) {
        if (!write_wav(a.wav, recorded, o.rate, channels)) {
            fprintf(stderr, "cannot write %s\n", a.wav);
            return 1;
        }
        printf("# wrote %s\n", a.wav);
    }
    return 0;
}

void usage() {
    fputs("usage: trsid-native --tcp HOST | --port TTY | --replay FILE | --bench   [options]\n"
          "  --tcp HOST          the TeensyROM's address (port 2113, else 2112); --tcp-port N to choose\n"
          "  --port TTY          its USB serial port (/dev/ttyACM0)\n"
          "  --replay FILE       a recording (host/trsnoop.py --raw): to --wav, or timed with --bench\n"
          "  --bench             time the player: a synthetic tune and an 8 kHz digi (or --replay FILE)\n"
          "  --wav OUT.wav       record what is played\n"
          "  --no-play           live: no sound card (with --wav: record only)\n"
          "  --device NAME       ALSA device (default: default)\n"
          "  --seconds S         live: stop after S seconds; bench: the synthetic streams' length (10)\n"
          "  --latency S         live: seconds behind the C64, fixed (default: automatic)\n"
          "  --buffer S          live: FIFO target, seconds (0.03)\n"
          "  --send-ms N         how often the TeensyROM sends (default 20 on USB serial, at once on TCP)\n"
          "  --isr               live: the TeensyROM's late bus-interrupt counts every second\n"
          "  --model 6581|8580  --no-filter  --rate HZ (48000)  --sid HEX (repeat; D400)  --stereo\n"
          "  --sampling fast|interpolate|resample   (live: fast; replay: interpolate; bench: all three)\n",
          stderr);
    exit(2);
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    LiveArgs la;
    bool do_bench = false, sampling_set = false;
    const char* replay_path = nullptr;
    double seconds = 0;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto val = [&]() -> const char* {
            if (i + 1 >= argc) usage();
            return argv[++i];
        };
        if (a == "--bench") do_bench = true;
        else if (a == "--replay") replay_path = val();
        else if (a == "--tcp") la.tcp = val();
        else if (a == "--port") la.serial = val();
        else if (a == "--tcp-port") la.tcp_port = atoi(val());
        else if (a == "--send-ms") la.send_ms = atoi(val());
        else if (a == "--latency") la.latency = atof(val());
        else if (a == "--buffer") la.buffer = atof(val());
        else if (a == "--no-play") la.play = false;
        else if (a == "--isr") la.isr = true;
        else if (a == "--device") la.device = val();
        else if (a == "--wav") la.wav = val();
        else if (a == "--seconds") seconds = atof(val());
        else if (a == "--rate") o.rate = atoi(val());
        else if (a == "--stereo") o.stereo = true;
        else if (a == "--no-filter") o.filter = false;
        else if (a == "--sid") {
            std::string s = val();
            if (!s.empty() && s[0] == '$') s = s.substr(1);
            o.bases.push_back((uint16_t)strtoul(s.c_str(), nullptr, 16));
        } else if (a == "--model") {
            std::string m = val();
            if (m != "6581" && m != "8580") usage();
            o.m8580 = m == "8580";
        } else if (a == "--sampling") {
            std::string m = val();
            sampling_set = true;
            if (m == "fast") o.sampling = reSID::SAMPLE_FAST;
            else if (m == "interpolate") o.sampling = reSID::SAMPLE_INTERPOLATE;
            else if (m == "resample") o.sampling = reSID::SAMPLE_RESAMPLE;
            else usage();
        } else {
            usage();
        }
    }
    if (o.bases.empty()) o.bases.push_back(0xD400);
    if ((la.tcp != nullptr) + (la.serial != nullptr) + (replay_path != nullptr && !do_bench) + do_bench != 1) usage();

    if (la.tcp || la.serial) {
        if (!sampling_set) o.sampling = reSID::SAMPLE_FAST;
        la.seconds = seconds;
        if (!la.play && !la.wav) fprintf(stderr, "# --no-play without --wav: nothing is kept\n");
        return live(la, o);
    }
    std::vector<uint8_t> data;
    if (replay_path && !read_file(replay_path, data)) {
        fprintf(stderr, "cannot read %s\n", replay_path);
        return 1;
    }
    if (!do_bench) {
        replay(data, o, la.wav);
        return 0;
    }
    std::vector<std::pair<std::string, std::vector<uint8_t>>> runs;
    if (replay_path) {
        runs.push_back({replay_path, data});
    } else {
        double s = seconds > 0 ? seconds : 10.0;
        runs.push_back({"tune (~1250 writes/s)", bench_stream(s, 0, o.bases[0], PHI2_PAL)});
        runs.push_back({"digi 8 kHz (~9250 writes/s)", bench_stream(s, 8000, o.bases[0], PHI2_PAL)});
    }
    std::vector<reSID::sampling_method> methods;
    if (sampling_set) methods.push_back(o.sampling);
    else methods = {reSID::SAMPLE_FAST, reSID::SAMPLE_INTERPOLATE, reSID::SAMPLE_RESAMPLE};
    for (reSID::sampling_method m : methods) {
        o.sampling = m;
        printf("# reSID %s, %s, filter %s, %d Hz, sampling %s, %zu SID%s\n", resid_version_string,
               o.m8580 ? "8580" : "6581", o.filter ? "on" : "off", o.rate, sampling_name(m), o.bases.size(),
               o.bases.size() > 1 ? "s" : "");
        for (auto& run : runs) bench(run.first.c_str(), run.second, o);
    }
    printf("# live play needs well under 100%%: aim for 70%% or less\n");
    return 0;
}
