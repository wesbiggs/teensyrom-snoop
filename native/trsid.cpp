// trsid-native: the teensyrom-snoop SID player in C++ on classic reSID, for computers too slow for
// host/trsid.py (a Raspberry Pi Zero: reSIDfp alone needs more than its one core).
//
//   trsid-native --bench                         time the player on a synthetic tune and an 8 kHz digi
//   trsid-native --bench --replay sid.snoop      ...or on a recording (host/trsnoop.py --raw)
//   trsid-native --replay sid.snoop --wav x.wav  a recording to WAV
//
// Options: --model 6581|8580, --sampling fast|interpolate|resample, --no-filter, --rate HZ,
// --sid HEX (repeat), --stereo, --seconds S (synthetic bench length).
// The wire format is docs/PROTOCOL.md; the rendering follows host/sidplay.py (SidRenderer).
// reSID (GPL-2.0-or-later) is fetched and compiled by native/Makefile.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <memory>
#include <string>
#include <vector>

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
    printf("%s: %.1f s of C64 time, %.0f writes/s: %.0f%% of one core (reSID alone %.0f%%)\n", name, c64,
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

void usage() {
    fputs("usage: trsid-native --bench [--replay FILE] | --replay FILE --wav OUT.wav\n"
          "  [--model 6581|8580] [--sampling fast|interpolate|resample] [--no-filter] [--rate HZ]\n"
          "  [--sid HEX ...] [--stereo] [--seconds S]\n"
          "--bench with no --sampling times all three sampling methods.\n", stderr);
    exit(2);
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    bool do_bench = false, sampling_set = false;
    const char* replay_path = nullptr;
    const char* wav = nullptr;
    double seconds = 10.0;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto val = [&]() -> const char* {
            if (i + 1 >= argc) usage();
            return argv[++i];
        };
        if (a == "--bench") do_bench = true;
        else if (a == "--replay") replay_path = val();
        else if (a == "--wav") wav = val();
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
    if (!do_bench && !replay_path) usage();

    std::vector<uint8_t> data;
    if (replay_path && !read_file(replay_path, data)) {
        fprintf(stderr, "cannot read %s\n", replay_path);
        return 1;
    }
    if (!do_bench) {
        replay(data, o, wav);
        return 0;
    }
    std::vector<std::pair<std::string, std::vector<uint8_t>>> runs;
    if (replay_path) {
        runs.push_back({replay_path, data});
    } else {
        runs.push_back({"tune (~1250 writes/s)", bench_stream(seconds, 0, o.bases[0], PHI2_PAL)});
        runs.push_back({"digi 8 kHz (~9250 writes/s)", bench_stream(seconds, 8000, o.bases[0], PHI2_PAL)});
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
