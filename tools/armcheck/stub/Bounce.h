#pragma once
#include <Arduino.h>
class Bounce { public: Bounce(uint8_t, unsigned long) {} bool update() { return false; } int read() { return 1; } };
