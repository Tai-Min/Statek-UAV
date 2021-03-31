#include "clock.hpp"
#include <mbed.h>

namespace {
    Timer t;
}

void clockStart(void) {
    t.start();
}

unsigned long  clockNow(void) {
    return t.read_ms();
}