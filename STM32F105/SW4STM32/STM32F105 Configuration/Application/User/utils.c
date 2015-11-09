#include "utils.h"

uint8_t ascii2byte(uint8_t val)
{
    if (val >= 'a') {
        return val - 'a' + 10; // convert chars a-f
    }
    if (val >= 'A') {
        return val - 'A' + 10; // convert chars A-F
    }
    return val - '0';     // convert chars 0-9
}

uint8_t nibble2ascii(uint8_t byte) {
    byte &= 0x0F;
    return byte < 10 ? byte + 48 : byte + 55;
}
