#ifndef FLOAT_TO_LIN_H
#define FLOAT_TO_LIN_H

static const uint16_t CNT = 2048;

static int16_t lookup[CNT] = {0};

static int16_t getRange(uint16_t idx) {

    bool neg = false;
    if (idx > (CNT / 2)) {
        idx = idx - (CNT / 2);
        neg = true;
    }

    int16_t value = 0;
    if (idx < 256)
        value = idx;
    else if (idx < 384)
        value = ((idx - 256) << 1) + 256;
    else if (idx < 512)
        value = ((idx - 384) << 2) + 512;
    else if (idx < 640)
        value = ((idx - 512) << 3) + 1024;
    else if (idx < 768)
        value = ((idx - 640) << 4) + 2048;
    else if (idx < 896)
        value = ((idx - 768) << 5) + 4096;
    else if (idx < 1024)
        value = ((idx - 896) << 6) + 8192;
    else if (idx == 1024)
        value = 32767;

    if (neg == true)
        value = -value;

    return value;
}

void FloatToLinGenerateTable() {
    for (uint16_t idx = 0; idx < CNT; idx++) {
        lookup[idx] = getRange(idx);
    }
}

int16_t Convert11bitFloat2LinearVal(uint16_t input) {
    return ((input < 2048) ? lookup[input] : 0);
}

#endif //FLOAT_TO_LIN_H