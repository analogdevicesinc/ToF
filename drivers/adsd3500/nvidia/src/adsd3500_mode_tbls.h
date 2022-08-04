#ifndef __ADSD3500_I2C_TABLES__
#define __ADSD3500_I2C_TABLES__

#define ADSD3500_TABLE_WAIT_MS 0
#define ADSD3500_TABLE_END 1

enum {
    ADSD3500_MODE_512x512_30FPS,
    ADSD3500_MODE_1024x512_30FPS,
    ADSD3500_MODE_1280x512_30FPS,
    ADSD3500_MODE_1536x512_30FPS,
    ADSD3500_MODE_1792x512_30FPS,
    ADSD3500_MODE_2048x512_30FPS,
    ADSD3500_MODE_2304x512_30FPS,
    ADSD3500_MODE_2560x512_30FPS,
    ADSD3500_MODE_3072x1024_30FPS,
};

static const int adsd3500_30fps[] = {
    30,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt adsd3500_frmfmt[] = {
    {{512, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_512x512_30FPS},
    {{1024, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_1024x512_30FPS},
    {{1280, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_1280x512_30FPS},
    {{1536, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_1536x512_30FPS},
    {{1792, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_1792x512_30FPS},
    {{2048, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_2048x512_30FPS},
    {{2304, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_2304x512_30FPS},
    {{2560, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_2560x512_30FPS},
    {{3072, 1024}, adsd3500_30fps, 1, 0, ADSD3500_MODE_3072x1024_30FPS},
    /* Add modes with no device tree support after below */
};

#endif /* __ADSD3500_I2C_TABLES__ */
