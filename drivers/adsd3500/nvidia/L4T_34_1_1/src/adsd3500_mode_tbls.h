#ifndef __ADSD3500_I2C_TABLES__
#define __ADSD3500_I2C_TABLES__

enum {
    ADSD3500_MODE_256x320_30FPS,
    ADSD3500_MODE_1280x320_30FPS,
    ADSD3500_MODE_512x512_30FPS,
    ADSD3500_MODE_2560x512_30FPS,
    ADSD3500_MODE_512x640_30FPS,
    ADSD3500_MODE_2048x640_30FPS,
    ADSD3500_MODE_2560x640_30FPS,
};

static const int adsd3500_30fps[] = {
    30,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt adsd3500_frmfmt[] = {
    {{256, 320}, adsd3500_30fps, 1, 0, ADSD3500_MODE_256x320_30FPS},
    {{1280, 320}, adsd3500_30fps, 1, 0, ADSD3500_MODE_1280x320_30FPS},
    {{512, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_512x512_30FPS},
    {{2560, 512}, adsd3500_30fps, 1, 0, ADSD3500_MODE_2560x512_30FPS},
    {{512, 640}, adsd3500_30fps, 1, 0, ADSD3500_MODE_512x640_30FPS},
    {{2048, 640}, adsd3500_30fps, 1, 0, ADSD3500_MODE_2048x640_30FPS},
    {{2560, 640}, adsd3500_30fps, 1, 0, ADSD3500_MODE_2560x640_30FPS},
    /* Add modes with no device tree support after below */
};

#endif /* __ADSD3500_I2C_TABLES__ */
