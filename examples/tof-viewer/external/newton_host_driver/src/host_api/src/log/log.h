#ifndef _LIBS_LOG_LOG_MAIN_H
#define _LIBS_LOG_LOG_MAIN_H

// These are here to satisfy log calls in str_parms.c
#define ALOGI(...) (adi_logi(__VA_ARGS__))
#define ALOGV(...) (adi_logv(__VA_ARGS__))

#endif /* _LIBS_LOG_LOG_MAIN_H */
