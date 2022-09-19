#ifndef _LIBC_SYS_TIME_H_
#define _LIBC_SYS_TIME_H_

#include_next <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define timeofday_save()    do { } while (0)
#define timeofday_restore() do { } while (0)

#ifdef __cplusplus
}
#endif

#endif /* _LIBC_SYS_TIME_H_ */
