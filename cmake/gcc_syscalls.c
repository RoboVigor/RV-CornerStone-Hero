/*
 * GCC-only newlib glue (not used by Keil).
 *
 * Under ARMCC, printf ends in the fputc() defined in library/atom/usart/usart.c (-> USART6).
 * newlib's printf never calls fputc; it calls _write(). Forward _write to that same fputc so
 * printf output keeps going to USART6 exactly as in the Keil build.
 * The remaining syscalls come from libnosys (--specs=nosys.specs).
 */
#include <stdio.h>

int fputc(int ch, FILE *f);

int _write(int fd, const char *buf, int len) {
    (void) fd;
    for (int i = 0; i < len; i++) {
        fputc((unsigned char) buf[i], stdout);
    }
    return len;
}
