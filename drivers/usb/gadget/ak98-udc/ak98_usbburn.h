#ifndef __AK98_USBBURN__
#define __AK98_USBBURN__

extern int sense_data;

extern struct semaphore sense_data_lock;

extern int usbburn_write(void *buf, size_t count);

extern int usbburn_read(void *buf, size_t count);

extern void usbburn_ioctl(void);

#endif
