#ifndef FW_FILE_H
#define FW_FILE_H
#include <linux/fs.h>
// #include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

struct file *file_open(const char *path, int flags, int rights) ;
void file_close(struct file *file) ;
int file_sync(struct file *file);


#endif // FW_FILE_H