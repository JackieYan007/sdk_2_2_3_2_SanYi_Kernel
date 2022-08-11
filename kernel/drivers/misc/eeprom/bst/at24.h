#ifndef AT24_H
#define AT24_H

int bst_at24_write(void *priv, unsigned int off, void *val, size_t count);
int bst_at24_read(void *priv, unsigned int off, void *val, size_t count);

#endif