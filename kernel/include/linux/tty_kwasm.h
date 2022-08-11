
#ifndef _LINUX_TTY_KWASM_H_
#define _LINUX_TTY_KWASM_H_

#include <linux/kfifo.h>
#include <linux/types.h>
#include <linux/tty.h>
#include <linux/wait.h>

#define KWASM_KEY_SIZE 4
#define KWASM_HSIZE 8
#define KWASM_TSIZE 4
#define _kw_max_cmd 10

typedef  enum{
	_no_start = 0,
	_parsing_head,
	_receiving_data,
	_receiving_data_ok,
}_kwasm_rstate;

typedef struct _kw_cmd{
    uint64_t pbuf;
    uint64_t blen;
    uint64_t ccode;
}_kw_cmd;

int _kw_out_kwasm_cmd(_kw_cmd *o);
int _kw_receive_off(void);
int _kw_receive_on(void);
wait_queue_head_t *_kw_get_wait_queue(void);
char *_kw_test_export(int i, long long l, char *ic, void *buf, int buff_len);
int tty_kwasm_init(void);
int _kw_parse_kwasm_cmd(struct tty_struct * tty,const char *buf, ssize_t s);

#endif

