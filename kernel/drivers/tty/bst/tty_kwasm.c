
#include <linux/tty_kwasm.h>
#include <linux/slab.h>
#include <linux/kern_levels.h>
#include <linux/wait.h>
#include <asm/current.h>
#include <linux/timer.h>


#undef TTY_KWASM_DEBUG
#define TTY_KWASM_DEBUG

#ifdef TTY_KWASM_DEBUG
#define _kwprint(fmt,...)  printk(KERN_DEBUG "[%s][%d]:"fmt,__func__,__LINE__, ##__VA_ARGS__)
#define _kwerr(fmt,...)  printk(KERN_ERR "[%s][%d]:"fmt,__func__,__LINE__, ##__VA_ARGS__)
#else
#define _kwprint(fmt,...)
#define _kwerr(fmt,...) printk(KERN_ERR "[%s][%d]:"fmt,__func__,__LINE__, ##__VA_ARGS__)
#endif

static int _kwasm_on = -1;
static int _kwasm_on_complete = 0;
// -1 off
// 0 on

/*
 * state and return value for interface of parsing.
 */
#define _kw_ignore        -1
#define _kw_abnormal      -2
#define _no_start          0
#define _parsing_head      1
#define _receiving_data    2
#define _receiving_data_ok 3

int _buf_len = (1024*1024);
char _KWASM_HEAD[KWASM_KEY_SIZE]={ 0x1B, 0x5B, 0x57, 0x5D };// \033 [ W ] 27 91 87 93

struct __kfifo kwasm_fifo;
struct spinlock kf_lock;
wait_queue_head_t rd_wait;

#define _fifo_size   8
#define _max_fds     3
#define _kwasm_bsize KWASM_HSIZE
char khead_buff[_kwasm_bsize+1];
static int _received = 0;
int _now_state;
struct spinlock state_lock;

int _restore_tty = 0;
unsigned char _cur_cmd = 0;
static unsigned char _c1;
static unsigned char _c2;
static unsigned char _c3;
static unsigned int _length;
char * _file_buff=NULL;// vfree by user(kwasm).
int   _cur_flen;
static struct timer_list kw_rtimer;
static int rtimer_count;
static int last_rec=0;

int _kw_in_kwasm_cmd(char *buf, ssize_t s);

/*
 * timer thread maybe set state,so need lock..
 */
static void set_state(int  new_state){
    spin_lock_irq(&state_lock);
    _now_state = new_state;
    spin_unlock_irq(&state_lock);
}

static int get_state(void)
{
    _kwasm_rstate s;
    spin_lock_irq(&state_lock);
    s = _now_state;
    spin_unlock_irq(&state_lock);
    return s;
}


//    _file_buff = NULL;
#define _kw_clear_state(c)\
do{\
    _received = 0;\
    set_state(_no_start);\
    _restore_tty = 0;\
    _cur_cmd = 0;\
    _c1 = 0;\
    _c2 = 0;\
    _c3 = 0;\
    _length = 0;\
    _cur_flen = 0;\
    if(c == _KWASM_HEAD[_received]){\
        khead_buff[_received++]=c;\
        set_state(_parsing_head);\
    }\
    last_rec = 0;\
}while(0)

#if 0
/* for test */
#define _print_state() \
    do{\
        _kwprint("state: %d\n",_now_state);\
        _kwprint("received num: %d\n",_received);\
        _kwprint("received str: [%s]\n",khead_buff);\
        int q = 0;\
        for(;q<_received;q++)\
            _kwprint("received char: [%d = 0x%x]\n",khead_buff[q],khead_buff[q]);\
    }while(0)
#else
#define _print_state()
#endif

/* for test */
void _kw_print_fifo(void){
    int i;
    int l;
    uint64_t *d;
    int tail_bufs;
    int cunt =0;
    int j;

    d = (uint64_t *)kwasm_fifo.data;
    i = kwasm_fifo.out;
    l = kwasm_fifo.in - kwasm_fifo.out;
    i = kwasm_fifo.out &kwasm_fifo.mask;
    tail_bufs = kwasm_fifo.mask+1-i;

    _kwprint("fifo in  :%d",kwasm_fifo.in);
    _kwprint("fifo out :%d",kwasm_fifo.out);
    _kwprint("fifo mask:%d",kwasm_fifo.mask);
    _kwprint("fifo esize:%d",kwasm_fifo.esize);

    while(cunt<l && i<=kwasm_fifo.mask){
        _kwprint("i:%d ,<%s>, len:%lld, cmd:%lld\n",i, d[i*3]?(char *)d[i*3]:"NULL buff",d[i*3+1],d[i*3+2]);
        i++;
        cunt++;
    }

    if(tail_bufs < l){
        int head_bufs;
        head_bufs = l -tail_bufs;
        j =0;
        while(j<head_bufs ){
            _kwprint("i:%d ,<%s>, len:%lld, cmd:%lld\n",j, d[j*3]?(char *)d[j*3]:"NULL buff",d[j*3+1],d[i*3+2]);
            j++;
        }
    }
}

/* unused */
void _kw_echo_close(struct tty_struct * tty){
    struct ktermios _flags;

    if(!(tty->termios.c_lflag & ECHO))
        return;

    memset(&_flags, 0, sizeof(_flags));
    
    _flags = tty->termios;
    _flags.c_lflag &=~ECHO;
    tty_set_termios(tty, &_flags);
    _restore_tty = 1;

    _kwprint("close echo..\n");
    return;
}

/* unused */
void _kw_echo_restore(struct tty_struct * tty){
    struct ktermios _flags;

    if(!_restore_tty)
        return;
    
    memset(&_flags, 0, sizeof(_flags));
    
    _flags = tty->termios;
    _flags.c_lflag &=ECHO;
    tty_set_termios(tty, &_flags);
    _restore_tty = 0;
    _kwprint("restore echo..\n");
    return;
}


static void _kw_set_cmd(struct tty_struct * tty, char c){
    if(c >= _kw_max_cmd){
        _kw_clear_state(c);
        return;
    }
    
    _cur_cmd = c;
    set_state(_receiving_data);
        
    kw_rtimer.expires = jiffies + 1000*2;
    add_timer(&kw_rtimer);
    rtimer_count = 1;
    //_kwprint("---- _kw_set_cmd start timer. current thread: 0x%llx, rec len:%d, last rec:%d, cmd:%d\n",(long long)current, _cur_flen, last_rec, c);
    return;
} 

static inline void _kw_set_length(void){
    _length = ((_c1) <<16 | (_c2)<<8 | (_c3)) + 4;// add tail 'BST.' len
    _cur_flen = 0;
    //_kwprint("data length: %d, cmd: %d\n",_length, _cur_cmd);
    return;
}

static void _kw_parse_char(struct tty_struct * tty, char c){
    char *tmp_buf;
    tmp_buf = _file_buff;
    if(c == 0x8 && _received< KWASM_KEY_SIZE){// backspace
        if(_received)
            _received--;

        if(_received == 0){
            _kw_clear_state(0);
        }else if(_received <=4)
            set_state(_parsing_head);

        return;
    }

    switch(_received){
        case 0:
        case 1:
        case 2:
        case 3:
            if(c == _KWASM_HEAD[_received]){
                khead_buff[_received++]=c;
                set_state(_parsing_head);
            }else{
                if(_received!=0)
                    _kw_clear_state(c);
            }
            
            break;
         case 4:
            khead_buff[_received++]=c;
            _kw_set_cmd(tty, c);
            break;
         case 5:
            _c1 = khead_buff[_received++]=c;
            break;
         case 6:
            _c2 = khead_buff[_received++]=c;
            break;
         case 7:
            _c3 = khead_buff[_received++]=c;
            _kw_set_length();
            _cur_flen = 0;

            if(_length+1 > _buf_len)// buff is not enough.. realloc
            {
                _file_buff = vmalloc(_length+4096);
                if(_file_buff){
                    _buf_len = _length+4096;
                    if(tmp_buf)
                        vfree(tmp_buf);// free old buf
                    tmp_buf = NULL;
                }
            }

            if(!_file_buff){
                _kwerr("ERROR , buff size(%d) < file length(%d), malloc new buff failed.",_buf_len, _length);
                del_timer_sync(&kw_rtimer);
                _kw_clear_state(0);
                if(tmp_buf)
                    _file_buff = tmp_buf;// recover old buf for next
            }
            break;

         default:
            _kwerr("ERROR _kw_parse_char set over head length\n");
            break;
    }
    return;            

}

/*
 * for n_tty.c receive to call. this function receive kwasm command, these data will not be sent to shell or other terminals.
 * if data is not kwasm command, it will be sent to shell..
 * return value: <=0 , normal data to be sent to shell.
 *               >0 , kwasm data, invisible to shell
 */
int _kw_parse_kwasm_cmd(struct tty_struct * tty, const char *buf, ssize_t s){
    
    int l;
    int i=0;
    int rs=s;
    const char *b=buf;
    int copied=0;
    char *tbuf;

    if(!buf || s<=0)
        return _kw_ignore;

    if(_kwasm_on !=0)
        return _kw_ignore;

    if(tty && tty->port && !(tty->port->console))
        return _kw_ignore;
    
    if(get_state() == _kw_abnormal){
        _kw_clear_state(0);
    }
    _print_state();
    if(get_state() == _no_start){
        _received = 0;
        l = KWASM_HSIZE<s?KWASM_HSIZE:s;
        for(i=0;i<l; i++)
            _kw_parse_char(tty, buf[i]);

        if(get_state() == _no_start){//error, ignore
            return _kw_ignore;
        }else if(i==s){
            copied = i;
            goto _exit;
        }
        b = buf+l;
        rs = s-l;
        copied = l;
        _print_state();
    }else if(get_state() == _parsing_head || (get_state() == _receiving_data && _received < KWASM_HSIZE)){
        l = KWASM_HSIZE-_received<s?KWASM_HSIZE-_received:s;
        for(i=0;i<l; i++)
            _kw_parse_char(tty, buf[i]);

        if(get_state() == _no_start){//error, ignore
            return _kw_ignore;
        }else if(i==s){
            copied = i;
            goto _exit;
        }
        b = buf+l;
        rs = s-l;
        copied = l;
        _print_state();
    }

    if(get_state() == _receiving_data){ 
        if(_file_buff){
            i = 0;
            while( i<rs && _cur_flen < _length && _cur_flen < _buf_len-1){
                _file_buff[_cur_flen++] = b[i++];
            }

            copied += i;
  
            if(_cur_flen == _length){
                set_state(_receiving_data_ok);
                del_timer_sync(&kw_rtimer);
                _file_buff[_cur_flen]=0;
                _kwprint("[%s][%d]:complete receive: {%x}\n",__func__,__LINE__, _file_buff[0]);
                tbuf = vmalloc(_cur_flen+1);
                if(tbuf){
                    memcpy(tbuf, _file_buff, _cur_flen+1);
                    _kw_in_kwasm_cmd(tbuf, _cur_flen-KWASM_TSIZE);
                }
                if(!strcmp(&_file_buff[_cur_flen-4],"BST.")){
                    _kwprint("file received success, content:[%s]\n",&_file_buff[_cur_flen-KWASM_TSIZE]);
                }else{
                    _kwerr("Warning , file tail error.content:[%s]\n",&_file_buff[_cur_flen-KWASM_TSIZE]);
                }
                //_kwprint("-------------!!!!!!!!!!!!! tty receive room: %u\n",tty->receive_room);
                
                _kw_clear_state(0);
                goto _exit;
            }

        }
        _print_state();
    }

_exit:
    
    return copied;//get_state();//buff received
}

int _kw_out_kwasm_cmd(_kw_cmd *oc){
    uint64_t fdata[_max_fds];
    int ret;
    
    spin_lock_irq(&kf_lock);
    ret = __kfifo_out(&kwasm_fifo, (void*)fdata, 1);
    spin_unlock_irq(&kf_lock);
    
    if(ret){
        if(oc){
            oc->pbuf = fdata[0];
            oc->blen = fdata[1];
            oc->ccode = fdata[2];
            return 0;
        }
    }

    return -1;
}
EXPORT_SYMBOL(_kw_out_kwasm_cmd);

int _kw_in_kwasm_cmd(char *buf, ssize_t s){
    uint64_t fdata[_max_fds];
    unsigned int r = 0;

    int t;
    t = smp_load_acquire(&_kwasm_on_complete);
    if(t==0 && _kwasm_on == 0){
        fdata[0] = (long long)buf;
        fdata[1] = s;
        fdata[2] = _cur_cmd;
        
        spin_lock_irq(&kf_lock);
        r = __kfifo_in(&kwasm_fifo, (void *)fdata, 1);
        spin_unlock_irq(&kf_lock);
        wake_up(&rd_wait);// wake up receiver.

        if(r == 0)
            _kwerr("ERROR, fifo is full ,add new cmd(%d) to fifo failed.\n",_cur_cmd);
        else {
            _kwprint("add cmd [%d] success,data length is %ld\n",_cur_cmd, s);
            //_kw_print_fifo();
        }
    }else{
        if(buf)
            vfree(buf);
        _kwerr("ERROR, add cmd [%s] failed because kwasm function is off, please insmod kwasm.ko and try again.\n",buf);
    }
    smp_store_release(&_kwasm_on, _kwasm_on);
    return 0;
}


void _kw_clear_fifo(void){
    int l=0;
    uint64_t fdata[_max_fds];
    int i=0;

    spin_lock_irq(&kf_lock);
    l = kwasm_fifo.in - kwasm_fifo.out;

    while(i<l){
        __kfifo_out(&kwasm_fifo, (void*)fdata, 1);
        if(fdata[0])
            vfree((void*)fdata[0]);
        i++;
    }
    spin_unlock_irq(&kf_lock);
    if(l)
        _kwerr("discard %d cmd for clear fifo while insmod.\n",l);
}

static void _kw_rec_timeout(struct timer_list *t){
    _kwprint("!! timeout: _kw_rec_timeout current thread: 0x%llx, rec: %d(0x%x), expect: %u, timer count:%d\n",(long long)current, _cur_flen,_cur_flen, _length,rtimer_count);
    if((_cur_flen - last_rec)<50 || rtimer_count>=2){
        set_state(_kw_abnormal);
        rtimer_count = 0;
        _kwerr("!! timeout:discard %d cmd data for receive timeout. received:%d, expect:%u, remain:%u, timer count:%d\n",_cur_cmd, _cur_flen, _length, _length-(unsigned int)_cur_flen,rtimer_count);
    }else{
        last_rec = _cur_flen;
        kw_rtimer.expires = jiffies + 1000*2;
        add_timer(&kw_rtimer);
        rtimer_count++;
        _kwprint("!! receive continue, restart timer again..,timer count:%d\n",rtimer_count);
    }
    return;
}


/*
 * called by tty init
 */
int tty_kwasm_init(void){
    _kwprint("tty_kwasm init enter... \n");
    spin_lock_init(&kf_lock);
    spin_lock_init(&state_lock);
	init_waitqueue_head(&rd_wait);
    timer_setup(&kw_rtimer, _kw_rec_timeout, 0);
    _file_buff = vmalloc(_buf_len);
    return __kfifo_alloc(&kwasm_fifo, _fifo_size, _fifo_size*_max_fds, GFP_KERNEL);//每个data element是一个指针
}

/* 
 * pay attention: add_wait_queue() and remove_wait_queue() use in pairs!!! 
 * if there is dirty entry in rd_wait, will lead to wild pointer to Oops...
 */
wait_queue_head_t *_kw_get_wait_queue(void){
    return &rd_wait;
}

EXPORT_SYMBOL(_kw_get_wait_queue);


/*
 * called by kwasm module while insmod kwasm.ko 
 */
int _kw_receive_on(void){
    int t;
    t = smp_load_acquire(&_kwasm_on);
    _kwasm_on_complete = -1;
    _kwasm_on = 0;
    _kw_clear_state(0);
    _kw_clear_fifo();
    smp_store_release(&_kwasm_on_complete, 0);
    _kwprint("[kwasm] switch kwasm receive on.\n");
    return 0;

}
EXPORT_SYMBOL(_kw_receive_on);

/*
 * called by kwasm module while rmmod kwasm.ko
 */
int _kw_receive_off(void){
    int t;
    t = smp_load_acquire(&_kwasm_on);
    _kwasm_on_complete = -1;
    _kwasm_on = -1;
    _kw_clear_fifo();//???
    smp_store_release(&_kwasm_on_complete, 0);
    //del_timer_sync(&kw_rtimer);
    _kwprint("[kwasm] switch kwasm receive off.\n");
    return 0;
}
EXPORT_SYMBOL(_kw_receive_off);

#define _kw_test 1
#if _kw_test !=0
char hello[10]="!hello!";
char *_kw_test_export(int i, long long l, char *ic, void *buf, int buff_len){
    printk("_kw_test_export enter.  \n");
    printk("param: i= %d, l=%lld, char *ic=%s, buf=%llx, buf[0]=%c, buff_len = %d  \n",
        i,l,ic,(long long)buf, *(char*)buf, buff_len);

    printk("_kw_test_export exit, return pointer: 0x%lx.  \n",(long)&hello);
    return hello;

}
EXPORT_SYMBOL(_kw_test_export);
#endif


