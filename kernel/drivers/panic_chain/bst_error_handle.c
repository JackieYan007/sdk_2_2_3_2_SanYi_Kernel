#include "bst_error_handle.h"

static const char *error_types[NR_ERROR] = {
#define Str(x,s)	[x] = s
    Str(EVENT_START,"It just a start,nothing will happen"),
    Str(EVENT_TEST,"Test whether the notification chain is clear"),
    Str(EVENT_DIE,"kernel has been a \"die\""),
    Str(EVENT_WARN,"kernel has been a \"warn\""),
    Str(EVENT_PANIC,"kernel has been a \"panic\""),
    Str(EVENT_REBOOT,"kernel has been a \"reboot\""),
    Str(EVENT_BUG,"kernel has been a \"bug\""),
    Str(EVENT_HUNG_TASK,"kernel has been a \"huang task\""),
    Str(EVENT_SOFT_LOCKUP,"kernel has been a \"soft lockup\""),
    Str(EVENT_USER_FAULTS,"kernel has been a \"user faults\""),
    Str(EVENT_INFO_UNAVA,"\"info unavailable\""),
};

/*
 * AArch64 PCS assigns the frame pointer to x29.
 *
 * A simple function prologue looks like this:
 * 	sub	sp, sp, #0x10
 *   	stp	x29, x30, [sp]
 *	mov	x29, sp
 *
 * A simple function epilogue looks like this:
 *	mov	sp, x29
 *	ldp	x29, x30, [sp]
 *	add	sp, sp, #0x10
 */
extern int unwind_frame(struct task_struct *tsk, struct stackframe *frame);
static void bst_dump_backtrace_entry(unsigned long where)
{
	bst_print(" %pS\n", (void *)where);
}

static void bst_dump_backtrace(struct pt_regs *regs,struct task_struct *tsk)
{
	struct stackframe frame;
	int skip = 0;

	bst_print("%s(regs = %p tsk = %p)\n", __func__, regs, tsk);

	if (regs) {
		if (user_mode(regs))
			return;
		skip = 1;
	}

	if (!tsk)
		tsk = current;

	if (!try_get_task_stack(tsk))
		return;

	if (tsk == current) {
		frame.fp = (unsigned long)__builtin_frame_address(0);
		frame.pc = (unsigned long)bst_dump_backtrace;
	} else {
		/*
		 * task blocked in __switch_to
		 */
		frame.fp = thread_saved_fp(tsk);
		frame.pc = thread_saved_pc(tsk);
	}
#ifdef CONFIG_FUNCTION_GRAPH_TRACER
	frame.graph = tsk->curr_ret_stack;
#endif

	bst_print("Call trace:\n");
	do {
		/* skip until specified stack frame */
		if (!skip) {
			bst_dump_backtrace_entry(frame.pc);
		} else if (frame.fp == regs->regs[29]) {
			skip = 0;
			/*
			 * Mostly, this is the case where this function is
			 * called in panic/abort. As exception handler's
			 * stack frame does not contain the corresponding pc
			 * at which an exception has taken place, use regs->pc
			 * instead.
			 */
			bst_dump_backtrace_entry(regs->pc);
		}
	} while (!unwind_frame(tsk, &frame));

	put_task_stack(tsk);
}

/*
 * bst__show_regs - Write regs to owm address
 * @regs:	arm regs addr
 */
static void bst__show_regs(struct pt_regs *regs)
{
	int i, top_reg;
	unsigned long long  lr, sp;

    if(!regs){
        return;
	}
	
	if (compat_user_mode(regs)) {
		lr = regs->compat_lr;
		sp = regs->compat_sp;
		top_reg = 12;
	} else {
		lr = regs->regs[30];
		sp = regs->sp;
		top_reg = 29;
	}

	if (!user_mode(regs)) {
		bst_print("pc : %pS\n", (void *)regs->pc);
		bst_print("lr : %pS\n", (void *)lr);
	} else {
		bst_print("pc : %016llx\n", regs->pc);
		bst_print("lr : %016llx\n", lr);
	}

	bst_print("sp : %016llx\n", sp);

	i = top_reg;

	while (i >= 0) {
		bst_print("x%-2d: %016llx ", i, regs->regs[i]);
		i--;

		if (i % 2 == 0) {
			bst_print("x%-2d: %016llx ", i, regs->regs[i]);
			i--;
		}

		bst_print("\n");
	}
}

static void bst_data_abort_decode(unsigned int esr)
{
	bst_print("Data abort info:\n");

	if (esr & ESR_ELx_ISV) {
		bst_print("  Access size = %u byte(s)\n",
			 1U << ((esr & ESR_ELx_SAS) >> ESR_ELx_SAS_SHIFT));
		bst_print("  SSE = %lu, SRT = %lu\n",
			 (esr & ESR_ELx_SSE) >> ESR_ELx_SSE_SHIFT,
			 (esr & ESR_ELx_SRT_MASK) >> ESR_ELx_SRT_SHIFT);
		bst_print("  SF = %lu, AR = %lu\n",
			 (esr & ESR_ELx_SF) >> ESR_ELx_SF_SHIFT,
			 (esr & ESR_ELx_AR) >> ESR_ELx_AR_SHIFT);
	} else {
		bst_print("  ISV = 0, ISS = 0x%08lx\n", esr & ESR_ELx_ISS_MASK);
	}

	bst_print("  CM = %lu, WnR = %lu\n",
		 (esr & ESR_ELx_CM) >> ESR_ELx_CM_SHIFT,
		 (esr & ESR_ELx_WNR) >> ESR_ELx_WNR_SHIFT);
}

static void bst_mem_abort_decode(unsigned int esr)
{
	bst_print("Mem abort info:\n");

	bst_print("  ESR = 0x%08x\n", esr);
	bst_print("  Exception class = %s, IL = %u bits\n",
		 esr_get_class_string(esr),
		 (esr & ESR_ELx_IL) ? 32 : 16);
	bst_print("  SET = %lu, FnV = %lu\n",
		 (esr & ESR_ELx_SET_MASK) >> ESR_ELx_SET_SHIFT,
		 (esr & ESR_ELx_FnV) >> ESR_ELx_FnV_SHIFT);
	bst_print("  EA = %lu, S1PTW = %lu\n",
		 (esr & ESR_ELx_EA) >> ESR_ELx_EA_SHIFT,
		 (esr & ESR_ELx_S1PTW) >> ESR_ELx_S1PTW_SHIFT);

	if (esr_is_data_abort(esr))
		bst_data_abort_decode(esr);
}

extern void bst_print_modules(char *p);
extern void bst_mem_init_print_info(char *p);
static void bst_common_handle(enum NOTIFIER_CHAIN_STA m)
{
	struct __kernel_timex txc;
	struct rtc_time tm;
	struct timespec64 ts;

	ktime_get_real_ts64(&ts);
	txc.time.tv_sec = ts.tv_sec;
	txc.time.tv_usec = ts.tv_nsec / 1000;
	rtc_time64_to_tm(txc.time.tv_sec,&tm);

    memset(pErrorData,0,5000);
	bst_print("=====================================================================\n");
	bst_print("%d-%02d-%02d %02d:%02d:%02d :%s\n",tm.tm_year+1900,tm.tm_mon+1, 
                                                    tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec,
                                                        error_types[m]);
	bst_mem_init_print_info(pErrorData);
	bst_print_modules(pErrorData);
}

extern int send_usrmsg(char *pbuf, uint16_t len);

void bst_test_handle(void *data)
{	
	if(!data)
        return;

	bst_common_handle(EVENT_TEST);
	bst_print("%s",(char *)data);
	
	send_usrmsg(pErrorData,strlen(pErrorData));
}

void bst_warn_handle(void *data)
{
    struct pt_regs *regs = (struct pt_regs *)data;

	bst_common_handle(EVENT_WARN);

	bst_print("WARNING: CPU: %d PID: %d\n", raw_smp_processor_id(), current->pid);
	bst_dump_backtrace(NULL,NULL);
    bst__show_regs(regs);

	send_usrmsg(pErrorData,strlen(pErrorData));
}

void bst_panic_handle(void *data)
{
	bst_common_handle(EVENT_PANIC);
    bst_dump_backtrace(NULL,NULL);
	bst_print("Kernel panic - not syncing: %s\n", data == NULL ? "no msg" : (char *)data);

	send_usrmsg(pErrorData,strlen(pErrorData));
}

void bst_user_faults_handle(void *data)
{
	user_fault_s *user_fault = (user_fault_s *)data;
	
	bst_common_handle(EVENT_USER_FAULTS);
	bst_print("%s[%d]: unhandled exception: ", user_fault->comm, user_fault->pid);

	if (user_fault->esr)
		bst_print("ESR 0x%08x, ",user_fault->esr);

	bst_print("%s", user_fault->str);
	bst_print("\n");
	bst__show_regs(&user_fault->regs);
	bst_dump_backtrace(&user_fault->regs,NULL);

	send_usrmsg(pErrorData,strlen(pErrorData));
}

struct die_s{
	const char *str;
	struct pt_regs *regs;
	unsigned int err;
};

void bst_die_handle(void *data)
{
	struct die_s * bst_die_s = (struct die_s *)data;

	bst_common_handle(EVENT_USER_FAULTS);
	bst_print("Internal error: %s \r\n", bst_die_s->str);

    bst__show_regs(bst_die_s->regs);
    bst_dump_backtrace(bst_die_s->regs,NULL);
	bst_mem_abort_decode(bst_die_s->err);

	send_usrmsg(pErrorData,strlen(pErrorData));
}