#ifndef __REGS_HH__
#define __REGS_HH__

/**
 * 	Event base addr is ipc_mbox:event_base
 */
// event mst_id offset
#define EVENT_MST_ID_A_CPU0				0x0000
#define EVENT_MST_ID_A_CPU1				0x0100
#define EVENT_MST_ID_A_CPU2				0x0200
#define EVENT_MST_ID_A_CPU3				0x0300
#define EVENT_MST_ID_A_CPU4				0x0400
#define EVENT_MST_ID_A_CPU5				0x0500
#define EVENT_MST_ID_A_CPU6				0x0600
#define EVENT_MST_ID_A_CPU7				0x0700
#define EVENT_MST_ID_R_CPU0				0x1800
#define EVENT_MST_ID_R_CPU0				0x0900
#define EVENT_MST_ID_DSP0				0x0a00
#define EVENT_MST_ID_DSP1				0x0b00
#define EVENT_MST_ID_DSP2				0x0c00
#define EVENT_MST_ID_DSP3				0x0d00
#define EVENT_MST_ID_ISP_RV				0x0e00
#define EVENT_MST_ID_VSP_RV				0x0f00
#define EVENT_MST_ID_NET_RV				0x1000
#define EVENT_MST_ID_SEC_R5				0x1100
#define EVENT_MST_A_CPU_STEP	(EVENT_MST_ID_A_CPU1 - EVENT_MST_ID_A_CPU0)

//source Registers offset for Event Notification
#define EVENT_OFFSET_A_CPU0_IPC_INT_SRC	0x0
#define EVENT_OFFSET_A_CPU1_IPC_INT_SRC	0x4
#define EVENT_OFFSET_A_CPU2_IPC_INT_SRC	0x8
#define EVENT_OFFSET_A_CPU3_IPC_INT_SRC	0xc
#define EVENT_OFFSET_A_CPU4_IPC_INT_SRC	0x10
#define EVENT_OFFSET_A_CPU5_IPC_INT_SRC	0x14
#define EVENT_OFFSET_A_CPU6_IPC_INT_SRC	0x18
#define EVENT_OFFSET_A_CPU7_IPC_INT_SRC	0x1c
#define EVENT_OFFSET_RV_NET_IPC_INT_SRC	0x40
#define EVENT_SRC_ACPU_STEP	\
	(EVENT_OFFSET_A_CPU1_IPC_INT_SRC-EVENT_OFFSET_A_CPU0_IPC_INT_SRC)

//enable Registers offset for Event Notification
#define EVENT_OFFSET_A_CPU0_IPC_INT_EN	0x80
#define EVENT_OFFSET_A_CPU1_IPC_INT_EN	0x84
#define EVENT_OFFSET_A_CPU2_IPC_INT_EN	0x88
#define EVENT_OFFSET_A_CPU3_IPC_INT_EN	0x8c
#define EVENT_OFFSET_A_CPU4_IPC_INT_EN	0x90
#define EVENT_OFFSET_A_CPU5_IPC_INT_EN	0x94
#define EVENT_OFFSET_A_CPU6_IPC_INT_EN	0x98
#define EVENT_OFFSET_A_CPU7_IPC_INT_EN	0x9c
#define EVENT_OFFSET_RV_NET_IPC_INT_EN	0xc0

#define EVENT_ENABLE_ACPU_STEP	\
	(EVENT_OFFSET_A_CPU1_IPC_INT_EN-EVENT_OFFSET_A_CPU0_IPC_INT_EN)

// source bit assignment
#define EVENT_BIT_APU0					0
#define EVENT_BIT_APU1					1
#define EVENT_BIT_APU2					2
#define EVENT_BIT_APU3					3
#define EVENT_BIT_APU4					4
#define EVENT_BIT_APU5					5
#define EVENT_BIT_APU6					6
#define EVENT_BIT_APU7					7
#define EVENT_BIT_RPU0					8
#define EVENT_BIT_RPU1					9
#define EVENT_BIT_R_SEC					17
#define EVENT_BIT_RISC_NET				16
#define EVENT_BIT_RISC_VSP				15
#define EVENT_BIT_RISC_ISP				14
#define EVENT_BIT_DSP_3					13
#define EVENT_BIT_DSP_2					12
#define EVENT_BIT_DSP_1					11
#define EVENT_BIT_DSP_0					10

/**
 * Semaphore base addr is ipc_mbox:sem_base
 */
// sem mst_id group offset
#define SEM_MST_ID_A_CPU				0x0200
#define SEM_MST_ID_GPU					0x0400
#define SEM_MST_ID_R_CPU				0x0600
#define SEM_MST_ID_SEC_RV				0x0800
#define SEM_MST_ID_DSP					0x0a00
#define SEM_MST_ID_ISP_RV				0x0c00
#define SEM_MST_ID_VSP_RV				0x0e00
#define SEM_MST_ID_NET_RV				0x1000

//banks offset
#define SEM_BANK0_OFFSET_BASE			0x000
#define SEM_BANK1_OFFSET_BASE			0x080
#define SEM_BANK2_OFFSET_BASE			0x100
#define SEM_BANK3_OFFSET_BASE			0x180
#define SEM_BANK_OFFSET_STEP (SEM_BANK1_OFFSET_BASE-SEM_BANK0_OFFSET_BASE)

// Semaphore offset in bank
#define SEM0_OFFSET_IN_BANK				0x0
#define SEM1_OFFSET_IN_BANK				0x4
#define SEM_PENDING_IN_BANK				0x40

#define SEM_IN_BANK_STEP (SEM1_OFFSET_IN_BANK-SEM0_OFFSET_IN_BANK)

//irq register Address offset
#define CPU_EN_REG_ADDR(read_reg_vaddr,cpu_id) \
    read_reg_vaddr + EVENT_MST_ID_A_CPU0 + (cpu_id*EVENT_MST_A_CPU_STEP) + EVENT_OFFSET_A_CPU0_IPC_INT_EN + (cpu_id *EVENT_ENABLE_ACPU_STEP)
//read src core and clear interrupt register offset address
#define READ_SRC_AND_CLEAR_IRQ_ADDR(read_reg_vaddr,src)\
    read_reg_vaddr + EVENT_MST_ID_A_CPU0 + (src*EVENT_MST_A_CPU_STEP) + EVENT_OFFSET_A_CPU0_IPC_INT_SRC + (src *EVENT_SRC_ACPU_STEP)
//check interrupt register offset address
#define IRQ_CHECK_ADDR(read_reg_vaddr,dst)\
    read_reg_vaddr + EVENT_MST_ID_A_CPU0 + (dst*EVENT_MST_A_CPU_STEP) + EVENT_OFFSET_A_CPU0_IPC_INT_SRC + (EVENT_SRC_ACPU_STEP *dst)
//write interrupt register offset address
#define IRQ_WRITE_ADDR(read_reg_vaddr,src,dest)\
    read_reg_vaddr + EVENT_MST_ID_A_CPU0 + ((src) *EVENT_MST_A_CPU_STEP) + EVENT_OFFSET_A_CPU0_IPC_INT_SRC + (EVENT_SRC_ACPU_STEP *(dest))
#endif

