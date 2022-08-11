/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 * Debug helper to dump the current kernel pagetables of the system
 * so that we can see what the various memory ranges are set to.
 *
 * Derived from x86 and arm implementation:
 * (C) Copyright 2008 Intel Corporation
 *
 * Author: Arjan van de Ven <arjan@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/ptdump.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/seq_file.h>

#include <asm/fixmap.h>
#include <asm/kasan.h>
#include <asm/memory.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>

#ifndef CONFIG_ARM64
#error "This module is only for the ARM64 architecture."
#endif

#define NONE_SPACE 0
#define KERNEL_SPACE 1
#define USER_SPACE 2
#define MAX_LENGTH	30

struct addr_marker {
	unsigned long start_address;
	char *name;
};

struct ptdump_info {
	struct mm_struct	*mm;
	struct addr_marker	*markers;
	unsigned long	base_addr;
	unsigned long check_addr;
	unsigned long check_mmap;
	char *name;
	int has_user_space;
	int space;
	unsigned long pid;
};

static struct dentry *pid_page_tables_file;
static struct mm_struct mm;
static pgd_t *kernel_pgd;

enum address_markers_idx {
	PAGE_OFFSET_NR = 0,
	PAGE_END_NR,
#if defined(CONFIG_KASAN_GENERIC) || defined(CONFIG_KASAN_SW_TAGS)
	KASAN_START_NR,
#endif
};

/* Kernel space */
static struct addr_marker address_markers[] = {
	{ PAGE_OFFSET,			"Linear Mapping start" },
	{ 0 /* PAGE_END */,		"Linear Mapping end" },
#if defined(CONFIG_KASAN_GENERIC) || defined(CONFIG_KASAN_SW_TAGS)
	{ 0 /* KASAN_SHADOW_START */,	"Kasan shadow start" },
	{ KASAN_SHADOW_END,		"Kasan shadow end" },
#endif
	{ BPF_JIT_REGION_START,		"BPF start" },
	{ BPF_JIT_REGION_END,		"BPF end" },
	{ MODULES_VADDR,		"Modules start" },
	{ MODULES_END,			"Modules end" },
	{ VMALLOC_START,		"vmalloc() area" },
	{ VMALLOC_END,			"vmalloc() end" },
	{ FIXADDR_START,		"Fixmap start" },
	{ FIXADDR_TOP,			"Fixmap end" },
	{ PCI_IO_START,			"PCI I/O start" },
	{ PCI_IO_END,			"PCI I/O end" },
#ifdef CONFIG_SPARSEMEM_VMEMMAP
	{ VMEMMAP_START,		"vmemmap start" },
	{ VMEMMAP_START + VMEMMAP_SIZE,	"vmemmap end" },
#endif
	{ -1,				NULL },
};

#define START_CODE	0
#define END_CODE	1
#define START_DATA	2
#define END_DATA	3
#define START_BRK	4
#define END_BRK		5
#define MMAP_END	6
#define MMAP_BASE	7
#define MISC_START	8
#define MISC_END	9

/* User space */
static struct addr_marker address_markers_user[] = {
	{ 0, 			"Start code" },
	{ 0, 			"End code" },
	{ 0, 			"Start data" },
	{ 0, 			"End data" },
	{ 0, 			"Start brk (heap)" },
	{ 0, 			"End brk (heap)" },
	{ 0, 			"Mmap end" },
	{ 0, 			"Mmap base" },
	{ 0, 			"Misc start" },
	{ 0, 			"Misc end" },
	{ -1,				NULL },
};

#define pt_dump_seq_printf(m, fmt, args...)	\
({						\
	if (m)					\
		seq_printf(m, fmt, ##args);	\
})

#define pt_dump_seq_puts(m, fmt)	\
({					\
	if (m)				\
		seq_printf(m, fmt);	\
})

/*
 * The page dumper groups page table entries of the same type into a single
 * description. It uses pg_state to track the range information while
 * iterating over the pte entries. When the continuity is broken it then
 * dumps out a description of the range.
 */
struct pg_state {
	struct ptdump_state ptdump;
	struct seq_file *seq;
	const struct addr_marker *marker;
	unsigned long start_address;
	unsigned level;
	u64 current_prot;
	bool check_wx;
	unsigned long wx_pages;
	unsigned long uxn_pages;
};

struct prot_bits {
	u64		mask;
	u64		val;
	const char	*set;
	const char	*clear;
};

static const struct prot_bits pte_bits[] = {
	{
		.mask	= PTE_VALID,
		.val	= PTE_VALID,
		.set	= " ",
		.clear	= "F",
	}, {
		.mask	= PTE_USER,
		.val	= PTE_USER,
		.set	= "USR",
		.clear	= "   ",
	}, {
		.mask	= PTE_RDONLY,
		.val	= PTE_RDONLY,
		.set	= "ro",
		.clear	= "RW",
	}, {
		.mask	= PTE_PXN,
		.val	= PTE_PXN,
		.set	= "NX",
		.clear	= "x ",
	}, {
		.mask	= PTE_SHARED,
		.val	= PTE_SHARED,
		.set	= "SHD",
		.clear	= "   ",
	}, {
		.mask	= PTE_AF,
		.val	= PTE_AF,
		.set	= "AF",
		.clear	= "  ",
	}, {
		.mask	= PTE_NG,
		.val	= PTE_NG,
		.set	= "NG",
		.clear	= "  ",
	}, {
		.mask	= PTE_CONT,
		.val	= PTE_CONT,
		.set	= "CON",
		.clear	= "   ",
	}, {
		.mask	= PTE_TABLE_BIT,
		.val	= PTE_TABLE_BIT,
		.set	= "   ",
		.clear	= "BLK",
	}, {
		.mask	= PTE_UXN,
		.val	= PTE_UXN,
		.set	= "UXN",
		.clear	= "   ",
	}, {
		.mask	= PTE_GP,
		.val	= PTE_GP,
		.set	= "GP",
		.clear	= "  ",
	}, {
		.mask	= PTE_ATTRINDX_MASK,
		.val	= PTE_ATTRINDX(MT_DEVICE_nGnRnE),
		.set	= "DEVICE/nGnRnE",
	}, {
		.mask	= PTE_ATTRINDX_MASK,
		.val	= PTE_ATTRINDX(MT_DEVICE_nGnRE),
		.set	= "DEVICE/nGnRE",
	}, {
		.mask	= PTE_ATTRINDX_MASK,
		.val	= PTE_ATTRINDX(MT_DEVICE_GRE),
		.set	= "DEVICE/GRE",
	}, {
		.mask	= PTE_ATTRINDX_MASK,
		.val	= PTE_ATTRINDX(MT_NORMAL_NC),
		.set	= "MEM/NORMAL-NC",
	}, {
		.mask	= PTE_ATTRINDX_MASK,
		.val	= PTE_ATTRINDX(MT_NORMAL),
		.set	= "MEM/NORMAL",
	}, {
		.mask	= PTE_ATTRINDX_MASK,
		.val	= PTE_ATTRINDX(MT_NORMAL_TAGGED),
		.set	= "MEM/NORMAL-TAGGED",
	}
};

struct pg_level {
	const struct prot_bits *bits;
	const char *name;
	size_t num;
	u64 mask;
};

static struct pg_level pg_level[] = {
	{ /* pgd */
		.name	= "PGD",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	}, { /* p4d */
		.name	= "P4D",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	}, { /* pud */
		.name	= (CONFIG_PGTABLE_LEVELS > 3) ? "PUD" : "PGD",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	}, { /* pmd */
		.name	= (CONFIG_PGTABLE_LEVELS > 2) ? "PMD" : "PGD",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	}, { /* pte */
		.name	= "PTE",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	},
};

static void dump_prot(struct pg_state *st, const struct prot_bits *bits,
			size_t num)
{
	unsigned i;

	for (i = 0; i < num; i++, bits++) {
		const char *s;

		if ((st->current_prot & bits->mask) == bits->val)
			s = bits->set;
		else
			s = bits->clear;

		if (s)
			pt_dump_seq_printf(st->seq, " %s", s);
	}
}

static void note_prot_uxn(struct pg_state *st, unsigned long addr)
{
	if (!st->check_wx)
		return;

	if ((st->current_prot & PTE_UXN) == PTE_UXN)
		return;

	WARN_ONCE(1, "arm64/mm: Found non-UXN mapping at address %p/%pS\n",
		  (void *)st->start_address, (void *)st->start_address);

	st->uxn_pages += (addr - st->start_address) / PAGE_SIZE;
}

static void note_prot_wx(struct pg_state *st, unsigned long addr)
{
	if (!st->check_wx)
		return;
	if ((st->current_prot & PTE_RDONLY) == PTE_RDONLY)
		return;
	if ((st->current_prot & PTE_PXN) == PTE_PXN)
		return;

	WARN_ONCE(1, "arm64/mm: Found insecure W+X mapping at address %p/%pS\n",
		  (void *)st->start_address, (void *)st->start_address);

	st->wx_pages += (addr - st->start_address) / PAGE_SIZE;
}

static void note_page(struct ptdump_state *pt_st, unsigned long addr, int level,
		      u64 val)
{
	struct pg_state *st = container_of(pt_st, struct pg_state, ptdump);
	static const char units[] = "KMGTPE";
	u64 prot = 0;

	if (level >= 0)
		prot = val & pg_level[level].mask;

	if (st->level == -1) {
		st->level = level;
		st->current_prot = prot;
		st->start_address = addr;
		pt_dump_seq_printf(st->seq, "---[ %s ]---\n", st->marker->name);
	} else if (prot != st->current_prot || level != st->level ||
		   addr >= st->marker[1].start_address) {
		const char *unit = units;
		unsigned long delta;

		if (st->current_prot) {
			note_prot_uxn(st, addr);
			note_prot_wx(st, addr);
		}

		pt_dump_seq_printf(st->seq, "0x%016lx-0x%016lx   ",
				   st->start_address, addr);

		delta = (addr - st->start_address) >> 10;
		while (!(delta & 1023) && unit[1]) {
			delta >>= 10;
			unit++;
		}
		pt_dump_seq_printf(st->seq, "%9lu%c %s", delta, *unit,
				   pg_level[st->level].name);
		if (st->current_prot && pg_level[st->level].bits)
			dump_prot(st, pg_level[st->level].bits,
				  pg_level[st->level].num);
		pt_dump_seq_puts(st->seq, "\n");

		if (addr >= st->marker[1].start_address) {
			st->marker++;
			pt_dump_seq_printf(st->seq, "---[ %s ]---\n", st->marker->name);
		}

		st->start_address = addr;
		st->current_prot = prot;
		st->level = level;
	}

	if (addr >= st->marker[1].start_address) {
		st->marker++;
		pt_dump_seq_printf(st->seq, "---[ %s ]---\n", st->marker->name);
	}
}

static unsigned long find_mmap_logic_end(struct mm_struct *mm)
{
	struct vm_area_struct *vma = mm->mmap;
	unsigned long mmap_end = 0;

	while (vma) {
		if (mmap_end < mm->brk) {
			mmap_end = vma->vm_start;
			vma = vma->vm_next;
			continue;
		}

		mmap_end = min_t(unsigned long, mmap_end, vma->vm_start);
		vma = vma->vm_next;
	}

	vma = mm->mmap->vm_prev;
	while (vma) {
		if (mmap_end < mm->brk) {
			mmap_end = vma->vm_start;
			vma = vma->vm_prev;
			continue;
		}
		mmap_end = min_t(unsigned long, mmap_end, vma->vm_start);
		vma = vma->vm_prev;
	}

	return mmap_end;
}

static unsigned long virt_addr_to_pfn(struct mm_struct *mm, unsigned long addr, int *type)
{
	pgd_t *pgdp;
	p4d_t *p4dp;
	pud_t *pudp, pud;
	pmd_t *pmdp, pmd;
	pte_t *ptep, pte;

	pgdp = pgd_offset(mm, addr);
	if (pgd_none(READ_ONCE(*pgdp)))
		return 0;

	p4dp = p4d_offset(pgdp, addr);
	if (p4d_none(READ_ONCE(*p4dp)))
		return 0;

	pudp = pud_offset(p4dp, addr);
	pud = READ_ONCE(*pudp);
	if (pud_none(pud))
		return 0;

	if (pud_sect(pud)) {
		*type = pfn_valid(pud_pfn(pud));

		return pud_pfn(pud);
	}

	pmdp = pmd_offset(pudp, addr);
	pmd = READ_ONCE(*pmdp);
	if (pmd_none(pmd))
		return 0;

	if (pmd_sect(pmd)) {
		*type = pfn_valid(pmd_pfn(pmd));
		return pmd_pfn(pmd);
	}

	ptep = pte_offset_kernel(pmdp, addr);
	pte = READ_ONCE(*ptep);
	if (pte_none(pte))
		return 0;

	*type = pfn_valid(pte_pfn(pte));
	return pte_pfn(pte);
}

static struct resource *next_resource(struct resource *p)
{
	if (p->child)
		return p->child;
	while (!p->sibling && p->parent)
		p = p->parent;
	return p->sibling;
}

static struct resource *find_iomem_res(unsigned long addr)
{
	struct resource *p;

	// read lock ?

	for (p = iomem_resource.child; p; p = next_resource(p)) {
		if (p->start <= addr && addr <= p->end)
			break;
	}

	if (!p)
		return NULL;

	return p;
}

static struct ptdump_info ptdump_info = {
	.markers	= address_markers,
	.base_addr	= PAGE_OFFSET,
	.name		= NULL,
	.has_user_space	= 0,
	.space = KERNEL_SPACE,
	.check_addr = 0,
	.check_mmap = 0,
};

static int ptdump_walk_ts(void)
{
	unsigned long pid = ptdump_info.pid;
	struct pid *kpid;
	struct task_struct *ts;

	BUG_ON(ptdump_info.space == KERNEL_SPACE);

	kpid = find_get_pid(pid);
	ts = get_pid_task(kpid, PIDTYPE_PID);
	if (ts) {
		if (ts->mm) {
			mm.pgd = ts->mm->pgd;
			ptdump_info.markers = address_markers_user;
			ptdump_info.base_addr = 0;
			ptdump_info.has_user_space = 1;
		} else {
			mm.pgd = kernel_pgd;
			ptdump_info.mm = &mm;
			ptdump_info.markers = address_markers;
			ptdump_info.base_addr = PAGE_OFFSET;
			ptdump_info.has_user_space = 0;
		}
		ptdump_info.name = ts->comm;

		if (ts->mm) {
			address_markers_user[START_CODE].start_address = rounddown(ts->mm->start_code, PAGE_SIZE);
			address_markers_user[END_CODE].start_address = roundup(ts->mm->end_code, PAGE_SIZE);

			address_markers_user[START_DATA].start_address = rounddown(ts->mm->start_data, PAGE_SIZE);
			address_markers_user[END_DATA].start_address = roundup(ts->mm->end_data, PAGE_SIZE);

			address_markers_user[START_BRK].start_address = rounddown(ts->mm->start_brk, PAGE_SIZE);
			address_markers_user[END_BRK].start_address = roundup(ts->mm->brk, PAGE_SIZE);

			address_markers_user[MMAP_END].start_address = roundup(ts->mm->brk, PAGE_SIZE);
			address_markers_user[MMAP_BASE].start_address = roundup(ts->mm->mmap_base, PAGE_SIZE);

			address_markers_user[MISC_START].start_address = roundup(ts->mm->mmap_base, PAGE_SIZE);
			address_markers_user[MISC_END].start_address = roundup(ts->mm->highest_vm_end, PAGE_SIZE);

			mm.start_code = ts->mm->start_code;
			mm.end_code = ts->mm->end_code;

			mm.start_data = ts->mm->start_data;
			mm.end_data = ts->mm->end_data;

			mm.start_brk = ts->mm->start_brk;
			mm.brk = ts->mm->brk;

			mm.mmap_base = ts->mm->mmap_base;
			mm.mmap = ts->mm->mmap;
			mm.highest_vm_end = ts->mm->highest_vm_end;

			mm.start_stack = ts->mm->start_stack;

			mm.arg_start = ts->mm->arg_start;
			mm.arg_end = ts->mm->arg_end;

			mm.env_start = ts->mm->env_start;
			mm.env_end = ts->mm->env_end;
		}
	} else {
		return -ESRCH;
		
	}

	return 0;
}

struct vma_flags_bits {
	u64		mask;
	u64		val;
	const char	*set;
};

static const struct vma_flags_bits vma_flags_bits[] = {
	{
		.mask	= VM_READ,
		.val	= VM_READ,
		.set	= "READ",
	}, {
		.mask	= VM_WRITE,
		.val	= VM_WRITE,
		.set	= "WRITE",
	}, {
		.mask	= VM_EXEC,
		.val	= VM_EXEC,
		.set	= "EXEC",
	}, {
		.mask	= VM_SHARED,
		.val	= VM_SHARED,
		.set	= "SHARED",
	}, {
		.mask	= VM_MAYREAD,
		.val	= VM_MAYREAD,
		.set	= "MAYREAD",
	}, {
		.mask	= VM_MAYWRITE,
		.val	= VM_MAYWRITE,
		.set	= "MAYWRITE",
	}, {
		.mask	= VM_MAYEXEC,
		.val	= VM_MAYEXEC,
		.set	= "MAYEXEC",
	}, {
		.mask	= VM_MAYSHARE,
		.val	= VM_MAYSHARE,
		.set	= "MAYSHARE",
	}, {
		.mask	= VM_GROWSDOWN,
		.val	= VM_GROWSDOWN,
		.set	= "GROWSDOWN",
	}, {
		.mask	= VM_UFFD_MISSING,
		.val	= VM_UFFD_MISSING,
		.set	= "UFFD_MISSING",
	}, {
		.mask	= VM_PFNMAP,
		.val	= VM_PFNMAP,
		.set	= "PFNMAP",
	}, {
		.mask	= VM_DENYWRITE,
		.val	= VM_DENYWRITE,
		.set	= "DENYWRITE",
	}, {
		.mask	= VM_UFFD_WP,
		.val	= VM_UFFD_WP,
		.set	= "UFFD_WP",
	}, {
		.mask	= VM_LOCKED,
		.val	= VM_LOCKED,
		.set	= "LOCKED",
	}, {
		.mask	= VM_IO,
		.val	= VM_IO,
		.set	= "IO",
	}, {
		.mask	= VM_SEQ_READ,
		.val	= VM_SEQ_READ,
		.set	= "SEQ_READ",
	}, {
		.mask	= VM_RAND_READ,
		.val	= VM_RAND_READ,
		.set	= "RAND_READ",
	}, {
		.mask	= VM_DONTCOPY,
		.val	= VM_DONTCOPY,
		.set	= "DONTCOPY",
	}, {
		.mask	= VM_DONTEXPAND,
		.val	= VM_DONTEXPAND,
		.set	= "DONTEXPAND",
	}, {
		.mask	= VM_LOCKONFAULT,
		.val	= VM_LOCKONFAULT,
		.set	= "LOCKONFAULT",
	}, {
		.mask	= VM_ACCOUNT,
		.val	= VM_ACCOUNT,
		.set	= "ACCOUNT",
	}, {
		.mask	= VM_NORESERVE,
		.val	= VM_NORESERVE,
		.set	= "NORESERVE",
	}, {
		.mask	= VM_HUGETLB,
		.val	= VM_HUGETLB,
		.set	= "HUGETLB",
	}, {
		.mask	= VM_SYNC,
		.val	= VM_SYNC,
		.set	= "SYNC",
	}, {
		.mask	= VM_ARCH_1,
		.val	= VM_ARCH_1,
		.set	= "ARCH_1",
	}, {
		.mask	= VM_WIPEONFORK,
		.val	= VM_WIPEONFORK,
		.set	= "WIPEONFORK",
	},
#ifdef CONFIG_MEM_SOFT_DIRTY
	{
		.mask	= VM_SOFTDIRTY,
		.val	= VM_SOFTDIRTY,
		.set	= "SOFTDIRTY",
	},
#endif
	{
		.mask	= VM_MIXEDMAP,
		.val	= VM_MIXEDMAP,
		.set	= "MIXEDMAP",
	}, {
		.mask	= VM_HUGEPAGE,
		.val	= VM_HUGEPAGE,
		.set	= "HUGEPAGE",
	}, {
		.mask	= VM_NOHUGEPAGE,
		.val	= VM_NOHUGEPAGE,
		.set	= "NOHUGEPAGE",
	}, {
		.mask	= VM_MERGEABLE,
		.val	= VM_MERGEABLE,
		.set	= "MERGEABLE",
	}
};

static void dump_vma_flags(struct seq_file *m, u64 current_flags,
				const struct vma_flags_bits *bits, size_t num)
{
	unsigned i;

	for (i = 0; i < num; i++, bits++) {
		const char *s;

		if ((current_flags & bits->mask) == bits->val)
			s = bits->set;
		else
			s = NULL;

		if (s)
			pt_dump_seq_printf(m, " %s", s);
	}
}

static int ptdump_walk_vma_mmap(struct seq_file *m, struct vm_area_struct *mmap)
{
	struct vm_area_struct *vma = mmap;

	if (!m || !vma)
		return -EINVAL;

	while (vma) {
		pt_dump_seq_printf(m, "0x%016lx-0x%016lx   ", vma->vm_start, vma->vm_end);
		if (vma->vm_file)
			pt_dump_seq_printf(m, "Mmap file name [%s]\n", vma->vm_file->f_path.dentry->d_name.name);
		else
			pt_dump_seq_puts(m, "Anonymous mapping\n");

		pt_dump_seq_printf(m, "\tFlags raw: [0x%016lx] ",  vma->vm_flags);
		dump_vma_flags(m, vma->vm_flags, vma_flags_bits, ARRAY_SIZE(vma_flags_bits));
		pt_dump_seq_puts(m, "\n\n");

		vma = vma->vm_next;
	}

	vma = mmap->vm_prev;
	while (vma) {
		pt_dump_seq_printf(m, "0x%016lx-0x%016lx   ", vma->vm_start, vma->vm_end);
		if (vma->vm_file)
			pt_dump_seq_printf(m, "Mmap file name [%s]\n", vma->vm_file->f_path.dentry->d_name.name);
		else
			pt_dump_seq_puts(m, "Anonymous mapping\n");

		pt_dump_seq_printf(m, "\tFlags raw: [0x%016lx] ",  vma->vm_flags);
		dump_vma_flags(m, vma->vm_flags, vma_flags_bits, ARRAY_SIZE(vma_flags_bits));
		pt_dump_seq_puts(m, "\n\n");

		vma = vma->vm_prev;
	}

	return 0;
}

static int ptdump_walk_pgd_raw(struct seq_file *m, struct ptdump_info *info)
{
	unsigned long end = ~0UL;
	struct mm_struct *mm = info->mm;
	struct pg_state st;

	if (info->base_addr < TASK_SIZE_64)
		end = TASK_SIZE_64;

	st = (struct pg_state) {
		.seq = m,
		.marker = info->markers,
		.ptdump = {
			.note_page = note_page,
			.range = (struct ptdump_range[]){
				{info->base_addr, end},
				{0, 0}
			}
		}
	};

	if (info->space == USER_SPACE) {
		int ret;
		ret = ptdump_walk_ts();
		if (ret)
			return ret;
	}

	if (info->has_user_space)
		pt_dump_seq_printf(m, "-----[User space]-----\n");
	else
		pt_dump_seq_printf(m, "-----[Kernel space]-----\n");

	if (info->name)
		pt_dump_seq_printf(m, "Find pid comm: %s\n", info->name);

	if (info->has_user_space) {
		pt_dump_seq_printf(m, "Real address space distribution:\n");
		pt_dump_seq_printf(m, "\tText code:    0x%016lx-0x%016lx\n", mm->start_code, mm->end_code);
		pt_dump_seq_printf(m, "\tData:         0x%016lx-0x%016lx\n", mm->start_data, mm->end_data);
		pt_dump_seq_printf(m, "\tBrk (heap):   0x%016lx-0x%016lx\n", mm->start_brk, mm->brk);
		pt_dump_seq_printf(m, "\tMmap (logic): 0x%016lx-0x%016lx\n", mm->mmap_base, find_mmap_logic_end(mm));
		pt_dump_seq_printf(m, "\tStack top:    0x%016lx\n", mm->start_stack);
		pt_dump_seq_printf(m, "\tArg:          0x%016lx-0x%016lx\n", mm->arg_start, mm->arg_end);
		pt_dump_seq_printf(m, "\tEnv:          0x%016lx-0x%016lx\n", mm->env_start, mm->env_end);

		pt_dump_seq_puts(m, "\n");
		if (info->check_mmap)
			return ptdump_walk_vma_mmap(m, mm->mmap);
	}
	pt_dump_seq_puts(m, "\n");

	if (info->check_addr) {
		unsigned long pfn = 0;
		int type = 0;

		pt_dump_seq_printf(m, "Find virt addr: 0x%016lx\n", info->check_addr);
		pt_dump_seq_printf(m, "Find result: ");

		pfn = virt_addr_to_pfn(mm, info->check_addr, &type);
		if (pfn) {
			struct resource *res;

			res = find_iomem_res(__pfn_to_phys(pfn) + offset_in_page(info->check_addr));
			if (res) {
				pt_dump_seq_printf(m, "%s [0x%016llx]", res->name, __pfn_to_phys(pfn) + offset_in_page(info->check_addr));
			} else {
				pt_dump_seq_printf(m, "I/O MEM or Resvered RAM [0x%016llx]", __pfn_to_phys(pfn) + offset_in_page(info->check_addr));
			}
			pt_dump_seq_printf(m, " %s\n", type ? "Mem Map" : "No Mem Map");
		} else
			pt_dump_seq_printf(m, "(null)\n");

		return 0;
	}
	if (!info->check_mmap)
		ptdump_walk_pgd(&st.ptdump, info->mm, NULL);
	else
		pt_dump_seq_printf(m, "No mmap info.\n");

	return 0;
}

static int ptdump_show(struct seq_file *m, void *v)
{
	int ret;
	struct ptdump_info *info = m->private;

	get_online_mems();
	ret = ptdump_walk_pgd_raw(m, info);
	put_online_mems();

	return ret;
}

static void ptdump_initialize(void)
{
	unsigned i, j;

	for (i = 0; i < ARRAY_SIZE(pg_level); i++)
		if (pg_level[i].bits)
			for (j = 0; j < pg_level[i].num; j++)
				pg_level[i].mask |= pg_level[i].bits[j].mask;
}

static int pid_page_tables_open(struct inode *inode, struct file *file)
{
	return single_open(file, &ptdump_show, inode->i_private);
}

static int parse_cmdline_str(char *str, int *space, unsigned long *pid, unsigned long *addr, int *check_mmap)
{
	size_t length;
	char *parse_buffer;
	char tmp_buffer[MAX_LENGTH] = { [0 ... MAX_LENGTH - 1 ] = 0};

	if (!space || !pid || !addr)
		return -EINVAL;

	parse_buffer = skip_spaces(str);

	if (strncmp(parse_buffer, "kernel", 6) == 0) {
		*space = KERNEL_SPACE;
		parse_buffer += 6;
	} else if (strncmp (parse_buffer, "pid", 3) == 0) {
		*space = USER_SPACE;
		parse_buffer += 3;

		parse_buffer = skip_spaces(parse_buffer);

		length = strcspn(parse_buffer, " ");
		if (length > MAX_LENGTH)
			return -EINVAL;

		strncpy(tmp_buffer, parse_buffer, length);
		parse_buffer += length;

		if (kstrtoul(tmp_buffer, 0, pid))
			return -EINVAL;
	} else
		return -EINVAL;

	parse_buffer = skip_spaces(parse_buffer);
	if (*parse_buffer == '\0')
		return 0;

	length = strcspn(parse_buffer, " ");
	if (length > MAX_LENGTH)
		return -EINVAL;

	strncpy(tmp_buffer, parse_buffer, length);
	if (strncmp(tmp_buffer, "mmap", 4) == 0) {
		if (check_mmap)
			*check_mmap = 1;
	} else if (kstrtoul(tmp_buffer, 0, addr))
		return -EINVAL;

	return 0;
}

static ssize_t pid_page_tables_write(struct file *f, const char __user *b, size_t s, loff_t *o)
{
	int ret = s;
	void *buffer;
	int space;
	unsigned long pid = 0;
	unsigned long addr = 0;
	int check_mmap = 0;

	buffer = kzalloc(s, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	
	if (copy_from_user(buffer, b, s)) {
		ret = -EFAULT;
		goto out;
	}

	if (parse_cmdline_str(buffer, &space, &pid, &addr, &check_mmap)) {
		ret = -EINVAL;
		goto out;
	}

	ptdump_info.check_addr = addr ? addr : 0;
	ptdump_info.check_mmap = check_mmap ? 1 : 0;

	if (space == KERNEL_SPACE) {
		mm.pgd = kernel_pgd;
		ptdump_info.mm = &mm;
		ptdump_info.markers = address_markers;
		ptdump_info.base_addr = PAGE_OFFSET;
		ptdump_info.name = NULL;
		ptdump_info.has_user_space = 0;
		ptdump_info.space = KERNEL_SPACE;
	} else {
		ptdump_info.space = USER_SPACE;
		ptdump_info.pid = pid;
	}
out:
	kfree(buffer);

	return ret;
}

static const struct file_operations pid_page_tables_fops = {
	.owner = THIS_MODULE,
	.open = pid_page_tables_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = pid_page_tables_write,
};

static int __init pid_page_tables_init(void)
{
	kernel_pgd = (pgd_t *)swapper_pg_dir;
	mm.pgd = kernel_pgd;
	ptdump_info.mm = &mm;

	address_markers[PAGE_END_NR].start_address = PAGE_END;
#if defined(CONFIG_KASAN_GENERIC) || defined(CONFIG_KASAN_SW_TAGS)
	address_markers[KASAN_START_NR].start_address = KASAN_SHADOW_START;
#endif

	ptdump_initialize();
	pid_page_tables_file = debugfs_create_file("pid_page_tables", 0444, NULL,
	&ptdump_info, &pid_page_tables_fops);

	return 0;
}

static void __exit pid_page_tables_exit(void)
{
	if (pid_page_tables_file)
		debugfs_remove(pid_page_tables_file);
}

module_init(pid_page_tables_init);
module_exit(pid_page_tables_exit);

MODULE_AUTHOR("yi.zhang@bst.ai");
MODULE_LICENSE("GPL");
MODULE_INFO(intree, "Y");
