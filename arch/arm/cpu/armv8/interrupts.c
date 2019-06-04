/*
 * (C) Copyright 2006
 * Stefan Roese, DENX Software Engineering, sr@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/config.h>
#include <asm/gic.h>
#include <asm/io.h>

static void gic_dist_init(void *base)
{
	unsigned int max_irq, i;
	/* Hard coding the cpumask as CPU0 */
	int cpumask = 1;

	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;

	writel(0, base + GICD_CTLR);

	/*
	 * Find out how many interrupts are supported.
	 */
	max_irq = readl(base + GICD_TYPER) & 0x1f;
	max_irq = (max_irq + 1) * 32;

	/*
	 * Set all global interrupts to be level triggered, active low.
	 */
	for (i = 32; i < max_irq; i += 16)
		writel(0, base + GICD_ICFGR + i * 4 / 16);

	/*
	 * Set all global interrupts to this CPU only.
	 */
	for (i = 32; i < max_irq; i += 4)
		writel(cpumask, base + GICD_ITARGETSRn + i * 4 / 4);

	/*
	 * Set priority on all interrupts.
	 */
	for (i = 0; i < max_irq; i += 4)
		writel(0xa0a0a0a0, base + GICD_IPRIORITYRn + i * 4 / 4);

	/*
	 * Disable all interrupts.
	 */
	for (i = 0; i < max_irq; i += 32)
		writel(0xffffffff, base +
				GICD_ICENABLERn + i * 4 / 32);

	writel(1, base + GICD_CTLR);
}

static void gic_cpu_init(void *base)
{
	writel(0xf0, base + GICC_PMR);
	writel(1, base + GICC_CTLR);
}

static void armv8_gic_init(void)
{
	gic_dist_init((void *)GIC_DIST_PHYS_BASE);
	gic_cpu_init((void *)GIC_CPU_PHYS_BASE);
}

/* route irq to EL2 if uboot is running with EL2 */
static inline void armv8_hcr_init(void)
{
	unsigned int v;
	asm volatile(
	"	mrs	x0, CurrentEL\n"
	"	cmp	x0, 0x8\n"
	"	b.ne	1f\n"
	"	mrs	x0, hcr_el2\n"
	"	orr	x0, x0, #(1 << 4)\n"
	"	msr	hcr_el2, x0\n"
	"1:\n"
	: "=&r" (v) : : "cc");
}

int arch_ack_irq(void)
{
	return gic_ack_irq();
}

void arch_eoi_irq(int iar)
{
	return gic_eoi_irq(iar);
}

int arch_enable_irq(int irq)
{
	if ((irq < 0) || (irq > N_IRQS)) {
		printf("irq %d is invalid!\n", irq);
		return -1;
	}

	writel((1 << (irq % 32)),
		GIC_DIST_PHYS_BASE + GICD_ISENABLERn + (irq / 32) * 4);
	return 0;
}

int arch_disable_irq(int irq)
{
	if ((irq < 0) || (irq > N_IRQS)) {
		printf("irq %d is invalid!\n", irq);
		return -1;
	}

	writel((1 << (irq % 32)),
		GIC_DIST_PHYS_BASE + GICD_ICENABLERn + (irq / 32) * 4);
	return 0;
}

int arch_interrupt_init(void)
{
	armv8_gic_init();
	armv8_hcr_init();

	return 0;
}
