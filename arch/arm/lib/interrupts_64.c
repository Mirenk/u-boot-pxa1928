/*
 * (C) Copyright 2013
 * David Feng <fenghua@phytium.com.cn>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <linux/compiler.h>
#include <asm/io.h>
#include <asm/arch/config.h>

#ifdef CONFIG_USE_IRQ
int interrupt_init(void)
{
	return arch_interrupt_init();
}

/* enable IRQ interrupts */
void enable_interrupts(void)
{
	__asm__ __volatile__("msr daifclr, #(1 << 1)");
}


/* disable IRQ/FIQ interrupts */
int disable_interrupts(void)
{
	__asm__ __volatile__("msr daifset, #(1 << 1)");
	return 1;
}

int disable_irq(int irq)
{
	return arch_disable_irq(irq);
}

int enable_irq(int irq)
{
	return arch_enable_irq(irq);
}

struct _irq_handler {
	void                *data;
	void (*func)(void *data);
};

static struct _irq_handler IRQ_HANDLER[N_IRQS];

void do_irq(struct pt_regs *pt_regs)
{
	int iar = arch_ack_irq();
	int irq = iar & 0x3ff;

	if (IRQ_HANDLER[irq].func)
		IRQ_HANDLER[irq].func(IRQ_HANDLER[irq].data);
	else
		printf("handler not registered for IRQ %d\n", irq);
	arch_eoi_irq(iar);
}

void irq_free_handler(int irq)
{
	if (irq >= N_IRQS || irq < 0) {
		printf("irq %d is invalid\n", irq);
		return;
	}

	arch_disable_irq(irq);
	IRQ_HANDLER[irq].data = 0;
	IRQ_HANDLER[irq].func = 0;
}

void irq_install_handler(int irq, interrupt_handler_t handle_irq, void *data)
{
	if (irq >= N_IRQS || irq < 0) {
		printf("irq %d is invalid\n", irq);
		return;
	}
	if (!handle_irq) {
		printf("irq %d's handle_irq is null\n", irq);
		return;
	}

	IRQ_HANDLER[irq].data = data;
	IRQ_HANDLER[irq].func = handle_irq;
	arch_enable_irq(irq);
}

#else

int interrupt_init(void)
{
	return 0;
}

void enable_interrupts(void)
{
	return;
}

int disable_interrupts(void)
{
	return 0;
}

int disable_irq(int irq)
{
	return 0;
}

int enable_irq(int irq)
{
	return 0;
}

void irq_free_handler(int irq)
{
	return;
}

void irq_install_handler(int irq, interrupt_handler_t handle_irq, void *data)
{
	return;
}

#endif

void show_regs(struct pt_regs *regs)
{
	int i;

	printf("ELR:     %lx\n", regs->elr);
	printf("LR:      %lx\n", regs->regs[30]);
	for (i = 0; i < 29; i += 2)
		printf("x%-2d: %016lx x%-2d: %016lx\n",
		       i, regs->regs[i], i+1, regs->regs[i+1]);
	printf("\n");
}

/*
 * do_bad_sync handles the impossible case in the Synchronous Abort vector.
 */
void do_bad_sync(struct pt_regs *pt_regs, unsigned int esr)
{
	printf("Bad mode in \"Synchronous Abort\" handler, esr 0x%08x\n", esr);
	show_regs(pt_regs);
	panic("Resetting CPU ...\n");
}

/*
 * do_bad_irq handles the impossible case in the Irq vector.
 */
void do_bad_irq(struct pt_regs *pt_regs, unsigned int esr)
{
	printf("Bad mode in \"Irq\" handler, esr 0x%08x\n", esr);
	show_regs(pt_regs);
	panic("Resetting CPU ...\n");
}

/*
 * do_bad_fiq handles the impossible case in the Fiq vector.
 */
void do_bad_fiq(struct pt_regs *pt_regs, unsigned int esr)
{
	printf("Bad mode in \"Fiq\" handler, esr 0x%08x\n", esr);
	show_regs(pt_regs);
	panic("Resetting CPU ...\n");
}

/*
 * do_bad_error handles the impossible case in the Error vector.
 */
void do_bad_error(struct pt_regs *pt_regs, unsigned int esr)
{
	printf("Bad mode in \"Error\" handler, esr 0x%08x\n", esr);
	show_regs(pt_regs);
	panic("Resetting CPU ...\n");
}

/*
 * do_sync handles the Synchronous Abort exception.
 */
void do_sync(struct pt_regs *pt_regs, unsigned int esr)
{
	printf("\"Synchronous Abort\" handler, esr 0x%08x\n", esr);
	show_regs(pt_regs);
	panic("Resetting CPU ...\n");
}

#ifndef CONFIG_USE_IRQ
/*
 * do_irq handles the Irq exception.
 */
void do_irq(struct pt_regs *pt_regs, unsigned int esr)
{
	printf("\"Irq\" handler, esr 0x%08x\n", esr);
	show_regs(pt_regs);
	panic("Resetting CPU ...\n");
}
#endif

/*
 * do_fiq handles the Fiq exception.
 */
void do_fiq(struct pt_regs *pt_regs, unsigned int esr)
{
	printf("\"Fiq\" handler, esr 0x%08x\n", esr);
	show_regs(pt_regs);
	panic("Resetting CPU ...\n");
}

/*
 * do_error handles the Error exception.
 * Errors are more likely to be processor specific,
 * it is defined with weak attribute and can be redefined
 * in processor specific code.
 */
void __weak do_error(struct pt_regs *pt_regs, unsigned int esr)
{
	printf("\"Error\" handler, esr 0x%08x\n", esr);
	show_regs(pt_regs);
	panic("Resetting CPU ...\n");
}
