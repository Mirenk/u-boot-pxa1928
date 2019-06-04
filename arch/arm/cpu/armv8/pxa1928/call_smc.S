#include <linux/linkage.h>
#include <asm/assembler.h>

/*
 * u32 smc_config_timer2(u64 fid, struct void *params)
 */
ENTRY(smc_config_timer2)
	sub sp, sp, #8
	str x19, [sp]
	mov x19, x1
	ldr x1, [x19, #0]
	ldr x2, [x19, #8]
	ldr x3, [x19, #16]
	smc #0
	str x0, [x19, #16]
	ldr x19, [sp]
	add sp, sp, #8
	ret
ENDPROC(smc_config_timer2)

/*
 * int smc_get_fuse_info(u64 function_id, void *arg)
 */
ENTRY(smc_get_fuse_info)
	sub sp, sp, #16
	str x19, [sp]
	mov x19, x1
	smc #0
	str w1, [x19], #4
	str w2, [x19], #4
	str w3, [x19], #4
	str w4, [x19], #4
	str w5, [x19], #4
	str w6, [x19], #4
	ldr x19, [sp]
	add sp, sp, #16
	ret
ENDPROC(smc_get_fuse_info)
