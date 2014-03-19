/*
 * Copyright (c) 2011-2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - Power Management support
 *
 * Based on arch/arm/mach-s3c2410/pm.c
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/of_address.h>

#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/smp_scu.h>
#include <asm/suspend.h>

#include <plat/pm-common.h>
#include <plat/pll.h>
#include <plat/regs-srom.h>

#include <mach/map.h>

#include "common.h"
#include "regs-pmu.h"

#define EXYNOS5420_CPU_ADDR	0x1c
#define EXYNOS5420_CPU_STATE	0x28

static int exynos5420_cpu_state[2];
static void __iomem *exynos5420_sysram_base;
static void __iomem *exynos5420_ns_sysram_base;

/**
 * struct exynos_wkup_irq - Exynos GIC to PMU IRQ mapping
 * @hwirq: Hardware IRQ signal of the GIC
 * @mask: Mask in PMU wake-up mask register
 */
struct exynos_wkup_irq {
	unsigned int hwirq;
	u32 mask;
};

static struct sleep_save exynos5_sys_save[] = {
	SAVE_ITEM(EXYNOS5_SYS_I2C_CFG),
};

static struct sleep_save exynos_core_save[] = {
	/* SROM side */
	SAVE_ITEM(S5P_SROM_BW),
	SAVE_ITEM(S5P_SROM_BC0),
	SAVE_ITEM(S5P_SROM_BC1),
	SAVE_ITEM(S5P_SROM_BC2),
	SAVE_ITEM(S5P_SROM_BC3),
};

static struct sleep_save exynos5420_reg_save[] = {
	SAVE_ITEM(EXYNOS5_SYS_DISP1_BLK_CFG),
	SAVE_ITEM(S5P_PMU_SPARE3),
};


/*
 * GIC wake-up support
 */

static u32 exynos_irqwake_intmask = 0xffffffff;

static const struct exynos_wkup_irq exynos4_wkup_irq[] = {
	{ 76, BIT(1) }, /* RTC alarm */
	{ 77, BIT(2) }, /* RTC tick */
	{ /* sentinel */ },
};

static const struct exynos_wkup_irq exynos5250_wkup_irq[] = {
	{ 75, BIT(1) }, /* RTC alarm */
	{ 76, BIT(2) }, /* RTC tick */
	{ /* sentinel */ },
};

static int exynos_irq_set_wake(struct irq_data *data, unsigned int state)
{
	const struct exynos_wkup_irq *wkup_irq;

	if (soc_is_exynos5250() || soc_is_exynos5420())
		wkup_irq = exynos5250_wkup_irq;
	else
		wkup_irq = exynos4_wkup_irq;

	while (wkup_irq->mask) {
		if (wkup_irq->hwirq == data->hwirq) {
			if (!state)
				exynos_irqwake_intmask |= wkup_irq->mask;
			else
				exynos_irqwake_intmask &= ~wkup_irq->mask;
			return 0;
		}
		++wkup_irq;
	}

	return -ENOENT;
}

/**
 * exynos_core_power_down : power down the specified cpu
 * @cpu : the cpu to power down
 *
 * Power down the specified cpu. The sequence must be finished by a
 * call to cpu_do_idle()
 *
 */
void exynos_cpu_power_down(int cpu)
{
	__raw_writel(0, EXYNOS_ARM_CORE_CONFIGURATION(cpu));
}

/**
 * exynos_cpu_power_up : power up the specified cpu
 * @cpu : the cpu to power up
 *
 * Power up the specified cpu
 */
void exynos_cpu_power_up(int cpu)
{
	__raw_writel(S5P_CORE_LOCAL_PWR_EN,
		     EXYNOS_ARM_CORE_CONFIGURATION(cpu));
}

/**
 * exynos_cpu_power_state : returns the power state of the cpu
 * @cpu : the cpu to retrieve the power state from
 *
 */
int exynos_cpu_power_state(int cpu)
{
	return (__raw_readl(EXYNOS_ARM_CORE_STATUS(cpu)) &
			S5P_CORE_LOCAL_PWR_EN);
}

/**
 * exynos_cluster_power_down : power down the specified cluster
 * @cluster : the cluster to power down
 */
void exynos_cluster_power_down(int cluster)
{
	__raw_writel(0, EXYNOS_COMMON_CONFIGURATION(cluster));
}

/**
 * exynos_cluster_power_up : power up the specified cluster
 * @cluster : the cluster to power up
 */
void exynos_cluster_power_up(int cluster)
{
	__raw_writel(S5P_CORE_LOCAL_PWR_EN,
		     EXYNOS_COMMON_CONFIGURATION(cluster));
}

/**
 * exynos_cluster_power_state : returns the power state of the cluster
 * @cluster : the cluster to retrieve the power state from
 *
 */
int exynos_cluster_power_state(int cluster)
{
	return (__raw_readl(EXYNOS_COMMON_STATUS(cluster)) &
			S5P_CORE_LOCAL_PWR_EN);
}

#define EXYNOS_BOOT_VECTOR_ADDR	(samsung_rev() == EXYNOS4210_REV_1_1 ? \
			S5P_INFORM7 : (samsung_rev() == EXYNOS4210_REV_1_0 ? \
			(sysram_base_addr + 0x24) : S5P_INFORM0))
#define EXYNOS_BOOT_VECTOR_FLAG	(samsung_rev() == EXYNOS4210_REV_1_1 ? \
			S5P_INFORM6 : (samsung_rev() == EXYNOS4210_REV_1_0 ? \
			(sysram_base_addr + 0x20) : S5P_INFORM1))

#define S5P_CHECK_AFTR  0xFCBA0D10
#define S5P_CHECK_SLEEP 0x00000BAD

/* Ext-GIC nIRQ/nFIQ is the only wakeup source in AFTR */
static void exynos_set_wakeupmask(long mask)
{
	__raw_writel(mask, S5P_WAKEUP_MASK);
}

static void exynos_cpu_set_boot_vector(long flags)
{
	__raw_writel(virt_to_phys(exynos_cpu_resume), EXYNOS_BOOT_VECTOR_ADDR);
	__raw_writel(flags, EXYNOS_BOOT_VECTOR_FLAG);
}

void exynos_enter_aftr(void)
{
	exynos_set_wakeupmask(0x0000ff3e);
	exynos_cpu_set_boot_vector(S5P_CHECK_AFTR);
	/* Set value of power down register for aftr mode */
	exynos_sys_powerdown_conf(SYS_AFTR);
}

/* For Cortex-A9 Diagnostic and Power control register */
static unsigned int save_arm_register[2];

static void exynos_cpu_save_register(void)
{
	unsigned long tmp;

	/* Save Power control register */
	asm ("mrc p15, 0, %0, c15, c0, 0"
	     : "=r" (tmp) : : "cc");

	save_arm_register[0] = tmp;

	/* Save Diagnostic register */
	asm ("mrc p15, 0, %0, c15, c0, 1"
	     : "=r" (tmp) : : "cc");

	save_arm_register[1] = tmp;
}

static void exynos_cpu_restore_register(void)
{
	unsigned long tmp;

	/* Restore Power control register */
	tmp = save_arm_register[0];

	asm volatile ("mcr p15, 0, %0, c15, c0, 0"
		      : : "r" (tmp)
		      : "cc");

	/* Restore Diagnostic register */
	tmp = save_arm_register[1];

	asm volatile ("mcr p15, 0, %0, c15, c0, 1"
		      : : "r" (tmp)
		      : "cc");
}

static int exynos_cpu_suspend(unsigned long arg)
{
#ifdef CONFIG_CACHE_L2X0
	outer_flush_all();
#endif

	/*
	 * Clear IRAM register for cpu state so that primary CPU does
	 * not enter low power start in U-Boot.
	 * This is specific to exynos5420 SoC only.
	 */
	if (soc_is_exynos5420())
	__raw_writel(0x0,
			exynos5420_sysram_base + EXYNOS5420_CPU_STATE);

	if (soc_is_exynos5250() || soc_is_exynos5420())
		flush_cache_all();

	/* issue the standby signal into the pm unit. */
	cpu_do_idle();

	pr_info("Failed to suspend the system\n");
	return 1; /* Aborting suspend */
}

static void exynos_pm_prepare(void)
{
	unsigned int tmp;

	/* Set wake-up mask registers */
	__raw_writel(exynos_get_eint_wake_mask(), S5P_EINT_WAKEUP_MASK);
	__raw_writel(exynos_irqwake_intmask & ~(1 << 31), S5P_WAKEUP_MASK);

	s3c_pm_do_save(exynos_core_save, ARRAY_SIZE(exynos_core_save));

	if (soc_is_exynos5250()) {
		s3c_pm_do_save(exynos5_sys_save, ARRAY_SIZE(exynos5_sys_save));
		/* Disable USE_RETENTION of JPEG_MEM_OPTION */
		tmp = __raw_readl(EXYNOS5_JPEG_MEM_OPTION);
		tmp &= ~EXYNOS5_OPTION_USE_RETENTION;
		__raw_writel(tmp, EXYNOS5_JPEG_MEM_OPTION);
	} else if (soc_is_exynos5420()) {

		s3c_pm_do_save(exynos5420_reg_save,
			ARRAY_SIZE(exynos5420_reg_save));

		/*
		 * The cpu state needs to be saved and restored so that the
		 * secondary CPUs will enter low power start. Though the U-Boot
		 * is setting the cpu state with low power flag, the kernel
		 * needs to restore it back in case, the primary cpu fails to
		 * suspend for any reason
		 */
		exynos5420_cpu_state[0] =
		__raw_readl(exynos5420_sysram_base + EXYNOS5420_CPU_STATE);
		exynos5420_cpu_state[1] =
		__raw_readl(exynos5420_ns_sysram_base + EXYNOS5420_CPU_ADDR);
	}

	/* Set value of power down register for sleep mode */

	exynos_sys_powerdown_conf(SYS_SLEEP);
	__raw_writel(S5P_CHECK_SLEEP, S5P_INFORM1);

	/* ensure at least INFORM0 has the resume address */

	__raw_writel(virt_to_phys(exynos_cpu_resume), S5P_INFORM0);

	if (soc_is_exynos5420()) {

		tmp = __raw_readl(EXYNOS5_ARM_L2_OPTION);
		tmp &= ~EXYNOS5_USE_RETENTION;
		__raw_writel(tmp, EXYNOS5_ARM_L2_OPTION);

		tmp = __raw_readl(EXYNOS5420_SFR_AXI_CGDIS1);
		tmp |= EXYNOS5420_UFS;
		__raw_writel(tmp, EXYNOS5420_SFR_AXI_CGDIS1);

		tmp = __raw_readl(EXYNOS5420_ARM_COMMON_OPTION);
		tmp &= ~EXYNOS5420_L2RSTDISABLE_VALUE;
		__raw_writel(tmp, EXYNOS5420_ARM_COMMON_OPTION);
		tmp = __raw_readl(EXYNOS5420_FSYS2_OPTION);
		tmp |= EXYNOS5420_EMULATION;
		__raw_writel(tmp, EXYNOS5420_FSYS2_OPTION);
		tmp = __raw_readl(EXYNOS5420_PSGEN_OPTION);
		tmp |= EXYNOS5420_EMULATION;
		__raw_writel(tmp, EXYNOS5420_PSGEN_OPTION);
	}
}

static void exynos_pm_central_suspend(void)
{
	unsigned long tmp;

	/* Setting Central Sequence Register for power down mode */
	tmp = __raw_readl(S5P_CENTRAL_SEQ_CONFIGURATION);
	tmp &= ~S5P_CENTRAL_LOWPWR_CFG;
	__raw_writel(tmp, S5P_CENTRAL_SEQ_CONFIGURATION);
}

static int exynos_pm_suspend(void)
{
	unsigned long tmp;
	unsigned int cluster_id;
	exynos_pm_central_suspend();

	/* Setting SEQ_OPTION register */

	if (soc_is_exynos5420()) {
		cluster_id = (read_cpuid(CPUID_MPIDR) >> 8) & 0xf;
		if (!cluster_id)
			__raw_writel(EXYNOS5420_ARM_USE_STANDBY_WFI0,
				     S5P_CENTRAL_SEQ_OPTION);
		else
			__raw_writel(EXYNOS5420_KFC_USE_STANDBY_WFI0,
				     S5P_CENTRAL_SEQ_OPTION);
	} else {
		tmp = (S5P_USE_STANDBY_WFI0 | S5P_USE_STANDBY_WFE0);
		__raw_writel(tmp, S5P_CENTRAL_SEQ_OPTION);
	}

	if (read_cpuid_part_number() == ARM_CPU_PART_CORTEX_A9)
		exynos_cpu_save_register();

	return 0;
}

static int exynos_pm_central_resume(void)
{
	unsigned long tmp;

	/*
	 * If PMU failed while entering sleep mode, WFI will be
	 * ignored by PMU and then exiting cpu_do_idle().
	 * S5P_CENTRAL_LOWPWR_CFG bit will not be set automatically
	 * in this situation.
	 */
	tmp = __raw_readl(S5P_CENTRAL_SEQ_CONFIGURATION);
	if (!(tmp & S5P_CENTRAL_LOWPWR_CFG)) {
		tmp |= S5P_CENTRAL_LOWPWR_CFG;
		__raw_writel(tmp, S5P_CENTRAL_SEQ_CONFIGURATION);
		/* clear the wakeup state register */
		__raw_writel(0x0, S5P_WAKEUP_STAT);
		/* No need to perform below restore code */
		return -1;
	}

	return 0;
}

static void exynos_pm_resume(void)
{
	unsigned long tmp;

	if (soc_is_exynos5420()) {
		/* Restore the IRAM register cpu state */
		__raw_writel(exynos5420_cpu_state[0],
		exynos5420_sysram_base + EXYNOS5420_CPU_STATE);
		__raw_writel(exynos5420_cpu_state[1],
		exynos5420_ns_sysram_base + EXYNOS5420_CPU_ADDR);

		__raw_writel(EXYNOS5420_USE_STANDBY_WFI_ALL,
			S5P_CENTRAL_SEQ_OPTION);
	}

	if (exynos_pm_central_resume())
		goto early_wakeup;

	if (read_cpuid_part_number() == ARM_CPU_PART_CORTEX_A9)
		exynos_cpu_restore_register();

	/* For release retention */
	if (soc_is_exynos5420()) {
		__raw_writel(1 << 28, EXYNOS_PAD_RET_DRAM_OPTION);
		__raw_writel(1 << 28, EXYNOS_PAD_RET_MAUDIO_OPTION);
		__raw_writel(1 << 28, EXYNOS_PAD_RET_JTAG_OPTION);
		__raw_writel(1 << 28, EXYNOS5420_PAD_RET_GPIO_OPTION);
		__raw_writel(1 << 28, EXYNOS5420_PAD_RET_UART_OPTION);
		__raw_writel(1 << 28, EXYNOS5420_PAD_RET_MMCA_OPTION);
		__raw_writel(1 << 28, EXYNOS5420_PAD_RET_MMCB_OPTION);
		__raw_writel(1 << 28, EXYNOS5420_PAD_RET_MMCC_OPTION);
		__raw_writel(1 << 28, EXYNOS5420_PAD_RET_HSI_OPTION);
		__raw_writel(1 << 28, EXYNOS_PAD_RET_EBIA_OPTION);
		__raw_writel(1 << 28, EXYNOS_PAD_RET_EBIB_OPTION);
		__raw_writel(1 << 28, EXYNOS5420_PAD_RET_SPI_OPTION);
		__raw_writel(1 << 28, EXYNOS5420_PAD_RET_DRAM_COREBLK_OPTION);
	} else {
		__raw_writel((1 << 28), S5P_PAD_RET_MAUDIO_OPTION);
		__raw_writel((1 << 28), S5P_PAD_RET_GPIO_OPTION);
		__raw_writel((1 << 28), S5P_PAD_RET_UART_OPTION);
		__raw_writel((1 << 28), S5P_PAD_RET_MMCA_OPTION);
		__raw_writel((1 << 28), S5P_PAD_RET_MMCB_OPTION);
		__raw_writel((1 << 28), S5P_PAD_RET_EBIA_OPTION);
		__raw_writel((1 << 28), S5P_PAD_RET_EBIB_OPTION);
	}
	if (soc_is_exynos5250())
		s3c_pm_do_restore(exynos5_sys_save,
			ARRAY_SIZE(exynos5_sys_save));
	else if (soc_is_exynos5420())
		s3c_pm_do_restore(exynos5420_reg_save,
			ARRAY_SIZE(exynos5420_reg_save));

	s3c_pm_do_restore_core(exynos_core_save, ARRAY_SIZE(exynos_core_save));

	if (read_cpuid_part_number() == ARM_CPU_PART_CORTEX_A9)
		scu_enable(S5P_VA_SCU);

early_wakeup:

	if (soc_is_exynos5420()) {
		tmp = __raw_readl(EXYNOS5420_SFR_AXI_CGDIS1);
		tmp &= ~EXYNOS5420_UFS;
		__raw_writel(tmp, EXYNOS5420_SFR_AXI_CGDIS1);
		tmp = __raw_readl(EXYNOS5420_FSYS2_OPTION);
		tmp &= ~EXYNOS5420_EMULATION;
		__raw_writel(tmp, EXYNOS5420_FSYS2_OPTION);
		tmp = __raw_readl(EXYNOS5420_PSGEN_OPTION);
		tmp &= ~EXYNOS5420_EMULATION;
		__raw_writel(tmp, EXYNOS5420_PSGEN_OPTION);
	}

	/* Clear SLEEP mode set in INFORM1 */
	__raw_writel(0x0, S5P_INFORM1);

	return;
}

static struct syscore_ops exynos_pm_syscore_ops = {
	.suspend	= exynos_pm_suspend,
	.resume		= exynos_pm_resume,
};

/*
 * Suspend Ops
 */

static int exynos_suspend_enter(suspend_state_t state)
{
	int ret;

	s3c_pm_debug_init();

	S3C_PMDBG("%s: suspending the system...\n", __func__);

	S3C_PMDBG("%s: wakeup masks: %08x,%08x\n", __func__,
			exynos_irqwake_intmask, exynos_get_eint_wake_mask());

	if (exynos_irqwake_intmask == -1U
	    && exynos_get_eint_wake_mask() == -1U) {
		pr_err("%s: No wake-up sources!\n", __func__);
		pr_err("%s: Aborting sleep\n", __func__);
		return -EINVAL;
	}

	s3c_pm_save_uarts();
	exynos_pm_prepare();
	flush_cache_all();
	s3c_pm_check_store();

	ret = cpu_suspend(0, exynos_cpu_suspend);
	if (ret)
		return ret;

	s3c_pm_restore_uarts();

	S3C_PMDBG("%s: wakeup stat: %08x\n", __func__,
			__raw_readl(S5P_WAKEUP_STAT));

	s3c_pm_check_restore();

	S3C_PMDBG("%s: resuming the system...\n", __func__);

	return 0;
}

static int exynos_suspend_prepare(void)
{
	s3c_pm_check_prepare();

	return 0;
}

static void exynos_suspend_finish(void)
{
	s3c_pm_check_cleanup();
}

static const struct platform_suspend_ops exynos_suspend_ops = {
	.enter		= exynos_suspend_enter,
	.prepare	= exynos_suspend_prepare,
	.finish		= exynos_suspend_finish,
	.valid		= suspend_valid_only_mem,
};

static int exynos_cpu_pm_notifier(struct notifier_block *self,
				  unsigned long cmd, void *v)
{
	int cpu = smp_processor_id();

	switch (cmd) {
	case CPU_PM_ENTER:
		if (cpu == 0) {
			exynos_pm_central_suspend();
			if (read_cpuid_part_number() == ARM_CPU_PART_CORTEX_A9)
			exynos_cpu_save_register();
		}
		break;

	case CPU_PM_EXIT:
		if (cpu == 0) {
			if (read_cpuid_part_number() ==
					ARM_CPU_PART_CORTEX_A9) {
				scu_enable(S5P_VA_SCU);
				exynos_cpu_restore_register();
			}
			exynos_pm_central_resume();
		}
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_cpu_pm_notifier_block = {
	.notifier_call = exynos_cpu_pm_notifier,
};

void __init exynos_pm_init(void)
{
	struct device_node *node;
	u32 tmp;

	if (soc_is_exynos5420()) {
		node = of_find_compatible_node(NULL, NULL,
			"samsung,exynos4210-sysram");
		if (!node) {
			pr_err("failed to find secure sysram node\n");
			return;
		}

		exynos5420_sysram_base = of_iomap(node, 0);
		of_node_put(node);
		if (!exynos5420_sysram_base) {
			pr_err("failed to map secure sysram base address\n");
			return;
		}

		node = of_find_compatible_node(NULL, NULL,
			"samsung,exynos4210-sysram-ns");
		if (!node) {
			pr_err("failed to find non-secure sysram node\n");
			iounmap(exynos5420_sysram_base);
			return;
		}

		exynos5420_ns_sysram_base = of_iomap(node, 0);
		of_node_put(node);
		if (!exynos5420_ns_sysram_base) {
			pr_err("failed to map non-secure sysram base address\n");
			iounmap(exynos5420_sysram_base);
			return;
		}
	}

	cpu_pm_register_notifier(&exynos_cpu_pm_notifier_block);

	/* Platform-specific GIC callback */
	gic_arch_extn.irq_set_wake = exynos_irq_set_wake;

	/* All wakeup disable */
	if (soc_is_exynos5420()) {
		tmp = __raw_readl(S5P_WAKEUP_MASK);
		tmp |= ((0x7F << 7) | (0x1F << 1));
		__raw_writel(tmp, S5P_WAKEUP_MASK);
	} else {
		tmp = __raw_readl(S5P_WAKEUP_MASK);
		tmp |= ((0xFF << 8) | (0x1F << 1));
		__raw_writel(tmp, S5P_WAKEUP_MASK);
	}
	register_syscore_ops(&exynos_pm_syscore_ops);
	suspend_set_ops(&exynos_suspend_ops);
}
