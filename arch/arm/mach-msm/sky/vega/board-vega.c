/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 * Copyright (c) 2013, Pantech Co Ltd.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/smp.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#ifdef CONFIG_SMB349_CHARGER
#include <linux/i2c/smb349.h>
#endif
#include <linux/i2c/sx150x.h>
#include <linux/slimbus/slimbus.h>
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/msm_ssbi.h>
#include <linux/spi/spi.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/qcom_crypto_device.h>
#include <linux/msm_ion.h>
#include <linux/memory.h>
#include <linux/memblock.h>
#include <linux/msm_thermal.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/cyttsp-qc.h>
#include <linux/i2c/isa1200.h>
#include <linux/gpio_keys.h>
#include <linux/epm_adc.h>
#include <linux/i2c/sx150x.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach/mmc.h>
#include <linux/platform_data/qcom_wcnss_device.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/ion.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/usb/android.h>
#include <mach/socinfo.h>
#include <mach/msm_spi.h>
#include "timer.h"
#include "devices.h"
#include <mach/gpiomux.h>
#include <mach/rpm.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#include <mach/msm_memtypes.h>
#include <linux/bootmem.h>
#include <asm/setup.h>
#include <mach/dma.h>
#include <mach/msm_dsps.h>
#include <mach/msm_bus_board.h>
#include <mach/cpuidle.h>
#include <mach/mdm2.h>
#include <linux/msm_tsens.h>
#include <mach/msm_xo.h>
#include <mach/msm_rtb.h>
#include <sound/cs8427.h>
#include <media/gpio-ir-recv.h>
#include <linux/fmem.h>
#include <mach/msm_pcie.h>
#include <mach/restart.h>
#include <mach/msm_iomap.h>
#include <ram_console.h>
#include <linux/persistent_ram.h>

#include "msm_watchdog.h"
#include "board-vega.h"
#include "clock.h"
#include "spm.h"
#include <mach/mpm.h>
#include "rpm_resources.h"
#include "pm.h"
#include "pm-boot.h"
#include "devices-msm8x60.h"
#include "smd_private.h"
#include "sysmon.h"

#define MSM_PMEM_ADSP_SIZE         0x7800000
#define MSM_PMEM_AUDIO_SIZE        0x4CF000
#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
#define MSM_PMEM_SIZE 0x4000000 /* 64 Mbytes */
#else
#define MSM_PMEM_SIZE 0x4000000 /* 64 Mbytes */
#endif

#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
#define HOLE_SIZE		0x20000
#define MSM_ION_MFC_META_SIZE  0x40000 /* 256 Kbytes */
#define MSM_CONTIG_MEM_SIZE  0x65000
#ifdef CONFIG_MSM_IOMMU
#define MSM_ION_MM_SIZE		0x4800000
#define MSM_ION_SF_SIZE		0
#define MSM_ION_QSECOM_SIZE	0x780000 /* (7.5MB) */
#define MSM_ION_HEAP_NUM	8
#else
#define MSM_ION_MM_SIZE		MSM_PMEM_ADSP_SIZE
#define MSM_ION_SF_SIZE		MSM_PMEM_SIZE
#define MSM_ION_QSECOM_SIZE	0x600000 /* (6MB) */
#define MSM_ION_HEAP_NUM	8
#endif
#define MSM_ION_MM_FW_SIZE	(0x200000 - HOLE_SIZE) /* (2MB - 128KB) */
#define MSM_ION_MFC_SIZE	(SZ_8K + MSM_ION_MFC_META_SIZE)
#define MSM_ION_AUDIO_SIZE	MSM_PMEM_AUDIO_SIZE
#else
#define MSM_CONTIG_MEM_SIZE  0x110C000
#define MSM_ION_HEAP_NUM	1
#endif

#define APQ8064_FIXED_AREA_START (0xa0000000 - (MSM_ION_MM_FW_SIZE + \
							HOLE_SIZE))
#define MAX_FIXED_AREA_SIZE	0x10000000
#define MSM_MM_FW_SIZE		(0x200000 - HOLE_SIZE)
#define APQ8064_FW_START	APQ8064_FIXED_AREA_START
#define MSM_ION_ADSP_SIZE	SZ_8M

#define QFPROM_RAW_FEAT_CONFIG_ROW0_MSB     (MSM_QFPROM_BASE + 0x23c)
#define QFPROM_RAW_OEM_CONFIG_ROW0_LSB      (MSM_QFPROM_BASE + 0x220)

/* PCIE AXI address space */
#define PCIE_AXI_BAR_PHYS   0x08000000
#define PCIE_AXI_BAR_SIZE   SZ_128M

/* PCIe pmic gpios */
#define PCIE_WAKE_N_PMIC_GPIO 12
#define PCIE_PWR_EN_PMIC_GPIO 13
#define PCIE_RST_N_PMIC_MPP 1


#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define VEGA_RAM_CONSOLE_SIZE (128*SZ_1K * 2)

static char bootreason[128] = {0,};

struct ram_console_platform_data ram_console_pdata = {
        .bootinfo = bootreason,
};

static struct platform_device ram_console_device = {
        .name = "ram_console",
        .id = -1,
        .dev = {
                .platform_data = &ram_console_pdata,
        }
};

void __init vega_add_ramconsole_devices(void)
{
        platform_device_register(&ram_console_device);
}

#else
static inline void __init vega_add_ramconsole_devices(void)
{
        /* empty */
}
#endif

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
#define VEGA_PERSISTENT_RAM_SIZE (SZ_1M)

static struct persistent_ram_descriptor pram_descs[] = {
#ifdef CONFIG_ANDROID_RAM_CONSOLE
        {
                .name = "ram_console",
                .size      = VEGA_RAM_CONSOLE_SIZE,
        }
#endif
};


static struct persistent_ram vega_persistent_ram = {
        .size = VEGA_RAM_CONSOLE_SIZE,
        .num_descs = ARRAY_SIZE(pram_descs),
        .descs = pram_descs,
};

void __init vega_add_persistent_ram(void)
{
        struct persistent_ram *pram = &vega_persistent_ram;
        struct membank *bank = &meminfo.bank[0];

        pram->start = bank->start + bank->size - VEGA_PERSISTENT_RAM_SIZE;

        persistent_ram_early_init(pram);

}
#else
static inline void __init vega_add_persistent_ram(void)
{

}
#endif

void __init vega_reserve(void)
{
        vega_add_persistent_ram();
}

#ifdef CONFIG_KERNEL_MSM_CONTIG_MEM_REGION
static unsigned msm_contig_mem_size = MSM_CONTIG_MEM_SIZE;
static int __init msm_contig_mem_size_setup(char *p)
{
	msm_contig_mem_size = memparse(p, NULL);
	return 0;
}
early_param("msm_contig_mem_size", msm_contig_mem_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_size = MSM_PMEM_SIZE;
static int __init pmem_size_setup(char *p)
{
	pmem_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_size", pmem_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;

static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device apq8064_android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device apq8064_android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device apq8064_android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};
#endif /* CONFIG_MSM_MULTIMEDIA_USE_ION */
#endif /* CONFIG_ANDROID_PMEM */

struct fmem_platform_data apq8064_fmem_pdata = {
};

static struct memtype_reserve apq8064_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init reserve_rtb_memory(void)
{
#if defined(CONFIG_MSM_RTB)
	apq8064_reserve_table[MEMTYPE_EBI1].size += apq8064_rtb_pdata.size;
	pr_info("mem_map: rtb reserved with size 0x%x in pool\n",
			apq8064_rtb_pdata.size);
#endif
}


static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_size;
	android_pmem_audio_pdata.size = MSM_PMEM_AUDIO_SIZE;
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/
}

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	apq8064_reserve_table[p->memory_type].size += p->size;
}
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
	apq8064_reserve_table[MEMTYPE_EBI1].size += msm_contig_mem_size;
	pr_info("mem_map: contig_mem reserved with size 0x%x in pool\n",
			msm_contig_mem_size);
#endif /*CONFIG_ANDROID_PMEM*/
}

static int apq8064_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

#define FMEM_ENABLED 0

#ifdef CONFIG_ION_MSM
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct ion_cp_heap_pdata cp_mm_apq8064_ion_pdata = {
	.permission_type = IPT_TYPE_MM_CARVEOUT,
	.align = PAGE_SIZE,
	.reusable = FMEM_ENABLED,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_MIDDLE,
	.is_cma = 1,
	.no_nonsecure_alloc = 1,
};

static struct ion_cp_heap_pdata cp_mfc_apq8064_ion_pdata = {
	.permission_type = IPT_TYPE_MFC_SHAREDMEM,
	.align = PAGE_SIZE,
	.reusable = 0,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_HIGH,
	.no_nonsecure_alloc = 1,
};

static struct ion_co_heap_pdata co_apq8064_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
	.mem_is_fmem = 0,
};

static struct ion_co_heap_pdata fw_co_apq8064_ion_pdata = {
	.adjacent_mem_id = ION_CP_MM_HEAP_ID,
	.align = SZ_128K,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_LOW,
};
#endif

static u64 msm_dmamask = DMA_BIT_MASK(32);

static struct platform_device ion_mm_heap_device = {
	.name = "ion-mm-heap-device",
	.id = -1,
	.dev = {
		.dma_mask = &msm_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

static struct platform_device ion_adsp_heap_device = {
	.name = "ion-adsp-heap-device",
	.id = -1,
	.dev = {
		.dma_mask = &msm_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

/**
 * These heaps are listed in the order they will be allocated. Due to
 * video hardware restrictions and content protection the FW heap has to
 * be allocated adjacent (below) the MM heap and the MFC heap has to be
 * allocated after the MM heap to ensure MFC heap is not more than 256MB
 * away from the base address of the FW heap.
 * However, the order of FW heap and MM heap doesn't matter since these
 * two heaps are taken care of by separate code to ensure they are adjacent
 * to each other.
 * Don't swap the order unless you know what you are doing!
 */
struct ion_platform_heap apq8064_heaps[] = {
		{
			.id	= ION_SYSTEM_HEAP_ID,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= ION_VMALLOC_HEAP_NAME,
		},
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		{
			.id	= ION_CP_MM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MM_HEAP_NAME,
			.size	= MSM_ION_MM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mm_apq8064_ion_pdata,
			.priv	= &ion_mm_heap_device.dev
		},
		{
			.id	= ION_MM_FIRMWARE_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_MM_FIRMWARE_HEAP_NAME,
			.size	= MSM_ION_MM_FW_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &fw_co_apq8064_ion_pdata,
		},
		{
			.id	= ION_CP_MFC_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MFC_HEAP_NAME,
			.size	= MSM_ION_MFC_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mfc_apq8064_ion_pdata,
		},
#ifndef CONFIG_MSM_IOMMU
		{
			.id	= ION_SF_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_SF_HEAP_NAME,
			.size	= MSM_ION_SF_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_apq8064_ion_pdata,
		},
#endif
		{
			.id	= ION_IOMMU_HEAP_ID,
			.type	= ION_HEAP_TYPE_IOMMU,
			.name	= ION_IOMMU_HEAP_NAME,
		},
		{
			.id	= ION_QSECOM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_QSECOM_HEAP_NAME,
			.size	= MSM_ION_QSECOM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_apq8064_ion_pdata,
		},
		{
			.id	= ION_AUDIO_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_HEAP_NAME,
			.size	= MSM_ION_AUDIO_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_apq8064_ion_pdata,
		},
		{
			.id     = ION_ADSP_HEAP_ID,
			.type   = ION_HEAP_TYPE_DMA,
			.name   = ION_ADSP_HEAP_NAME,
			.size   = MSM_ION_ADSP_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_apq8064_ion_pdata,
			.priv = &ion_adsp_heap_device.dev,
		},
#endif
};

static struct ion_platform_data apq8064_ion_pdata = {
	.nr = MSM_ION_HEAP_NUM,
	.heaps = apq8064_heaps,
};

static struct platform_device apq8064_ion_dev = {
	.name = "ion-msm",
	.id = 1,
	.dev = { .platform_data = &apq8064_ion_pdata },
};
#endif

static struct platform_device apq8064_fmem_device = {
	.name = "fmem",
	.id = 1,
	.dev = { .platform_data = &apq8064_fmem_pdata },
};

static void __init reserve_mem_for_ion(enum ion_memory_types mem_type,
				      unsigned long size)
{
	apq8064_reserve_table[mem_type].size += size;
}

static void __init apq8064_reserve_fixed_area(unsigned long fixed_area_size)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	int ret;

	if (fixed_area_size > MAX_FIXED_AREA_SIZE)
		panic("fixed area size is larger than %dM\n",
			MAX_FIXED_AREA_SIZE >> 20);

	reserve_info->fixed_area_size = fixed_area_size;
	reserve_info->fixed_area_start = APQ8064_FW_START;

	ret = memblock_remove(reserve_info->fixed_area_start,
		reserve_info->fixed_area_size);
	pr_info("mem_map: fixed_area reserved at 0x%lx with size 0x%lx\n",
			reserve_info->fixed_area_start,
			reserve_info->fixed_area_size);
	BUG_ON(ret);
#endif
}

/**
 * Reserve memory for ION and calculate amount of reusable memory for fmem.
 * We only reserve memory for heaps that are not reusable. However, we only
 * support one reusable heap at the moment so we ignore the reusable flag for
 * other than the first heap with reusable flag set. Also handle special case
 * for video heaps (MM,FW, and MFC). Video requires heaps MM and MFC to be
 * at a higher address than FW in addition to not more than 256MB away from the
 * base address of the firmware. This means that if MM is reusable the other
 * two heaps must be allocated in the same region as FW. This is handled by the
 * mem_is_fmem flag in the platform data. In addition the MM heap must be
 * adjacent to the FW heap for content protection purposes.
 */
static void __init reserve_ion_memory(void)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	unsigned int i;
	unsigned int ret;
	unsigned int fixed_size = 0;
	unsigned int fixed_low_size, fixed_middle_size, fixed_high_size;
	unsigned long fixed_low_start, fixed_middle_start, fixed_high_start;
	unsigned long cma_alignment;
	unsigned int low_use_cma = 0;
	unsigned int middle_use_cma = 0;
	unsigned int high_use_cma = 0;


	fixed_low_size = 0;
	fixed_middle_size = 0;
	fixed_high_size = 0;

	cma_alignment = PAGE_SIZE << max(MAX_ORDER, pageblock_order);

	for (i = 0; i < apq8064_ion_pdata.nr; ++i) {
		struct ion_platform_heap *heap =
			&(apq8064_ion_pdata.heaps[i]);
		int use_cma = 0;


		if (heap->extra_data) {
			int fixed_position = NOT_FIXED;

			switch ((int)heap->type) {
			case ION_HEAP_TYPE_CP:
				if (((struct ion_cp_heap_pdata *)
					heap->extra_data)->is_cma) {
					heap->size = ALIGN(heap->size,
						cma_alignment);
					use_cma = 1;
				}
				fixed_position = ((struct ion_cp_heap_pdata *)
					heap->extra_data)->fixed_position;
				break;
			case ION_HEAP_TYPE_DMA:
				use_cma = 1;
				/* Purposely fall through here */
			case ION_HEAP_TYPE_CARVEOUT:
				fixed_position = ((struct ion_co_heap_pdata *)
					heap->extra_data)->fixed_position;
				break;
			default:
				break;
			}

			if (fixed_position != NOT_FIXED)
				fixed_size += heap->size;
			else if (!use_cma)
				reserve_mem_for_ion(MEMTYPE_EBI1, heap->size);

			if (fixed_position == FIXED_LOW) {
				fixed_low_size += heap->size;
				low_use_cma = use_cma;
			} else if (fixed_position == FIXED_MIDDLE) {
				fixed_middle_size += heap->size;
				middle_use_cma = use_cma;
			} else if (fixed_position == FIXED_HIGH) {
				fixed_high_size += heap->size;
				high_use_cma = use_cma;
			} else if (use_cma) {
				/*
				 * Heaps that use CMA but are not part of the
				 * fixed set. Create wherever.
				 */
				dma_declare_contiguous(
					heap->priv,
					heap->size,
					0,
					0xb0000000);

			}
		}
	}

	if (!fixed_size)
		return;

	/*
	 * Given the setup for the fixed area, we can't round up all sizes.
	 * Some sizes must be set up exactly and aligned correctly. Incorrect
	 * alignments are considered a configuration issue
	 */

	fixed_low_start = APQ8064_FIXED_AREA_START;
	if (low_use_cma) {
		BUG_ON(!IS_ALIGNED(fixed_low_size + HOLE_SIZE, cma_alignment));
		BUG_ON(!IS_ALIGNED(fixed_low_start, cma_alignment));
	} else {
		BUG_ON(!IS_ALIGNED(fixed_low_size + HOLE_SIZE, SECTION_SIZE));
		ret = memblock_remove(fixed_low_start,
				      fixed_low_size + HOLE_SIZE);
		pr_info("mem_map: fixed_low_area reserved at 0x%lx with size \
				0x%x\n", fixed_low_start,
				fixed_low_size + HOLE_SIZE);
		BUG_ON(ret);
	}

	fixed_middle_start = fixed_low_start + fixed_low_size + HOLE_SIZE;
	if (middle_use_cma) {
		BUG_ON(!IS_ALIGNED(fixed_middle_start, cma_alignment));
		BUG_ON(!IS_ALIGNED(fixed_middle_size, cma_alignment));
	} else {
		BUG_ON(!IS_ALIGNED(fixed_middle_size, SECTION_SIZE));
		ret = memblock_remove(fixed_middle_start, fixed_middle_size);
		pr_info("mem_map: fixed_middle_area reserved at 0x%lx with \
				size 0x%x\n", fixed_middle_start,
				fixed_middle_size);
		BUG_ON(ret);
	}

	fixed_high_start = fixed_middle_start + fixed_middle_size;
	if (high_use_cma) {
		fixed_high_size = ALIGN(fixed_high_size, cma_alignment);
		BUG_ON(!IS_ALIGNED(fixed_high_start, cma_alignment));
	} else {
		/* This is the end of the fixed area so it's okay to round up */
		fixed_high_size = ALIGN(fixed_high_size, SECTION_SIZE);
		ret = memblock_remove(fixed_high_start, fixed_high_size);
		pr_info("mem_map: fixed_high_area reserved at 0x%lx with size \
				0x%x\n", fixed_high_start,
				fixed_high_size);
		BUG_ON(ret);
	}

	for (i = 0; i < apq8064_ion_pdata.nr; ++i) {
		struct ion_platform_heap *heap = &(apq8064_ion_pdata.heaps[i]);

		if (heap->extra_data) {
			int fixed_position = NOT_FIXED;
			struct ion_cp_heap_pdata *pdata = NULL;

			switch ((int) heap->type) {
			case ION_HEAP_TYPE_CP:
				pdata =
				(struct ion_cp_heap_pdata *)heap->extra_data;
				fixed_position = pdata->fixed_position;
				break;
			case ION_HEAP_TYPE_CARVEOUT:
			case ION_HEAP_TYPE_DMA:
				fixed_position = ((struct ion_co_heap_pdata *)
					heap->extra_data)->fixed_position;
				break;
			default:
				break;
			}

			switch (fixed_position) {
			case FIXED_LOW:
				heap->base = fixed_low_start;
				break;
			case FIXED_MIDDLE:
				heap->base = fixed_middle_start;
				if (middle_use_cma) {
					ret = dma_declare_contiguous(
						heap->priv,
						heap->size,
						fixed_middle_start,
						0xa0000000);
					WARN_ON(ret);
				}
				pdata->secure_base = fixed_middle_start
								- HOLE_SIZE;
				pdata->secure_size = HOLE_SIZE + heap->size;
				break;
			case FIXED_HIGH:
				heap->base = fixed_high_start;
				break;
			default:
				break;
			}
		}
	}
#endif
}

static void __init reserve_mdp_memory(void)
{
	apq8064_mdp_writeback(apq8064_reserve_table);
}

static void __init reserve_cache_dump_memory(void)
{
#ifdef CONFIG_MSM_CACHE_DUMP
	unsigned int total;

	total = apq8064_cache_dump_pdata.l1_size +
		apq8064_cache_dump_pdata.l2_size;
	apq8064_reserve_table[MEMTYPE_EBI1].size += total;
	pr_info("mem_map: cache_dump reserved with size 0x%x in pool\n",
			total);
#endif
}

static void __init reserve_mpdcvs_memory(void)
{
	apq8064_reserve_table[MEMTYPE_EBI1].size += SZ_32K;
}

static void __init apq8064_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
	reserve_ion_memory();
	reserve_mdp_memory();
	reserve_rtb_memory();
	reserve_cache_dump_memory();
	reserve_mpdcvs_memory();
}

static struct reserve_info apq8064_reserve_info __initdata = {
	.memtype_reserve_table = apq8064_reserve_table,
	.calculate_reserve_sizes = apq8064_calculate_reserve_sizes,
	.reserve_fixed_area = apq8064_reserve_fixed_area,
	.paddr_to_memtype = apq8064_paddr_to_memtype,
};

static char prim_panel_name[PANEL_NAME_MAX_LEN];
static char ext_panel_name[PANEL_NAME_MAX_LEN];

static int ext_resolution;

static int __init prim_display_setup(char *param)
{
	if (strnlen(param, PANEL_NAME_MAX_LEN))
		strlcpy(prim_panel_name, param, PANEL_NAME_MAX_LEN);
	return 0;
}
early_param("prim_display", prim_display_setup);

static int __init ext_display_setup(char *param)
{
	if (strnlen(param, PANEL_NAME_MAX_LEN))
		strlcpy(ext_panel_name, param, PANEL_NAME_MAX_LEN);
	return 0;
}
early_param("ext_display", ext_display_setup);

static int __init hdmi_resulution_setup(char *param)
{
	int ret;
	ret = kstrtoint(param, 10, &ext_resolution);
	return ret;
}
early_param("ext_resolution", hdmi_resulution_setup);

static void __init apq8064_reserve(void)
{
	msm_reserve();
	vega_reserve();
}

static void __init apq8064_early_reserve(void)
{
	reserve_info = &apq8064_reserve_info;
}
#ifdef CONFIG_USB_EHCI_MSM_HSIC
/* Bandwidth requests (zero) if no vote placed */
static struct msm_bus_vectors hsic_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ab = 0,
		.ib = 0,
	},
};

/* Bus bandwidth requests in Bytes/sec */
static struct msm_bus_vectors hsic_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ab = 0,
		.ib = 256000000, /*vote for 32Mhz dfab clk rate*/
	},
};

static struct msm_bus_paths hsic_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(hsic_init_vectors),
		hsic_init_vectors,
	},
	{
		ARRAY_SIZE(hsic_max_vectors),
		hsic_max_vectors,
	},
};

static struct msm_bus_scale_pdata hsic_bus_scale_pdata = {
	hsic_bus_scale_usecases,
	ARRAY_SIZE(hsic_bus_scale_usecases),
	.name = "hsic",
};

static struct msm_hsic_host_platform_data msm_hsic_pdata = {
	.strobe			= 88,
	.data			= 89,
	.bus_scale_table	= &hsic_bus_scale_pdata,
};
#else
static struct msm_hsic_host_platform_data msm_hsic_pdata;
#endif

#define PID_MAGIC_ID		0x71432909
#define SERIAL_NUM_MAGIC_ID	0x61945374
#define SERIAL_NUMBER_LENGTH	127
#define DLOAD_USB_BASE_ADD	0x2A03F0C8

struct magic_num_struct {
	uint32_t pid;
	uint32_t serial_num;
};

struct dload_struct {
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
	uint16_t	reserved4;
	uint16_t	pid;
	char		serial_number[SERIAL_NUMBER_LENGTH];
	uint16_t	reserved5;
	struct magic_num_struct magic_struct;
};

static int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum)
{
	struct dload_struct __iomem *dload = 0;

	dload = ioremap(DLOAD_USB_BASE_ADD, sizeof(*dload));
	if (!dload) {
		pr_err("%s: cannot remap I/O memory region: %08x\n",
					__func__, DLOAD_USB_BASE_ADD);
		return -ENXIO;
	}

	pr_debug("%s: dload:%p pid:%x serial_num:%s\n",
				__func__, dload, pid, snum);
	/* update pid */
	dload->magic_struct.pid = PID_MAGIC_ID;
	dload->pid = pid;

	/* update serial number */
	dload->magic_struct.serial_num = 0;
	if (!snum) {
		memset(dload->serial_number, 0, SERIAL_NUMBER_LENGTH);
		goto out;
	}

	dload->magic_struct.serial_num = SERIAL_NUM_MAGIC_ID;
	strlcpy(dload->serial_number, snum, SERIAL_NUMBER_LENGTH);
out:
	iounmap(dload);
	return 0;
}

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

/* Bandwidth requests (zero) if no vote placed */
static struct msm_bus_vectors usb_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

/* Bus bandwidth requests in Bytes/sec */
static struct msm_bus_vectors usb_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 60000000,		/* At least 480Mbps on bus. */
		.ib = 960000000,	/* MAX bursts rate */
	},
};

static struct msm_bus_paths usb_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(usb_init_vectors),
		usb_init_vectors,
	},
	{
		ARRAY_SIZE(usb_max_vectors),
		usb_max_vectors,
	},
};

static struct msm_bus_scale_pdata usb_bus_scale_pdata = {
	usb_bus_scale_usecases,
	ARRAY_SIZE(usb_bus_scale_usecases),
	.name = "usb",
};

#ifdef CONFIG_PANTECH_USB_TUNE_SIGNALING_PARAM
static int phy_init_seq[] = {
	0x3f, 0x81, /* update DC voltage level */
	0x33, 0x82, /* set pre-emphasis and rise/fall time EF51 : 0x3f */
	0x33, 0x83, /* set impedance EF51 : 0x13 */
	-1
};
#else
static int phy_init_seq[] = {
	0x68, 0x81, /* update DC voltage level */
	0x24, 0x82, /* set pre-emphasis and rise/fall time */
	-1
};
#endif

#define PMIC_GPIO_DP		27    /* PMIC GPIO for D+ change */
#define PMIC_GPIO_DP_IRQ	PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PMIC_GPIO_DP)
#define MSM_MPM_PIN_USB1_OTGSESSVLD	40

static int msm_otg_control_usb_switch(int pm_gpio, int value)
{
	static int gpio;
	int rc;

	gpio = PM8921_GPIO_PM_TO_SYS(pm_gpio);

	printk(KERN_INFO "%s: pm_gpio[%d], value[%d], gpio[%d]\n", 
			__func__, pm_gpio, value, gpio);

	if(pm_gpio == 36){
		rc = gpio_request(gpio, "msm_usb_id_sw");
		if(rc){
			pr_err("request gpio 36 failed, rc=%d\n", rc);
			return -ENODEV;
		}
	}else if(pm_gpio == 44){
		rc = gpio_request(gpio, "pmic_usb_id_sw");
		if(rc){
			pr_err("request gpio  failed, rc=%d\n", rc);
			return -ENODEV;
		}
	}

	if(value){
		gpio_set_value_cansleep(gpio, 1);
	}else{
		gpio_set_value_cansleep(gpio, 0);
	}

	gpio_free(gpio);
	return 0;
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.mode = USB_OTG,
	.otg_control = OTG_PHY_CONTROL,
	.phy_type = SNPS_28NM_INTEGRATED_PHY,
	.pmic_id_irq = PM8921_GPIO_IRQ(PM8921_IRQ_BASE, 25),
	.control_usb_switch = msm_otg_control_usb_switch,
	.power_budget = 750,
        .bus_scale_table = &usb_bus_scale_pdata,
	.phy_init_seq = phy_init_seq,
	.mpm_otgsessvld_int = MSM_MPM_PIN_USB1_OTGSESSVLD,
};

static struct msm_usb_host_platform_data msm_ehci_host_pdata3 = {
	.power_budget = 500,
};

#ifdef CONFIG_USB_EHCI_MSM_HOST4
static struct msm_usb_host_platform_data msm_ehci_host_pdata4;
#endif

static void __init apq8064_ehci_host_init(void)
{
	if (machine_is_apq8064_liquid() || machine_is_mpq8064_cdp() ||
		machine_is_mpq8064_hrd() || machine_is_mpq8064_dtv()) {
		if (machine_is_apq8064_liquid())
			msm_ehci_host_pdata3.dock_connect_irq =
					PM8921_MPP_IRQ(PM8921_IRQ_BASE, 9);
		else
			msm_ehci_host_pdata3.pmic_gpio_dp_irq =
							PMIC_GPIO_DP_IRQ;

		apq8064_device_ehci_host3.dev.platform_data =
				&msm_ehci_host_pdata3;
		platform_device_register(&apq8064_device_ehci_host3);

#ifdef CONFIG_USB_EHCI_MSM_HOST4
		apq8064_device_ehci_host4.dev.platform_data =
				&msm_ehci_host_pdata4;
		platform_device_register(&apq8064_device_ehci_host4);
#endif
	}
}

struct sx150x_platform_data apq8064_sx150x_data[] = {
	[SX150X_EPM] = {
		.gpio_base	= GPIO_EPM_EXPANDER_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0,
		.io_pulldn_ena	= 0x0,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= -1,
	},
};

static struct epm_chan_properties ads_adc_channel_data[] = {
	{10, 100}, {1000, 1}, {10, 100}, {1000, 1},
	{10, 100}, {1000, 1}, {10, 100}, {1000, 1},
	{10, 100}, {20, 100}, {500, 100}, {5, 100},
	{1000, 1}, {200, 100}, {50, 100}, {10, 100},
	{510, 100}, {50, 100}, {20, 100}, {100, 100},
	{510, 100}, {20, 100}, {50, 100}, {200, 100},
	{10, 100}, {20, 100}, {1000, 1}, {10, 100},
	{200, 100}, {510, 100}, {1000, 100}, {200, 100},
};

static struct epm_adc_platform_data epm_adc_pdata = {
	.channel		= ads_adc_channel_data,
	.bus_id	= 0x0,
	.epm_i2c_board_info = {
		.type	= "sx1509q",
		.addr = 0x3e,
		.platform_data = &apq8064_sx150x_data[SX150X_EPM],
	},
	.gpio_expander_base_addr = GPIO_EPM_EXPANDER_BASE,
};

static struct platform_device epm_adc_device = {
	.name   = "epm_adc",
	.id = -1,
	.dev = {
		.platform_data = &epm_adc_pdata,
	},
};

static void __init apq8064_epm_adc_init(void)
{
	epm_adc_pdata.num_channels = 32;
	epm_adc_pdata.num_adc = 2;
	epm_adc_pdata.chan_per_adc = 16;
	epm_adc_pdata.chan_per_mux = 8;
};

/* Micbias setting is based on 8660 CDP/MTP/FLUID requirement
 * 4 micbiases are used to power various analog and digital
 * microphones operating at 1800 mV. Technically, all micbiases
 * can source from single cfilter since all microphones operate
 * at the same voltage level. The arrangement below is to make
 * sure all cfilters are exercised. LDO_H regulator ouput level
 * does not need to be as high as 2.85V. It is choosen for
 * microphone sensitivity purpose.
 */
static struct wcd9xxx_pdata apq8064_tabla_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x10, 0, 0x17, 2},
	},
	.irq = MSM_GPIO_TO_INT(42),
	.irq_base = TABLA_INTERRUPT_BASE,
	.num_irqs = NR_WCD9XXX_IRQS,
	.reset_gpio = PM8921_GPIO_PM_TO_SYS(34),
	.micbias = {
		.ldoh_v = TABLA_LDOH_2P85_V,
		.cfilt1_mv = 1800,
		.cfilt2_mv = 2700,
		.cfilt3_mv = 1800,
		.bias1_cfilt_sel = TABLA_CFILT1_SEL,
		.bias2_cfilt_sel = TABLA_CFILT2_SEL,
		.bias3_cfilt_sel = TABLA_CFILT3_SEL,
		.bias4_cfilt_sel = TABLA_CFILT3_SEL,
	},
	.regulator = {
	{
		.name = "CDC_VDD_CP",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_CP_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_RX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_RX_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_TX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_TX_CUR_MAX,
	},
	{
		.name = "VDDIO_CDC",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_VDDIO_CDC_CUR_MAX,
	},
	{
		.name = "VDDD_CDC_D",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_D_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_A_1P2V",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_A_CUR_MAX,
	},
	},
};

static struct slim_device apq8064_slim_tabla = {
	.name = "tabla-slim",
	.e_addr = {0, 1, 0x10, 0, 0x17, 2},
	.dev = {
		.platform_data = &apq8064_tabla_platform_data,
	},
};

static struct wcd9xxx_pdata apq8064_tabla20_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x60, 0, 0x17, 2},
	},
	.irq = MSM_GPIO_TO_INT(42),
	.irq_base = TABLA_INTERRUPT_BASE,
	.num_irqs = NR_WCD9XXX_IRQS,
	.reset_gpio = PM8921_GPIO_PM_TO_SYS(34),
	.micbias = {
		.ldoh_v = TABLA_LDOH_2P85_V,
		.cfilt1_mv = 1800,
		.cfilt2_mv = 2700,
		.cfilt3_mv = 1800,
		.bias1_cfilt_sel = TABLA_CFILT1_SEL,
		.bias2_cfilt_sel = TABLA_CFILT2_SEL,
		.bias3_cfilt_sel = TABLA_CFILT3_SEL,
		.bias4_cfilt_sel = TABLA_CFILT3_SEL,
	},
	.regulator = {
	{
		.name = "CDC_VDD_CP",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_CP_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_RX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_RX_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_TX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_TX_CUR_MAX,
	},
	{
		.name = "VDDIO_CDC",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_VDDIO_CDC_CUR_MAX,
	},
	{
		.name = "VDDD_CDC_D",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_D_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_A_1P2V",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_A_CUR_MAX,
	},
	},
};

static struct slim_device apq8064_slim_tabla20 = {
	.name = "tabla2x-slim",
	.e_addr = {0, 1, 0x60, 0, 0x17, 2},
	.dev = {
		.platform_data = &apq8064_tabla20_platform_data,
	},
};

/* enable the level shifter for cs8427 to make sure the I2C
 * clock is running at 100KHz and voltage levels are at 3.3
 * and 5 volts
 */
static int enable_100KHz_ls(int enable)
{
	int ret = 0;
	if (enable) {
		ret = gpio_request(SX150X_GPIO(1, 10),
					"cs8427_100KHZ_ENABLE");
		if (ret) {
			pr_err("%s: Failed to request gpio %d\n", __func__,
				SX150X_GPIO(1, 10));
			return ret;
		}
		gpio_direction_output(SX150X_GPIO(1, 10), 1);
	} else {
		gpio_direction_output(SX150X_GPIO(1, 10), 0);
		gpio_free(SX150X_GPIO(1, 10));
	}
	return ret;
}

static struct cs8427_platform_data cs8427_i2c_platform_data = {
	.irq = SX150X_GPIO(1, 4),
	.reset_gpio = SX150X_GPIO(1, 6),
	.enable = enable_100KHz_ls,
};

static struct i2c_board_info cs8427_device_info[] __initdata = {
	{
		I2C_BOARD_INFO("cs8427", CS8427_ADDR4),
		.platform_data = &cs8427_i2c_platform_data,
	},
};

#define MSM_WCNSS_PHYS	0x03000000
#define MSM_WCNSS_SIZE	0x280000

static struct resource resources_wcnss_wlan[] = {
	{
		.start	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.end	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.name	= "wcnss_wlanrx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.end	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.name	= "wcnss_wlantx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_WCNSS_PHYS,
		.end	= MSM_WCNSS_PHYS + MSM_WCNSS_SIZE - 1,
		.name	= "wcnss_mmio",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 64,
		.end	= 68,
		.name	= "wcnss_gpios_5wire",
		.flags	= IORESOURCE_IO,
	},
};

static struct qcom_wcnss_opts qcom_wcnss_pdata = {
	.has_48mhz_xo	= 1,
};

static struct platform_device msm_device_wcnss_wlan = {
	.name		= "wcnss_wlan",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_wcnss_wlan),
	.resource	= resources_wcnss_wlan,
	.dev		= {.platform_data = &qcom_wcnss_pdata},
};

#ifdef CONFIG_RADIO_IRIS
static struct platform_device msm_device_iris_fm __devinitdata = {
	.name = "iris_fm",
	.id   = -1,
};
#endif

#ifdef CONFIG_QSEECOM
/* qseecom bus scaling */
static struct msm_bus_vectors qseecom_clks_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = 0,
		.ab = 0,
	},
};

static struct msm_bus_vectors qseecom_enable_dfab_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 70000000UL,
		.ib = 70000000UL,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 2480000000UL,
		.ib = 2480000000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = 0,
		.ab = 0,
	},
};

static struct msm_bus_vectors qseecom_enable_sfpb_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = (64 * 8) * 1000000UL,
		.ab = (64 * 8) *  100000UL,
	},
};

static struct msm_bus_vectors qseecom_enable_dfab_sfpb_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 70000000UL,
		.ib = 70000000UL,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 2480000000UL,
		.ib = 2480000000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = (64 * 8) * 1000000UL,
		.ab = (64 * 8) *  100000UL,
	},
};

static struct msm_bus_paths qseecom_hw_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(qseecom_clks_init_vectors),
		qseecom_clks_init_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_dfab_vectors),
		qseecom_enable_dfab_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_sfpb_vectors),
		qseecom_enable_sfpb_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_dfab_sfpb_vectors),
		qseecom_enable_dfab_sfpb_vectors,
	},
};

static struct msm_bus_scale_pdata qseecom_bus_pdata = {
	qseecom_hw_bus_scale_usecases,
	ARRAY_SIZE(qseecom_hw_bus_scale_usecases),
	.name = "qsee",
};

static struct platform_device qseecom_device = {
	.name		= "qseecom",
	.id		= 0,
	.dev		= {
		.platform_data = &qseecom_bus_pdata,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0x11000000

#define QCE_HW_KEY_SUPPORT	0
#define QCE_SHA_HMAC_SUPPORT	1
#define QCE_SHARE_CE_RESOURCE	3
#define QCE_CE_SHARED		0

static struct resource qcrypto_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV8064_CE_IN_CHAN,
		.end = DMOV8064_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV8064_CE_IN_CRCI,
		.end = DMOV8064_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV8064_CE_OUT_CRCI,
		.end = DMOV8064_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

static struct resource qcedev_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV8064_CE_IN_CHAN,
		.end = DMOV8064_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV8064_CE_IN_CRCI,
		.end = DMOV8064_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV8064_CE_OUT_CRCI,
		.end = DMOV8064_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	.bus_scale_table = NULL,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcrypto_resources),
	.resource	= qcrypto_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	.bus_scale_table = NULL,
};

static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcedev_resources),
	.resource	= qcedev_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

static struct mdm_vddmin_resource mdm_vddmin_rscs = {
	.rpm_id = MSM_RPM_ID_VDDMIN_GPIO,
	.ap2mdm_vddmin_gpio = 30,
	.modes  = 0x03,
	.drive_strength = 8,
	.mdm2ap_vddmin_gpio = 80,
};

static struct gpiomux_setting mdm2ap_status_gpio_run_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct mdm_platform_data mdm_platform_data = {
	.mdm_version = "3.0",
	.ramdump_delay_ms = 2000,
	.early_power_on = 1,
	.sfr_query = 1,
	.send_shdn = 1,
	.vddmin_resource = &mdm_vddmin_rscs,
	.peripheral_platform_device = &apq8064_device_hsic_host,
	.ramdump_timeout_ms = 120000,
	.mdm2ap_status_gpio_run_cfg = &mdm2ap_status_gpio_run_cfg,
	.sysmon_subsys_id_valid = 1,
	.sysmon_subsys_id = SYSMON_SS_EXT_MODEM,
};

static struct mdm_platform_data amdm_platform_data = {
	.mdm_version = "3.0",
	.ramdump_delay_ms = 2000,
	.early_power_on = 1,
	.sfr_query = 1,
	.send_shdn = 1,
	.vddmin_resource = &mdm_vddmin_rscs,
	.peripheral_platform_device = &apq8064_device_hsic_host,
	.ramdump_timeout_ms = 120000,
	.mdm2ap_status_gpio_run_cfg = &mdm2ap_status_gpio_run_cfg,
	.sysmon_subsys_id_valid = 1,
	.sysmon_subsys_id = SYSMON_SS_EXT_MODEM,
	.no_a2m_errfatal_on_ssr = 1,
};

static struct mdm_vddmin_resource bmdm_vddmin_rscs = {
	.rpm_id = MSM_RPM_ID_VDDMIN_GPIO,
	.ap2mdm_vddmin_gpio = 30,
	.modes  = 0x03,
	.drive_strength = 8,
	.mdm2ap_vddmin_gpio = 64,
};

static struct mdm_platform_data bmdm_platform_data = {
	.mdm_version = "3.0",
	.ramdump_delay_ms = 2000,
	.sfr_query = 1,
	.send_shdn = 1,
	.vddmin_resource = &bmdm_vddmin_rscs,
	.peripheral_platform_device = &apq8064_device_ehci_host3,
	.ramdump_timeout_ms = 120000,
	.mdm2ap_status_gpio_run_cfg = &mdm2ap_status_gpio_run_cfg,
	.sysmon_subsys_id_valid = 1,
	.sysmon_subsys_id = SYSMON_SS_EXT_MODEM2,
	.no_a2m_errfatal_on_ssr = 1,
};

static struct tsens_platform_data apq_tsens_pdata  = {
		.tsens_factor		= 1000,
		.hw_type		= APQ_8064,
		.tsens_num_sensor	= 11,
		.slope = {1176, 1176, 1154, 1176, 1111,
			1132, 1132, 1199, 1132, 1199, 1132},
};

static struct platform_device msm_tsens_device = {
	.name   = "tsens8960-tm",
	.id = -1,
};

static struct msm_thermal_data msm_thermal_pdata = {
	.sensor_id = 7,
	.poll_ms = 250,
	.limit_temp_degC = 60,
	.temp_hysteresis_degC = 10,
	.freq_step = 2,
	.core_limit_temp_degC = 80,
	.core_temp_hysteresis_degC = 10,
	.core_control_mask = 0xe,
};

#define MSM_SHARED_RAM_PHYS 0x80000000
static void __init apq8064_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_apq8064_io();
	if (socinfo_init() < 0)
		pr_err("socinfo_init() failed!\n");
}

static void __init apq8064_init_irq(void)
{
	struct msm_mpm_device_data *data = NULL;

#ifdef CONFIG_MSM_MPM
	data = &apq8064_mpm_dev_data;
#endif

	msm_mpm_irq_extn_init(data);
	gic_init(0, GIC_PPI_START, MSM_QGIC_DIST_BASE,
						(void *)MSM_QGIC_CPU_BASE);
}

static struct platform_device msm8064_device_saw_regulator_core0 = {
	.name	= "saw-regulator",
	.id	= 0,
	.dev	= {
		.platform_data = &msm8064_saw_regulator_pdata_8921_s5,
	},
};

static struct platform_device msm8064_device_saw_regulator_core1 = {
	.name	= "saw-regulator",
	.id	= 1,
	.dev	= {
		.platform_data = &msm8064_saw_regulator_pdata_8921_s6,
	},
};

static struct platform_device msm8064_device_saw_regulator_core2 = {
	.name	= "saw-regulator",
	.id	= 2,
	.dev	= {
		.platform_data = &msm8064_saw_regulator_pdata_8821_s0,
	},
};

static struct platform_device msm8064_device_saw_regulator_core3 = {
	.name	= "saw-regulator",
	.id	= 3,
	.dev	= {
		.platform_data = &msm8064_saw_regulator_pdata_8821_s1,

	},
};

static struct msm_rpmrs_level msm_rpmrs_levels[] = {
	{
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		1, 784, 180000, 100,
	},

	{
		MSM_PM_SLEEP_MODE_RETENTION,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		415, 715, 340827, 475,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		1300, 228, 1200000, 2000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, GDHS, MAX, ACTIVE),
		false,
		2000, 138, 1208400, 3200,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		6000, 119, 1850300, 9000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, GDHS, MAX, ACTIVE),
		false,
		9200, 68, 2839200, 16400,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, MAX, ACTIVE),
		false,
		10300, 63, 3128000, 18200,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		18000, 10, 4602600, 27000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, RET_HIGH, RET_LOW),
		false,
		20000, 2, 5752000, 32000,
	},
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_TZ,
};

static struct msm_rpmrs_platform_data msm_rpmrs_data __initdata = {
	.levels = &msm_rpmrs_levels[0],
	.num_levels = ARRAY_SIZE(msm_rpmrs_levels),
	.vdd_mem_levels  = {
		[MSM_RPMRS_VDD_MEM_RET_LOW]	= 750000,
		[MSM_RPMRS_VDD_MEM_RET_HIGH]	= 750000,
		[MSM_RPMRS_VDD_MEM_ACTIVE]	= 1050000,
		[MSM_RPMRS_VDD_MEM_MAX]		= 1150000,
	},
	.vdd_dig_levels = {
		[MSM_RPMRS_VDD_DIG_RET_LOW]	= 500000,
		[MSM_RPMRS_VDD_DIG_RET_HIGH]	= 750000,
		[MSM_RPMRS_VDD_DIG_ACTIVE]	= 950000,
		[MSM_RPMRS_VDD_DIG_MAX]		= 1150000,
	},
	.vdd_mask = 0x7FFFFF,
	.rpmrs_target_id = {
		[MSM_RPMRS_ID_PXO_CLK]		= MSM_RPM_ID_PXO_CLK,
		[MSM_RPMRS_ID_L2_CACHE_CTL]	= MSM_RPM_ID_LAST,
		[MSM_RPMRS_ID_VDD_DIG_0]	= MSM_RPM_ID_PM8921_S3_0,
		[MSM_RPMRS_ID_VDD_DIG_1]	= MSM_RPM_ID_PM8921_S3_1,
		[MSM_RPMRS_ID_VDD_MEM_0]	= MSM_RPM_ID_PM8921_L24_0,
		[MSM_RPMRS_ID_VDD_MEM_1]	= MSM_RPM_ID_PM8921_L24_1,
		[MSM_RPMRS_ID_RPM_CTL]		= MSM_RPM_ID_RPM_CTL,
	},
};

static uint8_t spm_wfi_cmd_sequence[] __initdata = {
	0x03, 0x0f,
};

static uint8_t spm_power_collapse_without_rpm[] __initdata = {
	0x00, 0x24, 0x54, 0x10,
	0x09, 0x03, 0x01,
	0x10, 0x54, 0x30, 0x0C,
	0x24, 0x30, 0x0f,
};

static uint8_t spm_retention_cmd_sequence[] __initdata = {
	0x00, 0x05, 0x03, 0x0D,
	0x0B, 0x00, 0x0f,
};



static uint8_t spm_power_collapse_with_rpm[] __initdata = {
	0x00, 0x24, 0x54, 0x10,
	0x09, 0x07, 0x01, 0x0B,
	0x10, 0x54, 0x30, 0x0C,
	0x24, 0x30, 0x0f,
};

/* 8064AB has a different command to assert apc_pdn */
static uint8_t spm_power_collapse_without_rpm_krait_v3[] __initdata = {
	0x00, 0x24, 0x84, 0x10,
	0x09, 0x03, 0x01,
	0x10, 0x84, 0x30, 0x0C,
	0x24, 0x30, 0x0f,
};

static uint8_t spm_power_collapse_with_rpm_krait_v3[] __initdata = {
	0x00, 0x24, 0x84, 0x10,
	0x09, 0x07, 0x01, 0x0B,
	0x10, 0x84, 0x30, 0x0C,
	0x24, 0x30, 0x0f,
};

static struct msm_spm_seq_entry msm_spm_boot_cpu_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_MODE_CLOCK_GATING,
		.notify_rpm = false,
		.cmd = spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_MODE_POWER_RETENTION,
		.notify_rpm = false,
		.cmd = spm_retention_cmd_sequence,
	},
	[2] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = false,
		.cmd = spm_power_collapse_without_rpm,
	},
	[3] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = spm_power_collapse_with_rpm,
	},
};
static struct msm_spm_seq_entry msm_spm_nonboot_cpu_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_MODE_CLOCK_GATING,
		.notify_rpm = false,
		.cmd = spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_MODE_POWER_RETENTION,
		.notify_rpm = false,
		.cmd = spm_retention_cmd_sequence,
	},
	[2] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = false,
		.cmd = spm_power_collapse_without_rpm,
	},
	[3] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = spm_power_collapse_with_rpm,
	},
};

static uint8_t l2_spm_wfi_cmd_sequence[] __initdata = {
	0x00, 0x20, 0x03, 0x20,
	0x00, 0x0f,
};

static uint8_t l2_spm_gdhs_cmd_sequence[] __initdata = {
	0x00, 0x20, 0x34, 0x64,
	0x48, 0x07, 0x48, 0x20,
	0x50, 0x64, 0x04, 0x34,
	0x50, 0x0f,
};
static uint8_t l2_spm_power_off_cmd_sequence[] __initdata = {
	0x00, 0x10, 0x34, 0x64,
	0x48, 0x07, 0x48, 0x10,
	0x50, 0x64, 0x04, 0x34,
	0x50, 0x0F,
};

static struct msm_spm_seq_entry msm_spm_l2_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_L2_MODE_RETENTION,
		.notify_rpm = false,
		.cmd = l2_spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_L2_MODE_GDHS,
		.notify_rpm = true,
		.cmd = l2_spm_gdhs_cmd_sequence,
	},
	[2] = {
		.mode = MSM_SPM_L2_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = l2_spm_power_off_cmd_sequence,
	},
};


static struct msm_spm_platform_data msm_spm_l2_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW_L2_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x00A000AE,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A00020,
		.modes = msm_spm_l2_seq_list,
		.num_modes = ARRAY_SIZE(msm_spm_l2_seq_list),
	},
};

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x03020004,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0084009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A4001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_boot_cpu_seq_list),
		.modes = msm_spm_boot_cpu_seq_list,
	},
	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x03020004,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0084009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A4001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_nonboot_cpu_seq_list),
		.modes = msm_spm_nonboot_cpu_seq_list,
	},
	[2] = {
		.reg_base_addr = MSM_SAW2_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x03020004,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0084009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A4001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_nonboot_cpu_seq_list),
		.modes = msm_spm_nonboot_cpu_seq_list,
	},
	[3] = {
		.reg_base_addr = MSM_SAW3_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x03020004,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0084009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A4001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_nonboot_cpu_seq_list),
		.modes = msm_spm_nonboot_cpu_seq_list,
	},
};

static void __init apq8064ab_update_krait_spm(void)
{
	int i;

	/* Update the SPM sequences for SPC and PC */
	for (i = 0; i < ARRAY_SIZE(msm_spm_data); i++) {
		int j;
		struct msm_spm_platform_data *pdata = &msm_spm_data[i];
		for (j = 0; j < pdata->num_modes; j++) {
			if (pdata->modes[j].cmd ==
					spm_power_collapse_without_rpm)
				pdata->modes[j].cmd =
				spm_power_collapse_without_rpm_krait_v3;
			else if (pdata->modes[j].cmd ==
					spm_power_collapse_with_rpm)
				pdata->modes[j].cmd =
				spm_power_collapse_with_rpm_krait_v3;
		}
	}
}

static void __init apq8064_init_buses(void)
{
	msm_bus_rpm_set_mt_mask();
	msm_bus_8064_apps_fabric_pdata.rpm_enabled = 1;
	msm_bus_8064_sys_fabric_pdata.rpm_enabled = 1;
	msm_bus_8064_mm_fabric_pdata.rpm_enabled = 1;
	msm_bus_8064_apps_fabric.dev.platform_data =
		&msm_bus_8064_apps_fabric_pdata;
	msm_bus_8064_sys_fabric.dev.platform_data =
		&msm_bus_8064_sys_fabric_pdata;
	msm_bus_8064_mm_fabric.dev.platform_data =
		&msm_bus_8064_mm_fabric_pdata;
	msm_bus_8064_sys_fpb.dev.platform_data = &msm_bus_8064_sys_fpb_pdata;
	msm_bus_8064_cpss_fpb.dev.platform_data = &msm_bus_8064_cpss_fpb_pdata;
}



static struct platform_device apq8064_device_ext_5v_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_MPP_PM_TO_SYS(7),
	.dev	= {
		.platform_data
			= &apq8064_gpio_regulator_pdata[GPIO_VREG_ID_EXT_5V],
	},
};

static struct platform_device apq8064_device_ext_mpp8_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_MPP_PM_TO_SYS(8),
	.dev	= {
		.platform_data
			= &apq8064_gpio_regulator_pdata[GPIO_VREG_ID_EXT_MPP8],
	},
};

/* GPIO I2C BATTERY_MAX17058 */
#define GPIO_FG_SCL      1
#define GPIO_FG_SDA      37

static struct i2c_gpio_platform_data i2c_gpio09_data = {
        .scl_pin = GPIO_FG_SCL,
        .sda_pin = GPIO_FG_SDA,
        .udelay = 5,    /* 100 KHz */
};

struct platform_device i2c_gpio09 = {
        .name = "i2c-gpio",
        .id = APQ_8064_GPIO09_I2C_BUS_ID,
        .dev = {
                .platform_data = &i2c_gpio09_data,
        }
};

/* GPIO I2C SMB347-PT  */
#define GPIO_SC_SCL      54
#define GPIO_SC_SDA     53
static struct i2c_gpio_platform_data i2c_gpio10_data = {
	.sda_pin	= GPIO_SC_SDA,
	.scl_pin	= GPIO_SC_SCL,
};

static struct platform_device i2c_gpio10 = {
	.name		= "i2c-gpio",
	.id		= APQ_8064_GPIO10_I2C_BUS_ID,
	.dev		= {
		.platform_data	= &i2c_gpio10_data,
	},
};

#define GPIO_PIN_SUBSYSTEM_SCL	70
#define GPIO_PIN_SUBSYSTEM_SDA	71

static struct i2c_gpio_platform_data i2c_gpio11_data = {
	.scl_pin = GPIO_PIN_SUBSYSTEM_SCL,
	.sda_pin = GPIO_PIN_SUBSYSTEM_SDA,
	.udelay = 5,	/* 100 KHz */
};

struct platform_device i2c_gpio11 = {
	.name = "i2c-gpio",
	.id = APQ_8064_GPIO11_I2C_BUS_ID,
	.dev = {
       .platform_data = &i2c_gpio11_data,
	}
};

static struct platform_device apq8064_device_rpm_regulator __devinitdata = {
	.name	= "rpm-regulator",
	.id	= 0,
	.dev	= {
		.platform_data = &apq8064_rpm_regulator_pdata,
	},
};

static struct platform_device
apq8064_pm8921_device_rpm_regulator __devinitdata = {
	.name	= "rpm-regulator",
	.id	= 1,
	.dev	= {
		.platform_data = &apq8064_rpm_regulator_pm8921_pdata,
	},
};

static struct platform_device *common_not_mpq_devices[] __initdata = {
	&apq8064_device_qup_i2c_gsbi1,
	&apq8064_device_qup_i2c_gsbi3,
	&apq8064_device_qup_i2c_gsbi4,
};

static struct platform_device *early_common_devices[] __initdata = {
	&apq8064_device_acpuclk,
	&apq8064_device_dmov,
	&apq8064_device_qup_spi_gsbi5,
};

static struct platform_device *pm8921_common_devices[] __initdata = {
	&apq8064_device_ext_5v_vreg,
	&apq8064_device_ext_mpp8_vreg,

	&apq8064_device_ssbi_pmic1,
	&apq8064_device_ssbi_pmic2,

};

static struct platform_device *pm8917_common_devices[] __initdata = {
	&apq8064_device_ext_mpp8_vreg,

	&apq8064_device_ssbi_pmic1,
	&apq8064_device_ssbi_pmic2,

};

static struct platform_device *common_devices[] __initdata = {
	&msm_device_smd_apq8064,
	&apq8064_device_otg,
	&apq8064_device_gadget_peripheral,
	&apq8064_device_hsusb_host,
	&android_usb_device,
	&msm_device_wcnss_wlan,
#ifdef CONFIG_RADIO_IRIS
	&msm_device_iris_fm,
#endif
	&apq8064_fmem_device,
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	&apq8064_android_pmem_device,
	&apq8064_android_pmem_adsp_device,
	&apq8064_android_pmem_audio_device,
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/
#ifdef CONFIG_ION_MSM
	&apq8064_ion_dev,
#endif
	&msm8064_device_watchdog,
	&msm8064_device_saw_regulator_core0,
	&msm8064_device_saw_regulator_core1,
	&msm8064_device_saw_regulator_core2,
	&msm8064_device_saw_regulator_core3,
#if defined(CONFIG_QSEECOM)
	&qseecom_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

#ifdef CONFIG_HW_RANDOM_MSM
	&apq8064_device_rng,
#endif
	&apq_pcm,
	&apq_pcm_routing,
	&apq_cpudai0,
	&apq_cpudai1,
	&mpq_cpudai_sec_i2s_rx,
	&mpq_cpudai_mi2s_tx,
	&apq_cpudai_hdmi_rx,
	&apq_cpudai_bt_rx,
	&apq_cpudai_bt_tx,
	&apq_cpudai_fm_rx,
	&apq_cpudai_fm_tx,
	&apq_cpu_fe,
	&apq_stub_codec,
	&apq_voice,
	&apq_voip,
	&apq_lpa_pcm,
	&apq_compr_dsp,
	&apq_multi_ch_pcm,
	&apq_lowlatency_pcm,
	&apq_pcm_hostless,
	&apq_cpudai_afe_01_rx,
	&apq_cpudai_afe_01_tx,
	&apq_cpudai_afe_02_rx,
	&apq_cpudai_afe_02_tx,
	&apq_pcm_afe,
	&apq_cpudai_auxpcm_rx,
	&apq_cpudai_auxpcm_tx,
	&apq_cpudai_stub,
	&apq_cpudai_slimbus_1_rx,
	&apq_cpudai_slimbus_1_tx,
	&apq_cpudai_slimbus_2_rx,
	&apq_cpudai_slimbus_2_tx,
	&apq_cpudai_slimbus_3_rx,
	&apq_cpudai_slimbus_3_tx,
	&apq8064_rpm_device,
	&apq8064_rpm_log_device,
	&apq8064_rpm_stat_device,
	&apq8064_rpm_master_stat_device,
	&apq_device_tz_log,
	&msm_bus_8064_apps_fabric,
	&msm_bus_8064_sys_fabric,
	&msm_bus_8064_mm_fabric,
	&msm_bus_8064_sys_fpb,
	&msm_bus_8064_cpss_fpb,
	&apq8064_msm_device_vidc,
	&msm_pil_dsps,
	&msm_8960_q6_lpass,
	&msm_pil_vidc,
	&msm_gss,
	&apq8064_rtb_device,
	&apq8064_dcvs_device,
	&apq8064_msm_gov_device,
	&apq8064_device_cache_erp,
	&msm8960_device_ebi1_ch0_erp,
	&msm8960_device_ebi1_ch1_erp,
	&epm_adc_device,
	&coresight_tpiu_device,
	&coresight_etb_device,
	&apq8064_coresight_funnel_device,
	&coresight_etm0_device,
	&coresight_etm1_device,
	&coresight_etm2_device,
	&coresight_etm3_device,
	&apq_cpudai_slim_4_rx,
	&apq_cpudai_slim_4_tx,
#ifdef CONFIG_MSM_GEMINI
	&msm8960_gemini_device,
#endif
	&apq8064_iommu_domain_device,
	&msm_tsens_device,
	&apq8064_cache_dump_device,

#ifdef CONFIG_BATTERY_BCL
	&battery_bcl_device,
#endif
	&apq8064_msm_mpd_device,

	&i2c_gpio09,
	&i2c_gpio10,
	&i2c_gpio11,

};

static struct platform_device *cdp_devices[] __initdata = {
	&apq8064_device_uart_gsbi5,
	&msm_device_sps_apq8064,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm8064_pc_cntr,
};

static struct msm_spi_platform_data apq8064_qup_spi_gsbi5_pdata = {
	.max_clock_speed = 1100000,
};

static struct spi_board_info spi_board_info[] __initdata = {
};

static struct slim_boardinfo apq8064_slim_devices[] = {
	{
		.bus_num = 1,
		.slim_slave = &apq8064_slim_tabla,
	},
	{
		.bus_num = 1,
		.slim_slave = &apq8064_slim_tabla20,
	},
	/* add more slimbus slaves as needed */
};

static struct msm_i2c_platform_data apq8064_i2c_qup_gsbi1_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
};

static struct msm_i2c_platform_data apq8064_i2c_qup_gsbi3_pdata = {
	.clk_freq = 384000,
	.src_clk_rate = 24000000,
};

static struct msm_i2c_platform_data apq8064_i2c_qup_gsbi4_pdata = {
	.clk_freq = 384000,

	.src_clk_rate = 24000000,
};


#ifdef CONFIG_BATTERY_MAX17058
static struct i2c_board_info i2c_gpio09_devs[] __initdata={
	{
		I2C_BOARD_INFO("max17058-i2c", 0x6C>>1),
	},
};
#endif

#ifdef CONFIG_CHARGER_SMB347_PT
static struct i2c_board_info i2c_gpio10_devs[] __initdata = {
	{
		I2C_BOARD_INFO("smb347-i2c", 0x0C>> 1), 
		//.platform_data = 
	},
};
#endif


#define GSBI_DUAL_MODE_CODE 0x60
#define MSM_GSBI1_PHYS		0x12440000
static void __init apq8064_i2c_init(void)
{
	apq8064_device_qup_i2c_gsbi1.dev.platform_data =
					&apq8064_i2c_qup_gsbi1_pdata;
	apq8064_device_qup_i2c_gsbi3.dev.platform_data =
					&apq8064_i2c_qup_gsbi3_pdata;
	apq8064_device_qup_i2c_gsbi1.dev.platform_data =
					&apq8064_i2c_qup_gsbi1_pdata;
	apq8064_device_qup_i2c_gsbi4.dev.platform_data =
					&apq8064_i2c_qup_gsbi4_pdata;
}


#define GPIO_KEY_VOLUME_UP		PM8921_GPIO_PM_TO_SYS(9)
#define GPIO_KEY_VOLUME_DOWN	PM8921_GPIO_PM_TO_SYS(10)


static struct gpio_keys_button vega_keys[] = {
	{
		.code           = KEY_VOLUMEUP,
		.gpio           = GPIO_KEY_VOLUME_UP,
		.desc           = "volume_up_key",
		.active_low     = 0,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},
	{
		.code           = KEY_VOLUMEDOWN,
		.gpio           = GPIO_KEY_VOLUME_DOWN,
		.desc           = "volume_down_key",
		.active_low     = 0,
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 15,
	},

};

static struct gpio_keys_platform_data vega_keys_data = {
	.buttons        = vega_keys,
	.nbuttons       = ARRAY_SIZE(vega_keys),
};

static struct platform_device vega_kp_pdev = {
	.name           = "gpio-keys",
	.id             = -1,
	.dev            = {
		.platform_data  = &vega_keys_data,
	},
};



/* Sensors DSPS platform data */
#define DSPS_PIL_GENERIC_NAME		"dsps"
static void __init apq8064_init_dsps(void)
{
	struct msm_dsps_platform_data *pdata =
		msm_dsps_device_8064.dev.platform_data;
	pdata->pil_name = DSPS_PIL_GENERIC_NAME;
	pdata->gpios = NULL;
	pdata->gpios_num = 0;

	platform_device_register(&msm_dsps_device_8064);
}

#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)
#define I2C_LIQUID (1 << 4)
#define I2C_MPQ_CDP	BIT(5)
#define I2C_MPQ_HRD	BIT(6)
#define I2C_MPQ_DTV	BIT(7)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry apq8064_i2c_devices[] __initdata = {
	{
		I2C_MPQ_CDP,
		APQ_8064_GSBI5_QUP_I2C_BUS_ID,
		cs8427_device_info,
		ARRAY_SIZE(cs8427_device_info),
	},
};

#define SX150X_EXP1_INT_N	PM8921_MPP_IRQ(PM8921_IRQ_BASE, 9)
#define SX150X_EXP2_INT_N	MSM_GPIO_TO_INT(81)

struct sx150x_platform_data mpq8064_sx150x_pdata[] = {
	[SX150X_EXP1] = {
		.gpio_base	= SX150X_EXP1_GPIO_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0,
		.io_pulldn_ena	= 0x0,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= SX150X_EXP1_INT_N,
		.irq_base	= SX150X_EXP1_IRQ_BASE,
	},
	[SX150X_EXP2] = {
		.gpio_base	= SX150X_EXP2_GPIO_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0f,
		.io_pulldn_ena	= 0x70,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= SX150X_EXP2_INT_N,
		.irq_base	= SX150X_EXP2_IRQ_BASE,
	},
	[SX150X_EXP3] = {
		.gpio_base	= SX150X_EXP3_GPIO_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0,
		.io_pulldn_ena	= 0x0,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= -1,
	},
	[SX150X_EXP4] = {
		.gpio_base	= SX150X_EXP4_GPIO_BASE,
		.oscio_is_gpo	= false,
		.io_pullup_ena	= 0x0,
		.io_pulldn_ena	= 0x0,
		.io_open_drain_ena = 0x0,
		.io_polarity	= 0,
		.irq_summary	= -1,
	},
};

static struct i2c_board_info sx150x_gpio_exp_info[] = {
	{
		I2C_BOARD_INFO("sx1509q", 0x70),
		.platform_data = &mpq8064_sx150x_pdata[SX150X_EXP1],
	},
	{
		I2C_BOARD_INFO("sx1508q", 0x23),
		.platform_data = &mpq8064_sx150x_pdata[SX150X_EXP2],
	},
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data = &mpq8064_sx150x_pdata[SX150X_EXP3],
	},
	{
		I2C_BOARD_INFO("sx1509q", 0x3E),
		.platform_data = &mpq8064_sx150x_pdata[SX150X_EXP4],
	},
};

#define MPQ8064_I2C_GSBI5_BUS_ID	5

static struct i2c_registry mpq8064_i2c_devices[] __initdata = {
	{
		I2C_MPQ_CDP,
		MPQ8064_I2C_GSBI5_BUS_ID,
		sx150x_gpio_exp_info,
		ARRAY_SIZE(sx150x_gpio_exp_info),
	},
};

static void __init register_i2c_devices(void)
{
	u8 mach_mask = 0;
	int i;

#ifdef CONFIG_MSM_CAMERA
	struct i2c_registry apq8064_camera_i2c_devices = {
		I2C_SURF | I2C_FFA | I2C_LIQUID | I2C_RUMI,
		APQ_8064_GSBI4_QUP_I2C_BUS_ID,
		apq8064_camera_board_info.board_info,
		apq8064_camera_board_info.num_i2c_board_info,
	};
#endif
	/* Build the matching 'supported_machs' bitmask */

	mach_mask = I2C_FFA;

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(apq8064_i2c_devices); ++i) {
		if (apq8064_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(apq8064_i2c_devices[i].bus,
						apq8064_i2c_devices[i].info,
						apq8064_i2c_devices[i].len);
	}
#ifdef CONFIG_MSM_CAMERA
	if (apq8064_camera_i2c_devices.machs & mach_mask)
		i2c_register_board_info(apq8064_camera_i2c_devices.bus,
			apq8064_camera_i2c_devices.info,
			apq8064_camera_i2c_devices.len);
#endif

	for (i = 0; i < ARRAY_SIZE(mpq8064_i2c_devices); ++i) {
		if (mpq8064_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(
					mpq8064_i2c_devices[i].bus,
					mpq8064_i2c_devices[i].info,
					mpq8064_i2c_devices[i].len);
	}

#ifdef CONFIG_BATTERY_MAX17058
	i2c_register_board_info(APQ_8064_GPIO09_I2C_BUS_ID, 
			i2c_gpio09_devs,
			ARRAY_SIZE(i2c_gpio09_devs));
#endif

#ifdef CONFIG_CHARGER_SMB347_PT
	i2c_register_board_info(APQ_8064_GPIO10_I2C_BUS_ID, 
			i2c_gpio10_devs,
			ARRAY_SIZE(i2c_gpio10_devs));
#endif

}

static void __init apq8064_common_init(void)
{
	u32 platform_version = socinfo_get_platform_version();

	platform_device_register(&msm_gpio_device);
	msm_tsens_early_init(&apq_tsens_pdata);
	msm_thermal_init(&msm_thermal_pdata);

	printk(KERN_INFO "platform subtype %d , soc info version minor %d\n",socinfo_get_platform_subtype(),
										SOCINFO_VERSION_MINOR(platform_version));

	if (socinfo_init() < 0)
		pr_err("socinfo_init() failed!\n");

	BUG_ON(msm_rpm_init(&apq8064_rpm_data));
	BUG_ON(msm_rpmrs_levels_init(&msm_rpmrs_data));

	regulator_suppress_info_printing();

	platform_device_register(&apq8064_device_rpm_regulator);
	if (socinfo_get_pmic_model() != PMIC_MODEL_PM8917)
		platform_device_register(&apq8064_pm8921_device_rpm_regulator);
	if (msm_xo_init())
		pr_err("Failed to initialize XO votes\n");

	msm_clock_init(&apq8064_clock_init_data);
	apq8064_init_gpiomux();
	apq8064_i2c_init();
	register_i2c_devices();

	apq8064_init_pmic();

	android_usb_pdata.swfi_latency =
		msm_rpmrs_levels[0].latency_us;

	apq8064_device_otg.dev.platform_data = &msm_otg_pdata;
	apq8064_ehci_host_init();
	apq8064_init_buses();

	platform_add_devices(early_common_devices,
				ARRAY_SIZE(early_common_devices));

	if (socinfo_get_pmic_model() != PMIC_MODEL_PM8917)
		platform_add_devices(pm8921_common_devices,
					ARRAY_SIZE(pm8921_common_devices));
	else
		platform_add_devices(pm8917_common_devices,
					ARRAY_SIZE(pm8917_common_devices));

	platform_add_devices(common_devices, ARRAY_SIZE(common_devices));
	
	platform_add_devices(common_not_mpq_devices,
			ARRAY_SIZE(common_not_mpq_devices));

	apq8064_device_qup_spi_gsbi5.dev.platform_data =
						&apq8064_qup_spi_gsbi5_pdata;

	msm_hsic_pdata.swfi_latency =
		msm_rpmrs_levels[0].latency_us;

	msm_hsic_pdata.log2_irq_thresh = 5,
	apq8064_device_hsic_host.dev.platform_data = &msm_hsic_pdata;
	device_initialize(&apq8064_device_hsic_host.dev);

	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_DSDA2) {
		apq8064_device_ehci_host3.dev.platform_data =
			&msm_ehci_host_pdata3;
		device_initialize(&apq8064_device_ehci_host3.dev);
	}

	apq8064_pm8xxx_gpio_mpp_init();
	apq8064_init_mmc();

	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_DSDA2) {
		amdm_8064_device.dev.platform_data = &amdm_platform_data;
		platform_device_register(&amdm_8064_device);
		bmdm_8064_device.dev.platform_data =
			&bmdm_platform_data;
		platform_device_register(&bmdm_8064_device);
	} else if (SOCINFO_VERSION_MINOR(platform_version) == 1) {
		i2s_mdm_8064_device.dev.platform_data =
			&mdm_platform_data;
		platform_device_register(&i2s_mdm_8064_device);
	} else {
		mdm_8064_device.dev.platform_data = &mdm_platform_data;
		platform_device_register(&mdm_8064_device);
	}

	platform_device_register(&apq8064_slim_ctrl);
	slim_register_board_info(apq8064_slim_devices,
		ARRAY_SIZE(apq8064_slim_devices));
	if (!PLATFORM_IS_MPQ8064()) {
		apq8064_init_dsps();
		platform_device_register(&msm_8960_riva);
	}
	if (cpu_is_apq8064ab())
		apq8064ab_update_krait_spm();
	msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	msm_spm_l2_init(msm_spm_l2_data);
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	apq8064_epm_adc_init();
	msm_pm_set_tz_retention_flag(1);
}

static void __init apq8064_allocate_memory_regions(void)
{
	apq8064_allocate_fb_region();
}
#if 0
#define PERF_DEF_OPTS (1 | 16)

static void enable_cpu_counters(void *data)
{
    /* enable user-mode access */
    printk(KERN_INFO "%s : enable user-mode access\n",__func__);
    asm ("mcr p15, 0, %0, c9, c14, 0\n\t" :: "r"(1));

	/* Program PMU and enable all counters */
	asm ("mcr p15, 0, %0, c9, c12, 0" :: "r"(PERF_DEF_OPTS));
    asm ("mcr p15, 0, %0, c9, c12, 1" :: "r"(0x8000000f));

}
#endif
static void __init apq8064_vega_init(void)
{
	//on_each_cpu(enable_cpu_counters, NULL, 1);

	if (meminfo_init(SYS_MEMORY, SZ_256M) < 0)
		pr_err("meminfo_init() failed!\n");

	apq8064_common_init();
	vega_add_sound_devices();
	vega_add_ramconsole_devices();
	msm_rotator_set_split_iommu_domain();
	platform_add_devices(cdp_devices, ARRAY_SIZE(cdp_devices));
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	apq8064_init_fb();
	apq8064_init_gpu();
	platform_add_devices(apq8064_footswitch, apq8064_num_footswitch);
#ifdef CONFIG_MSM_CAMERA
	apq8064_init_cam();
#endif
	platform_device_register(&vega_kp_pdev);

	//change_memory_power = &apq8064_change_memory_power;

	apq8064_init_input();

}

MACHINE_START(APQ8064_VEGA, "QCT APQ8064 VEGA")
	.map_io = apq8064_map_io,
	.reserve = apq8064_reserve,
	.init_irq = apq8064_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = apq8064_vega_init,
	.init_early = apq8064_allocate_memory_regions,
	.init_very_early = apq8064_early_reserve,
	.restart = msm_restart,
MACHINE_END

