/**
 * @file main.c
 * @brief
 *
 * Copyright (c) 2022 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */
#include "bflb_platform.h"
#include "rv_hpm.h"
#include "rv_pmp.h"
#include "hal_mtimer.h"
// #include "bl808_lz4d.h"
#include "bl808_psram_uhs.h"
#include "bl808_glb.h"
#include "bl808_gpio.h"
#include "bl808_ipc.h"
#include "sdh_reg.h"
#include "bflb_ipc.h"

extern void unlz4(const void *aSource, void *aDestination, uint32_t FileLen);

#define PSRAM_BASIC_ADDR 0x50000000
#define VRAM_BASIC_ADDR  0x3f008000

#define VM_LINUX_SRC_ADDR 0x580A0000 // 4M 3980268
#define VM_LINUX_DST_ADDR 0x50000000

#define OPENSBI_SRC_ADDR 0x58090000 // 64K 0xc000
#define OPENSBI_DST_ADDR 0x3eff0000

#define DTB_SRC_ADDR 0x58080000 // 64k
#define DTB_DST_ADDR 0x51ff8000

#if (__riscv_xlen == 64)
/* linux pmp setting */
const pmp_config_entry_t pmp_entry_tab[8] = {
    [0] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x20000000,
        .entry_pa_length = PMP_REG_SZ_1M,
    },

    [1] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x30000000,
        .entry_pa_length = PMP_REG_SZ_1M,
    },

    [2] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_X | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x3eff0000,
        .entry_pa_length = 0x10000,
    },

    [3] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_X | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x40000000,
        .entry_pa_length = PMP_REG_SZ_16K,
    },

    [4] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_X | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x50000000,
        .entry_pa_length = 0x4000000,
    },

    [5] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x58000000,
        .entry_pa_length = 0x4000000,
    },

    [6] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0xe0000000,
        .entry_pa_length = 0x8000000,
    },

    [7] = {
        .entry_flag = ENTRY_FLAG_ADDR_TOR,
        .entry_pa_base = 0xffffffffff, /* 40-bit PA */
        .entry_pa_length = 0,
    },
};

#endif

void linux_load()
{
    MSG("linux load start... \r\n");
    uint32_t *pSrc, *pDest;
    uint32_t header_kernel_len = 0;
    header_kernel_len = *(volatile uint32_t *)(VM_LINUX_SRC_ADDR - 4);
    /* Copy and unlz4 vm linux code */
    MSG("len:0x%08x\r\n", header_kernel_len);
    __NOP_BARRIER();

    unlz4((const void *)VM_LINUX_SRC_ADDR, (void *)VM_LINUX_DST_ADDR, header_kernel_len /*3980268*/ /*3993116 v0.3.0*/ /*4010168*/);

    /* let's start */
    /* there are 7bytes file head that lz4d HW IP does not need, skip! */
    // LZ4D_Decompress((const void *)(VM_LINUX_SRC_ADDR + 7), (void *)VM_LINUX_DST_ADDR);

    /* method 1: wait when done */
    // while (!LZ4D_GetStatus(LZ4D_STATUS_DONE))
    // ;
    // __ISB();
    MSG("vm linux load done!\r\n");

    /* Copy dtb code */
    pSrc = (uint32_t *)DTB_SRC_ADDR;
    pDest = (uint32_t *)DTB_DST_ADDR;
    memcpy((void *)pDest, (void *)pSrc, 0x10000);
    MSG("dtb load done!\r\n");

    /* Copy opensbi code */
    pSrc = (uint32_t *)OPENSBI_SRC_ADDR;
    pDest = (uint32_t *)OPENSBI_DST_ADDR;
    memcpy((void *)pDest, (void *)pSrc, 0xc000);
    MSG("opensbi load done!\r\n");

    csi_dcache_clean_invalid();
    // csi_dcache_clean();
}

#define SDH_GetIntStatus()       BL_RD_REG(SDH_BASE, SDH_SD_NORMAL_INT_STATUS);
#define SDH_GetIntEnableStatus() BL_RD_REG(SDH_BASE, SDH_SD_NORMAL_INT_STATUS_INT_EN);
#define SDH_ClearIntStatus(mask) BL_WR_REG(SDH_BASE, SDH_SD_NORMAL_INT_STATUS, (mask));

void SDH_MMC1_IRQHandler(void)
{
    uint32_t intFlag, intMask;

    CPU_Interrupt_Disable(SDH_IRQn);
    //    MSG("%s\r\n", __func__);

    intFlag = SDH_GetIntStatus();
    intMask = SDH_GetIntEnableStatus();
    intFlag &= intMask;

    //    MSG("Triggering D0\r\n");
    IPC_M0_Trigger_D0(BFLB_IPC_DEVICE_SDHCI);

    //    SDH_ClearIntStatus(intFlag);
    return;
}

void UART2_IRQHandler(void)
{
    CPU_Interrupt_Disable(UART2_IRQn);
    IPC_M0_Trigger_D0(BFLB_IPC_DEVICE_UART2);
    return;
}

#ifdef CPU_M0
static void lp_ipc_handler(uint32_t src)
{
    MSG("%s: src: 0x%08x\r\n", __func__, src);
}

static uint32_t ipc_irqs[32] = {
    [BFLB_IPC_DEVICE_SDHCI] = SDH_IRQn,
    [BFLB_IPC_DEVICE_UART2] = UART2_IRQn,
    0,
};

static void d0_ipc_handler(uint32_t src)
{
    for (int bit = 0; src != 0 && bit < 32; bit++)
    {
        if (((src >> bit) & 1) && ipc_irqs[bit] != 0) CPU_Interrupt_Enable(ipc_irqs[bit]);

        src &= ~(1 << bit);
    }
}
#endif

#ifdef CPU_M0
static void dump_ipc(unsigned int base)
{
    MSG("base: 0x%08x\n", base);
    MSG("CPU1 ISWR [0x%08x]: 0x%08x ",   base + IPC_CPU1_IPC_ISWR_OFFSET,  BL_RD_REG(base, IPC_CPU1_IPC_ISWR));
    MSG("CPU1 IRSRR[0x%08x]: 0x%08x\n",  base + IPC_CPU1_IPC_IRSRR_OFFSET, BL_RD_REG(base, IPC_CPU1_IPC_IRSRR));
    MSG("CPU1 ICR  [0x%08x]: 0x%08x ",   base + IPC_CPU1_IPC_ICR_OFFSET,   BL_RD_REG(base, IPC_CPU1_IPC_ICR));
    MSG("CPU1 IUSR [0x%08x]: 0x%08x\n",  base + IPC_CPU1_IPC_IUSR_OFFSET,  BL_RD_REG(base, IPC_CPU1_IPC_IUSR));
    MSG("CPU1 IUCR [0x%08x]: 0x%08x ",   base + IPC_CPU1_IPC_IUCR_OFFSET,  BL_RD_REG(base, IPC_CPU1_IPC_IUCR));
    MSG("CPU1 ILSLR[0x%08x]: 0x%08x\n",  base + IPC_CPU1_IPC_ILSLR_OFFSET, BL_RD_REG(base, IPC_CPU1_IPC_ILSLR));
    MSG("CPU1 ILSHR[0x%08x]: 0x%08x ",   base + IPC_CPU1_IPC_ILSHR_OFFSET, BL_RD_REG(base, IPC_CPU1_IPC_ILSHR));
    MSG("CPU1 ISR  [0x%08x]: 0x%08x\n",  base + IPC_CPU1_IPC_ISR_OFFSET,   BL_RD_REG(base, IPC_CPU1_IPC_ISR));

    MSG("CPU0 ISWR [0x%08x]: 0x%08x ",   base + IPC_CPU0_IPC_ISWR_OFFSET,  BL_RD_REG(base, IPC_CPU0_IPC_ISWR));
    MSG("CPU0 IRSRR[0x%08x]: 0x%08x\n",  base + IPC_CPU0_IPC_IRSRR_OFFSET, BL_RD_REG(base, IPC_CPU0_IPC_IRSRR));
    MSG("CPU0 ICR  [0x%08x]: 0x%08x ",   base + IPC_CPU0_IPC_ICR_OFFSET,   BL_RD_REG(base, IPC_CPU0_IPC_ICR));
    MSG("CPU0 IUSR [0x%08x]: 0x%08x\n",  base + IPC_CPU0_IPC_IUSR_OFFSET,  BL_RD_REG(base, IPC_CPU0_IPC_IUSR));
    MSG("CPU0 IUCR [0x%08x]: 0x%08x ",   base + IPC_CPU0_IPC_IUCR_OFFSET,  BL_RD_REG(base, IPC_CPU0_IPC_IUCR));
    MSG("CPU0 ILSLR[0x%08x]: 0x%08x\n",  base + IPC_CPU0_IPC_ILSLR_OFFSET, BL_RD_REG(base, IPC_CPU0_IPC_ILSLR));
    MSG("CPU0 ILSHR[0x%08x]: 0x%08x ",   base + IPC_CPU0_IPC_ILSHR_OFFSET, BL_RD_REG(base, IPC_CPU0_IPC_ILSHR));
    MSG("CPU0 ISR  [0x%08x]: 0x%08x\n",  base + IPC_CPU0_IPC_ISR_OFFSET,   BL_RD_REG(base, IPC_CPU0_IPC_ISR));
}
#endif

int main(void)
{
    bflb_platform_init(0);
#ifdef CPU_M0
    MSG("E907 start...\r\n");
    mtimer_init();
    MSG("mtimer clk:%d\r\n", CPU_Get_MTimer_Clock());

    MSG("psram clk init ok!\r\n");
    // MSG("m0 main! size_t:%d\r\n", sizeof(size_t));

    IPC_M0_Init(d0_ipc_handler, lp_ipc_handler);

    MSG("registering SDH, UART2 interrupt handler\r\n");
    Interrupt_Handler_Register(SDH_IRQn, SDH_MMC1_IRQHandler);
    Interrupt_Handler_Register(UART2_IRQn, UART2_IRQHandler);
    CPU_Interrupt_Enable(SDH_IRQn);
    CPU_Interrupt_Enable(UART2_IRQn);
    {
      uint32_t intFlag;
      intFlag = SDH_GetIntStatus();
      MSG("SDH int status: 0x%x\n", intFlag);
    }

    csi_dcache_disable();
#ifdef DUALCORE
    BL_WR_WORD(IPC_SYNC_ADDR1, IPC_SYNC_FLAG);
    BL_WR_WORD(IPC_SYNC_ADDR2, IPC_SYNC_FLAG);
    L1C_DCache_Clean_By_Addr(IPC_SYNC_ADDR1, 8);
#endif

    while (1) {
      {
	static int x = 0;
	if ((x++ % 999999999) == 0) {
	  {
              //dump_ipc(IPC0_BASE);
              //dump_ipc(IPC1_BASE);
              uint32_t intFlag;
              intFlag = SDH_GetIntStatus();
              MSG("SDH int status: 0x%x\n", intFlag);
	  }
	}
      }
#ifdef __riscv_muldiv
        int dummy;
        /* In lieu of a halt instruction, induce a long-latency stall. */
        __asm__ __volatile__("div %0, %0, zero"
                             : "=r"(dummy));
#endif
    }

#endif

#ifdef CPU_D0

    // #define GLB_AHB_CLOCK_LZ4 (0x0008000000000000UL)

    // GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_LZ4);
    MSG("C906 start...\r\n");
    uint64_t start_time, stop_time;
    mtimer_init();
    MSG("mtimer clk:%d\r\n", CPU_Get_MTimer_Clock());
    // MSG("C906 main! size_t:%d\r\n", sizeof(size_t));
    bflb_platform_delay_ms(100);

    void (*opensbi)(int hart_id, int fdt_addr) = (void (*)(int hart_id, int fdt_addr))OPENSBI_DST_ADDR;

    start_time = bflb_platform_get_time_us();
    linux_load();
    stop_time = bflb_platform_get_time_us();
    MSG("\r\nload time: %ld us \r\n", (stop_time - start_time));

    __ASM volatile("csrw mcor, %0"
                   :
                   : "r"(0x30013));
    // csi_dcache_disable();
    // csi_icache_disable();
    // __set_MHINT(0x450c);
    rvpmp_init(pmp_entry_tab, sizeof(pmp_entry_tab) / sizeof(pmp_config_entry_t));
    __ISB();

    /* Linux clk and rtc clk setting */
    // unsigned int reg;
    // reg = *(volatile unsigned int *)0x30007004;
    // reg |= (0x0 << 0); // pll div --> 400Mhz
    // *(volatile unsigned int *)0x30000018 = 0x8000017b;

    /* set core volt 1.2 */
    // *(volatile unsigned int *)0x2000f814 = 0x14476c20;

    /* set dtb addr */
    /* go to run linux ... */
    opensbi(0, DTB_DST_ADDR);
#endif

    while (1) {
#ifdef __riscv_muldiv
        int dummy;
        /* In lieu of a halt instruction, induce a long-latency stall. */
        __asm__ __volatile__("div %0, %0, zero"
                             : "=r"(dummy));
#endif
    }
}
