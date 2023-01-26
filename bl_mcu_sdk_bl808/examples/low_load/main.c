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
#include "pds_reg.h"
#include "usb_reg.h"
#include "emac_reg.h"

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

#ifdef CPU_M0

static uint32_t ipc_irqs[32] = {
    [BFLB_IPC_DEVICE_SDHCI] = SDH_IRQn,
    [BFLB_IPC_DEVICE_UART2] = UART2_IRQn,
    [BFLB_IPC_DEVICE_USB]   = USB_IRQn,
    [BFLB_IPC_DEVICE_EMAC]  = EMAC_IRQn,
    [BFLB_IPC_DEVICE_GPIO] = GPIO_INT0_IRQn,
    0,
};

static uint32_t irq_stats[32] = { 0 };

static void print_irq_stats(void)
{
    MSG("IRQS: SDHCI: %d, UART2: %d, USB: %d, EMAC: %d\r\n",
        irq_stats[BFLB_IPC_DEVICE_SDHCI],
        irq_stats[BFLB_IPC_DEVICE_UART2],
        irq_stats[BFLB_IPC_DEVICE_USB],
        irq_stats[BFLB_IPC_DEVICE_EMAC]);
}

static void Send_IPC_IRQ(int device)
{
    irq_stats[device]++;
    CPU_Interrupt_Disable(ipc_irqs[device]);
    BL_WR_REG(IPC2_BASE, IPC_CPU1_IPC_ISWR, (1 << device));
}

void SDH_MMC1_IRQHandler(void)
{
    bflb_platform_printf("S");
    Send_IPC_IRQ(BFLB_IPC_DEVICE_SDHCI);
}

void UART2_IRQHandler(void)
{
    bflb_platform_printf("U");
    Send_IPC_IRQ(BFLB_IPC_DEVICE_UART2);
}

void USB_IRQHandler(void)
{
    bflb_platform_printf("B");
    Send_IPC_IRQ(BFLB_IPC_DEVICE_USB);
}

void EMAC_IRQHandler(void)
{
    bflb_platform_printf("E");
    Send_IPC_IRQ(BFLB_IPC_DEVICE_EMAC);
}

void GPIO_IRQHandler(void)
{
    bflb_platform_printf("G");
    Send_IPC_IRQ(BFLB_IPC_DEVICE_GPIO);
}

static void IPC_M0_IRQHandler(void)
{
    int i;
    uint32_t irqStatus = IPC_M0_Get_Int_Raw_Status();

    for (i = 0; i < sizeof(irqStatus) * 8; i++) {
        if (irqStatus & (1 << i))
            CPU_Interrupt_Enable(ipc_irqs[i]);
    }

    BL_WR_REG(IPC0_BASE, IPC_CPU0_IPC_ICR, irqStatus);
}

#if 0
static void dump_emac(void)
{
    MSG("GLB base: 0x%08x\r\n", GLB_BASE);
    MSG("ETH_CFG0   [0x%08x]: 0x%08x ",    GLB_BASE + GLB_ETH_CFG0_OFFSET,  BL_RD_REG(GLB_BASE, GLB_ETH_CFG0));
    MSG("CGEN_CFG2  [0x%08x]: 0x%08x\r\n", GLB_BASE + GLB_CGEN_CFG2_OFFSET, BL_RD_REG(GLB_BASE, GLB_CGEN_CFG2));
    MSG("GPIO_CFG24 [0x%08x]: 0x%08x ",    GLB_BASE + GLB_GPIO_CFG24_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG24));
    MSG("GPIO_CFG25 [0x%08x]: 0x%08x\r\n", GLB_BASE + GLB_GPIO_CFG25_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG25));
    MSG("GPIO_CFG26 [0x%08x]: 0x%08x ",    GLB_BASE + GLB_GPIO_CFG26_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG26));
    MSG("GPIO_CFG27 [0x%08x]: 0x%08x\r\n", GLB_BASE + GLB_GPIO_CFG27_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG27));
    MSG("GPIO_CFG28 [0x%08x]: 0x%08x ",    GLB_BASE + GLB_GPIO_CFG28_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG28));
    MSG("GPIO_CFG29 [0x%08x]: 0x%08x\r\n", GLB_BASE + GLB_GPIO_CFG29_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG29));
    MSG("GPIO_CFG30 [0x%08x]: 0x%08x ",    GLB_BASE + GLB_GPIO_CFG30_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG30));
    MSG("GPIO_CFG31 [0x%08x]: 0x%08x\r\n", GLB_BASE + GLB_GPIO_CFG31_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG31));
    MSG("GPIO_CFG32 [0x%08x]: 0x%08x ",    GLB_BASE + GLB_GPIO_CFG32_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG32));
    MSG("GPIO_CFG33 [0x%08x]: 0x%08x\r\n", GLB_BASE + GLB_GPIO_CFG33_OFFSET,  BL_RD_REG(GLB_BASE, GLB_GPIO_CFG33));

    MSG("EMAC base: 0x%08x\r\n", EMAC_BASE);
    MSG("MODE       [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_MODE_OFFSET,        BL_RD_REG(EMAC_BASE, EMAC_MODE));
    MSG("INT_SOURCE [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_INT_SOURCE_OFFSET,  BL_RD_REG(EMAC_BASE, EMAC_INT_SOURCE));
    MSG("INT_MASK   [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_INT_MASK_OFFSET,    BL_RD_REG(EMAC_BASE, EMAC_INT_MASK));
    MSG("IPGT       [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_IPGT_OFFSET,        BL_RD_REG(EMAC_BASE, EMAC_IPGT));
    MSG("PACKETLEN  [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_PACKETLEN_OFFSET,   BL_RD_REG(EMAC_BASE, EMAC_PACKETLEN));
    MSG("COLLCONFIG [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_COLLCONFIG_OFFSET,  BL_RD_REG(EMAC_BASE, EMAC_COLLCONFIG));
    MSG("TX_BD_NUM  [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_TX_BD_NUM_OFFSET,   BL_RD_REG(EMAC_BASE, EMAC_TX_BD_NUM));
    MSG("TX_BD_NUM  [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_TX_BD_NUM_OFFSET,   BL_RD_REG(EMAC_BASE, EMAC_TX_BD_NUM));
    MSG("MIIMODE    [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_MIIMODE_OFFSET,     BL_RD_REG(EMAC_BASE, EMAC_MIIMODE));
    MSG("MIICOMMAND [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_MIICOMMAND_OFFSET,  BL_RD_REG(EMAC_BASE, EMAC_MIICOMMAND));
    MSG("MIIADDRESS [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_MIIADDRESS_OFFSET,  BL_RD_REG(EMAC_BASE, EMAC_MIIADDRESS));
    MSG("MIITX_DATA [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_MIITX_DATA_OFFSET,  BL_RD_REG(EMAC_BASE, EMAC_MIITX_DATA));
    MSG("MIIRX_DATA [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_MIIRX_DATA_OFFSET,  BL_RD_REG(EMAC_BASE, EMAC_MIIRX_DATA));
    MSG("MIISTATUS  [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_MIISTATUS_OFFSET,   BL_RD_REG(EMAC_BASE, EMAC_MIISTATUS));
    MSG("MAC_ADDR0  [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_MAC_ADDR0_OFFSET,   BL_RD_REG(EMAC_BASE, EMAC_MAC_ADDR0));
    MSG("MAC_ADDR1  [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_MAC_ADDR1_OFFSET,   BL_RD_REG(EMAC_BASE, EMAC_MAC_ADDR1));
    MSG("HASH0_ADDR [0x%08x]: 0x%08x ",    EMAC_BASE + EMAC_HASH0_ADDR_OFFSET,  BL_RD_REG(EMAC_BASE, EMAC_HASH0_ADDR));
    MSG("HASH1_ADDR [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_HASH1_ADDR_OFFSET,  BL_RD_REG(EMAC_BASE, EMAC_HASH1_ADDR));
    MSG("TXCTRL     [0x%08x]: 0x%08x\r\n", EMAC_BASE + EMAC_TXCTRL_OFFSET,      BL_RD_REG(EMAC_BASE, EMAC_TXCTRL));
}
#endif

#if 0
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

static void bflb_usb_phy_init(void)
{
    uint32_t regval;

    /* USB_PHY_CTRL[3:2] reg_usb_phy_xtlsel=0                             */
    /* 2000e504 = 0x40; #100; USB_PHY_CTRL[6] reg_pu_usb20_psw=1 (VCC33A) */
    /* 2000e504 = 0x41; #500; USB_PHY_CTRL[0] reg_usb_phy_ponrst=1        */
    /* 2000e500 = 0x20; #100; USB_CTL[0] reg_usb_sw_rst_n=0               */
    /* 2000e500 = 0x22; #500; USB_CTL[1] reg_usb_ext_susp_n=1             */
    /* 2000e500 = 0x23; #100; USB_CTL[0] reg_usb_sw_rst_n=1               */
    /* #1.2ms; wait UCLK                                                  */
    /* wait(soc616_b0.usb_uclk);                                          */

    regval = BL_RD_REG(PDS_BASE, PDS_USB_PHY_CTRL);
    regval &= ~PDS_REG_USB_PHY_XTLSEL_MSK;
    BL_WR_REG(PDS_BASE, PDS_USB_PHY_CTRL, regval);

    regval = BL_RD_REG(PDS_BASE, PDS_USB_PHY_CTRL);
    regval |= PDS_REG_PU_USB20_PSW_MSK;
    BL_WR_REG(PDS_BASE, PDS_USB_PHY_CTRL, regval);

    regval = BL_RD_REG(PDS_BASE, PDS_USB_PHY_CTRL);
    regval |= PDS_REG_USB_PHY_PONRST_MSK;
    BL_WR_REG(PDS_BASE, PDS_USB_PHY_CTRL, regval);

    /* greater than 5T */
    arch_delay_us(1);

    regval = BL_RD_REG(PDS_BASE, PDS_USB_CTL);
    regval &= ~PDS_REG_USB_SW_RST_N_MSK;
    BL_WR_REG(PDS_BASE, PDS_USB_CTL, regval);

    /* greater than 5T */
    arch_delay_us(1);

    regval = BL_RD_REG(PDS_BASE, PDS_USB_CTL);
    regval |= PDS_REG_USB_EXT_SUSP_N_MSK;
    BL_WR_REG(PDS_BASE, PDS_USB_CTL, regval);

    /* wait UCLK 1.2ms */
    arch_delay_ms(3);

    regval = BL_RD_REG(PDS_BASE, PDS_USB_CTL);
    regval |= PDS_REG_USB_SW_RST_N_MSK;
    BL_WR_REG(PDS_BASE, PDS_USB_CTL, regval);

    arch_delay_ms(2);
}

static void usb_hc_low_level_init(void)
{
    uint32_t regval;

    bflb_usb_phy_init();

    /* enable device-A for host */
    regval = BL_RD_REG(PDS_BASE, PDS_USB_CTL);
    regval &= ~PDS_REG_USB_IDDIG_MSK;
    BL_WR_REG(PDS_BASE, PDS_USB_CTL, regval);

    regval = BL_RD_REG(USB_BASE, USB_OTG_CSR);
    regval |= USB_A_BUS_DROP_HOV_MSK;
    regval &= ~USB_A_BUS_REQ_HOV_MSK;
    BL_WR_REG(USB_BASE, USB_OTG_CSR, regval);

    arch_delay_ms(10);

    /* enable vbus and bus control */
    regval = BL_RD_REG(USB_BASE, USB_OTG_CSR);
    regval &= ~USB_A_BUS_DROP_HOV_MSK;
    regval |= USB_A_BUS_REQ_HOV_MSK;
    BL_WR_REG(USB_BASE, USB_OTG_CSR, regval);

    regval = BL_RD_REG(USB_BASE, USB_GLB_INT);
    regval |= USB_MDEV_INT_MSK;
    regval |= USB_MOTG_INT_MSK;
    regval &= ~USB_MHC_INT_MSK;
    BL_WR_REG(USB_BASE, USB_GLB_INT, regval);
}

static void emac_low_level_init(void)
{
    GLB_GPIO_Cfg_Type gpio_cfg;
    int pin;

    GLB_Set_EMAC_CLK(1);
    GLB_Set_ETH_REF_O_CLK_Sel(GLB_ETH_REF_CLK_OUT_OUTSIDE_50M);
    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_EMAC);

    for (pin = GLB_GPIO_PIN_24; pin <= GLB_GPIO_PIN_33; pin++) {
        gpio_cfg.drive = 1;
        gpio_cfg.smtCtrl = 1;
        gpio_cfg.outputMode = 0;
        gpio_cfg.gpioMode = GPIO_MODE_AF;
        gpio_cfg.pullType = GPIO_PULL_UP;
        gpio_cfg.gpioPin = pin;
        gpio_cfg.gpioFun = GPIO_FUN_ETHER_MAC;
        GLB_GPIO_Init(&gpio_cfg);
    }
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

    MSG("initialize USB OTG to host mode\r\n");
    usb_hc_low_level_init();

    MSG("initialize EMAC\r\n");
    emac_low_level_init();

    MSG("registering IPC interrupt handler\r\n");
    Interrupt_Handler_Register(IPC_M0_IRQn, IPC_M0_IRQHandler);
    IPC_M0_Int_Unmask_By_Word(0xffffffff);
    CPU_Interrupt_Enable(IPC_M0_IRQn);

    MSG("registering SDH, UART2, USB, EMAC, GPIO interrupt handlers\r\n");
    Interrupt_Handler_Register(SDH_IRQn, SDH_MMC1_IRQHandler);
    Interrupt_Handler_Register(UART2_IRQn, UART2_IRQHandler);
    Interrupt_Handler_Register(USB_IRQn, USB_IRQHandler);
    Interrupt_Handler_Register(EMAC_IRQn, EMAC_IRQHandler);
    Interrupt_Handler_Register(EMAC2_IRQn, EMAC_IRQHandler);
    Interrupt_Handler_Register(GPIO_INT0_IRQn, GPIO_IRQHandler);
    CPU_Interrupt_Enable(SDH_IRQn);
    CPU_Interrupt_Enable(UART2_IRQn);
    CPU_Interrupt_Enable(USB_IRQn);
    CPU_Interrupt_Enable(EMAC_IRQn);
    CPU_Interrupt_Enable(EMAC2_IRQn);
    CPU_Interrupt_Enable(GPIO_INT0_IRQn);

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
              print_irq_stats();
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
