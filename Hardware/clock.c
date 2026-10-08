/**
 *******************************************************************************
 * @file  clock.c
 * @brief Loader clock initialization implementation.
 *******************************************************************************
 */

#include "clock.h"

#include "main.h"

/// @brief Initialize the XTAL pins for the loader clock source.
static void clock_xtal_pin_init(void)
{
    stc_gpio_init_t stc_gpio_init;

    /* PH0 set to XTAL-IN. */
    (void)GPIO_StructInit(&stc_gpio_init);
    stc_gpio_init.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_H, GPIO_PIN_00, &stc_gpio_init);

    /* PH1 set to XTAL-OUT. */
    (void)GPIO_StructInit(&stc_gpio_init);
    stc_gpio_init.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_H, GPIO_PIN_01, &stc_gpio_init);
}

/// @brief Initialize the loader system clock tree.
void clock_init(void)
{
    /* Configure XTAL related GPIO before XTAL starts. */
    clock_xtal_pin_init();

    /* Match the F460 XHCode clock tree: 8 MHz XTAL -> 200 MHz MPLL. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 |
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));

    SRAM_SetWaitCycle(SRAM_SRAM_ALL, SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    EFM_SetWaitCycle(EFM_WAIT_CYCLE5);

    /* XTAL config */
    stc_clock_xtal_init_t stc_xtal_init;
    (void)CLK_XtalStructInit(&stc_xtal_init);
    stc_xtal_init.u8State = CLK_XTAL_ON;
    stc_xtal_init.u8Drv = CLK_XTAL_DRV_HIGH;
    stc_xtal_init.u8Mode = CLK_XTAL_MD_OSC;
    stc_xtal_init.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stc_xtal_init);

    /* MPLL config: 8 MHz / 1 * 50 / 2 = 200 MHz. */
    stc_clock_pll_init_t stc_pll_init;
    (void)CLK_PLLStructInit(&stc_pll_init);
    stc_pll_init.PLLCFGR = 0UL;
    stc_pll_init.PLLCFGR_f.PLLM = (1UL - 1UL);
    stc_pll_init.PLLCFGR_f.PLLN = (50UL - 1UL);
    stc_pll_init.PLLCFGR_f.PLLP = (2UL - 1UL);
    stc_pll_init.PLLCFGR_f.PLLQ = (2UL - 1UL);
    stc_pll_init.PLLCFGR_f.PLLR = (2UL - 1UL);
    stc_pll_init.u8PLLState = CLK_PLL_ON;
    stc_pll_init.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stc_pll_init);

    /* 3 cycles for 150MHz ~ 200MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);

    /* Switch the F460 power mode before selecting the 200 MHz clock. */
    PWC_HighSpeedToHighPerformance();

    /* Set the system clock source */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
}
