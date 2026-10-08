/**
 *******************************************************************************
 * @file  bsp.c
 * @brief Loader board support package implementation.
 *******************************************************************************
 */

#include "bsp.h"
#include "hc32_ll.h"

#include "main.h"
#include "can.h"
#include "clock.h"
#include "flash.h"
#include "uart3.h"
#include "base.h"

const struct firmware_identity g_loader_identity
    __attribute__((used, aligned(8), section(".boot_identity"))) = {
    .build_time = FW_BUILD_VERSION,
    .magic = {'F', 'I', 'D', '1'},
    .product_id = FIRMWARE_PRODUCT_ID,
    .version = BOOT_FIRMWARE_VERSION,
    .reserved = {0U},
};

/// @brief Initialize SysTick for the loader polling loop.
static void systick_init(void)
{
    SysTick->LOAD = (HCLK_VALUE / 2000UL) - 1UL;
    SysTick->VAL = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Keep TICKINT disabled.
}

/// @brief Initialize loader board support peripherals.
void bsp_init(void)
{
    /* English comment. */
    LL_PERIPH_WE(LL_PERIPH_ALL);
    /* English comment. */
    clock_init();
    systick_init();
    flash_init();
    uart3_init(921600u);
    can1_init();
    /* English comment. */
    LL_PERIPH_WP(LL_PERIPH_ALL);
}

/// @brief Deinitialize loader board support peripherals before jumping to the app.
void bsp_deinit(void)
{
    /* Open peripheral write protection before touching system/peripheral registers. */
    LL_PERIPH_WE(LL_PERIPH_ALL);

    /* Stop SysTick to avoid periodic side effects during app jump sequence. */
    SysTick->CTRL = 0UL;
    SysTick->LOAD = 0UL;
    SysTick->VAL = 0UL;

    /* Disable and deinitialize USART3 used by loader Modbus link. */
    USART_FuncCmd(CM_USART3, (USART_TX | USART_RX), DISABLE);
    (void)USART_DeInit(CM_USART3);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, DISABLE);

    /* Deinitialize the F460 classical CAN peripheral and gate its clock. */
    (void)CAN_DeInit(CM_CAN);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN, DISABLE);

    /* Keep flash control registers in lock state before leaving loader. */
    flash_init();

    /* Restore peripheral write protection. */
    LL_PERIPH_WP(LL_PERIPH_ALL);
}
