/**
 *******************************************************************************
 * @file  uart3.c
 * @brief Loader USART initialization implementation.
 *******************************************************************************
 */

#include "uart3.h"

#include <stdarg.h>
#include <stdio.h>

#include "hc32_ll.h"
#define UART_PRINTF_BUFFER_SIZE (128U)

/// @brief Clear UART3 RX error flags that can block receive operations.
static void uart3_pin_init(void)
{
    /* Pin mapping from Loader.xml: PH2 -> USART3-RX, PC13 -> USART3-TX. */
    GPIO_SetFunc(GPIO_PORT_H, GPIO_PIN_02, GPIO_FUNC_33);
    GPIO_SetFunc(GPIO_PORT_C, GPIO_PIN_13, GPIO_FUNC_32);
}

/// @brief Clear UART3 RX error flags and drain stale data if needed.
static void uart_clear_rx_error(void)
{
    uint32_t u32_err_flags = 0UL;

    /* Check condition and branch. */
    if (USART_GetStatus(CM_USART3, USART_FLAG_OVERRUN) == SET)
    {
        u32_err_flags |= USART_FLAG_OVERRUN;
    }
    /* Check condition and branch. */
    if (USART_GetStatus(CM_USART3, USART_FLAG_FRAME_ERR) == SET)
    {
        u32_err_flags |= USART_FLAG_FRAME_ERR;
    }
    /* Check condition and branch. */
    if (USART_GetStatus(CM_USART3, USART_FLAG_PARITY_ERR) == SET)
    {
        u32_err_flags |= USART_FLAG_PARITY_ERR;
    }

    /* Check condition and branch. */
    if (u32_err_flags != 0UL)
    {
        /* Loop through current range. */
        while (USART_GetStatus(CM_USART3, USART_FLAG_RX_FULL) == SET)
        {
            (void)USART_ReadData(CM_USART3);
        }
        USART_ClearStatus(CM_USART3, u32_err_flags);
    }
}

/// @brief Initialize UART3 for the loader communication path.
/// @param bps UART baud rate in bps.
void uart3_init(uint32_t bps)
{
    stc_usart_uart_init_t stc_uart_init;

    /* Configure USART3 pins before peripheral setup. */
    uart3_pin_init();

    /* Enable USART3 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, ENABLE);

    /* Configure USART3 */
    (void)USART_DeInit(CM_USART3);
    (void)USART_UART_StructInit(&stc_uart_init);
    stc_uart_init.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stc_uart_init.u32ClockDiv = USART_CLK_DIV1;
    stc_uart_init.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stc_uart_init.u32Baudrate = bps;
    stc_uart_init.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stc_uart_init.u32StopBit = USART_STOPBIT_1BIT;
    stc_uart_init.u32Parity = USART_PARITY_NONE;
    stc_uart_init.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stc_uart_init.u32FirstBit = USART_FIRST_BIT_LSB;
    stc_uart_init.u32StartBitPolarity = USART_START_BIT_FALLING;
    /* F460 DDL exposes RTS as the no-CTSE path used without hardware flow control. */
    stc_uart_init.u32HWFlowControl = USART_HW_FLOWCTRL_RTS;
    (void)USART_UART_Init(CM_USART3, &stc_uart_init, NULL);

    /* Enable USART_TX | USART_RX function */
    USART_FuncCmd(CM_USART3, (USART_TX | USART_RX), ENABLE);

    /* Clear possible stale error/data state before first receive. */
    USART_ClearStatus(CM_USART3, USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR | USART_FLAG_PARITY_ERR);
    /* Loop through current range. */
    while (USART_GetStatus(CM_USART3, USART_FLAG_RX_FULL) == SET)
    {
        (void)USART_ReadData(CM_USART3);
    }
}

/// @brief Send bytes through UART3 in blocking mode.
/// @param data Source buffer pointer.
/// @param len Number of bytes to send.
void uart3_send(const void *data, uint32_t len)
{
    uint8_t *pu8_data = (uint8_t *)data;

    /* Check condition and branch. */
    if ((NULL == pu8_data) || (0UL == len))
    {
        return;
    }

    (void)USART_UART_Trans(CM_USART3, pu8_data, len, USART_MAX_TIMEOUT);
}

/// @brief Send bytes through UART3 in blocking mode.
/// @param data Source buffer pointer.
/// @param len Number of bytes to send.
/// @return Number of bytes received from UART3.
uint32_t uart3_recv(void *data, uint32_t len)
{
    uint8_t *pu8_data = (uint8_t *)data;
    uint32_t i;

    /* Check condition and branch. */
    if ((NULL == pu8_data) || (0UL == len))
    {
        return 0UL;
    }

    uart_clear_rx_error();

    /* Stop when the software buffer is full. */
    for (i = 0UL; i < len; i++)
    {
        /* Check condition and branch. */
        if (USART_GetStatus(CM_USART3, USART_FLAG_RX_FULL) == SET)
        {
            pu8_data[i] = (uint8_t)USART_ReadData(CM_USART3);
        }
        else
        {
            break;
        }
    }

    return i;
}

/// @brief Print a formatted string through UART3.
/// @param fmt `printf`-style format string.
void uart3_printf(const char *fmt, ...)
{
    char ac_buffer[UART_PRINTF_BUFFER_SIZE];
    int32_t s32_len;
    va_list va_args;

    /* Check condition and branch. */
    if (NULL == fmt)
    {
        return;
    }

    va_start(va_args, fmt);
    s32_len = vsnprintf(ac_buffer, sizeof(ac_buffer), fmt, va_args);
    va_end(va_args);

    /* Check condition and branch. */
    if (s32_len <= 0)
    {
        return;
    }

    /* English comment. */
    if ((uint32_t)s32_len >= sizeof(ac_buffer))
    {
        s32_len = (int32_t)(sizeof(ac_buffer) - 1UL);
    }

    uart3_send(ac_buffer, (uint32_t)s32_len);
}
