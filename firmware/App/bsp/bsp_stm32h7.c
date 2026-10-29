/*
 * bsp_stm32h7.c - BSP for NUCLEO-H753ZI using the STM32 HAL.
 * Requires a CubeMX project with (see firmware/README.md):
 *   TIM4 CH3  PWM 20 kHz (ARR = MOTOR_PWM_COUNTS-1)  -> MDD10A PWM1 (PB8, CN7 D15)
 *   TIM2      encoder mode TI1+TI2, 32-bit            -> FIT0186 A/B
 *   TIM3 CH1/CH2 PWM 50 Hz, 1 MHz tick                -> servo A/B
 *   USART3    921600 8N1, RX interrupt                -> ST-LINK VCP
 *   GPIO out: MOTOR_DIR, LED_1..LED_4   GPIO in: END_MIN (END_MAX optional)
 * Compiled only on the target (USE_HAL_DRIVER is defined by CubeMX projects).
 */
#ifdef USE_HAL_DRIVER
#include "main.h"
#include "../bsp.h"
#include "../app.h"
#include "../config.h"

extern TIM_HandleTypeDef  htim2, htim3, htim4;

/* motor PWM timer/channel: must match firmware.ioc */
#define MOTOR_HTIM   htim4
#define MOTOR_CH     TIM_CHANNEL_3
extern UART_HandleTypeDef huart3;

static uint8_t rx_byte;

void bsp_start(void)
{
    /* microsecond clock from the DWT cycle counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_PWM_Start(&MOTOR_HTIM, MOTOR_CH);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void bsp_motor_write(uint32_t compare, bool positive)
{
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, positive ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&MOTOR_HTIM, MOTOR_CH, compare);
}

void bsp_servo_write(uint8_t channel, uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim3, channel == 0 ? TIM_CHANNEL_1 : TIM_CHANNEL_2, pulse_us);
}

void bsp_led_write(uint8_t mask)
{
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, (mask & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, (mask & 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, (mask & 4u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, (mask & 8u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int32_t bsp_encoder_read(void) { return (int32_t)__HAL_TIM_GET_COUNTER(&htim2); }

uint8_t bsp_endstops_read(void)
{
    uint8_t v = 0;
#if USE_END_MIN
    if (HAL_GPIO_ReadPin(END_MIN_GPIO_Port, END_MIN_Pin) == GPIO_PIN_SET) v |= 1u;
#endif
#if USE_END_MAX
    if (HAL_GPIO_ReadPin(END_MAX_GPIO_Port, END_MAX_Pin) == GPIO_PIN_SET) v |= 2u;
#endif
    return v;
}

uint32_t bsp_micros(void) { return DWT->CYCCNT / (SystemCoreClock / 1000000u); }
uint32_t bsp_millis(void) { return HAL_GetTick(); }

void bsp_uart_send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 5);
}

/* HAL callbacks: forward received bytes to the application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        app_uart_rx_byte(rx_byte);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {          /* overrun / framing: restart reception */
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}
#endif /* USE_HAL_DRIVER */
