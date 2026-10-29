/* stubs/main.h - minimal HAL stand-ins so bsp_stm32h7.c can be syntax-checked
 * on the host (make check-bsp). NOT used on the target. */
#include <stdint.h>
typedef struct { int Instance; } UART_HandleTypeDef;
typedef struct { int dummy; } TIM_HandleTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
#define USART3 1
#define TIM_CHANNEL_1 0x0u
#define TIM_CHANNEL_2 0x4u
#define TIM_CHANNEL_3 0x8u
#define TIM_CHANNEL_ALL 0x3Cu
extern struct { uint32_t DEMCR; } *CoreDebug;
extern struct { uint32_t LAR, CYCCNT, CTRL; } *DWT;
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
extern uint32_t SystemCoreClock;
#define MOTOR_DIR_GPIO_Port 0
#define MOTOR_DIR_Pin 0
#define LED_1_GPIO_Port 0
#define LED_1_Pin 0
#define LED_2_GPIO_Port 0
#define LED_2_Pin 0
#define LED_3_GPIO_Port 0
#define LED_3_Pin 0
#define LED_4_GPIO_Port 0
#define LED_4_Pin 0
#define END_MIN_GPIO_Port 0
#define END_MIN_Pin 0
#define END_MAX_GPIO_Port 0
#define END_MAX_Pin 0
void HAL_GPIO_WritePin(int port, int pin, GPIO_PinState s);
GPIO_PinState HAL_GPIO_ReadPin(int port, int pin);
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t c);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c);
int HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *b, uint16_t n);
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *b, uint16_t n, uint32_t t);
uint32_t HAL_GetTick(void);
#define __HAL_TIM_SET_COMPARE(h, c, v) ((void)(h), (void)(c), (void)(v))
#define __HAL_TIM_GET_COUNTER(h) ((void)(h), 0u)
#define __HAL_TIM_SET_COUNTER(h, v) ((void)(h), (void)(v))
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
