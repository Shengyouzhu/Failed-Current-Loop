#include "vofa_telemetry.h"
#include "CurrentLoop.h"
#include "usart.h"

/* These values are written by the motor-control ISR or MotorTask. */
extern volatile float debug_i;
extern volatile float debug_id;
extern volatile float debug_i_raw;
extern volatile float debug_uq;
extern volatile float debug_ud;
extern float angle;
extern volatile float theta_e;
extern volatile uint32_t motor_control_state;
extern volatile uint32_t motor_sensor_fail_count;
extern volatile uint32_t motor_reset_reason;
extern volatile uint32_t motor_last_angle_adc_count;
extern float Motor_MM;

typedef struct
{
    float channel[VOFA_TELEMETRY_CHANNELS];
    uint8_t tail[4];
} VOFA_JustFloatFrame_t;

/* DMA must read a persistent buffer until USART transmission is complete. */
static VOFA_JustFloatFrame_t tx_frame;
static DMA_HandleTypeDef hdma_usart1_tx;
static uint32_t previous_sample_count;
static uint8_t initialized;

volatile uint32_t vofa_frames_started;
volatile uint32_t vofa_frames_skipped;
volatile uint32_t vofa_errors;

HAL_StatusTypeDef VOFA_TelemetryInit(void)
{
    if (initialized != 0U)
        return HAL_OK;

    if ((huart1.Instance != USART1) || (huart1.hdmatx != NULL))
    {
        vofa_errors++;
        return HAL_ERROR;
    }

    __HAL_RCC_DMA2_CLK_ENABLE();

    /* STM32F407 USART1_TX is DMA2 Stream7, Channel4. */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_usart1_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart1_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart1_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
        vofa_errors++;
        return HAL_ERROR;
    }

    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    /* Both IRQs are required: DMA completion is followed by USART TC. */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    tx_frame.tail[0] = 0x00U;
    tx_frame.tail[1] = 0x00U;
    tx_frame.tail[2] = 0x80U;
    tx_frame.tail[3] = 0x7FU;
    previous_sample_count = motor_current.sample_count;
    initialized = 1U;
    return HAL_OK;
}

void VOFA_TelemetrySend(void)
{
    uint32_t irq_state;
    uint32_t sample_count;
    uint32_t last_angle_adc_count;
    HAL_StatusTypeDef status;

    if (initialized == 0U)
        return;

    /* Never overwrite the buffer while the previous DMA transfer is active. */
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        vofa_frames_skipped++;
        return;
    }

    /* Copy one coherent set of ISR variables; keep IRQ masking very short. */
    irq_state = __get_PRIMASK();
    __disable_irq();
    tx_frame.channel[0] = CurrentLoop_GetTargetIq();
    tx_frame.channel[1] = debug_i;
    tx_frame.channel[2] = debug_id;
    tx_frame.channel[18] = debug_i_raw;
    tx_frame.channel[3] = debug_uq;
    tx_frame.channel[4] = debug_ud;
    tx_frame.channel[5] = motor_current.ia;
    tx_frame.channel[6] = motor_current.ib;
    tx_frame.channel[7] = motor_current.ic;
    tx_frame.channel[9] = theta_e;
    tx_frame.channel[10] = angle;
    tx_frame.channel[11] = Motor_MM;
    sample_count = motor_current.sample_count;
    last_angle_adc_count = motor_last_angle_adc_count;
    tx_frame.channel[13] = (float)motor_control_state;
    tx_frame.channel[14] = (float)motor_sensor_fail_count;
    tx_frame.channel[15] = (float)motor_reset_reason;
    __set_PRIMASK(irq_state);

    tx_frame.channel[8] = tx_frame.channel[5] + tx_frame.channel[6]
                        + tx_frame.channel[7];
    tx_frame.channel[11] *= 9.5492966f; /* rad/s to rpm */
    tx_frame.channel[12] = (float)(sample_count - previous_sample_count);
    tx_frame.channel[16] = (float)(sample_count - last_angle_adc_count);
    tx_frame.channel[17] = 0.0f; /* Legacy channel; rejection removed. */

    status = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&tx_frame,
                                   (uint16_t)sizeof(tx_frame));
    if (status == HAL_OK)
    {
        previous_sample_count = sample_count;
        vofa_frames_started++;
    }
    else if (status == HAL_BUSY)
    {
        vofa_frames_skipped++;
    }
    else
    {
        vofa_errors++;
    }
}

void DMA2_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
