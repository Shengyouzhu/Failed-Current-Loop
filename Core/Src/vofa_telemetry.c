#include "vofa_telemetry.h"
#include "foc.h"
#include "haptic.h"
#include "usart.h"

typedef struct
{
    float channel[VOFA_TELEMETRY_CHANNELS];
    uint8_t tail[4];
} VOFA_Frame_t;

static VOFA_Frame_t tx_frame;
static DMA_HandleTypeDef hdma_usart1_tx;
static uint8_t initialized;
volatile uint32_t vofa_frames_started;
volatile uint32_t vofa_frames_skipped;
volatile uint32_t vofa_errors;

HAL_StatusTypeDef VOFA_TelemetryInit(void)
{
    if (initialized) return HAL_OK;
    if (huart1.Instance != USART1 || huart1.hdmatx != NULL) return HAL_ERROR;
    __HAL_RCC_DMA2_CLK_ENABLE();
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
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) return HAL_ERROR;
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    tx_frame.tail[0] = 0x00;
    tx_frame.tail[1] = 0x00;
    tx_frame.tail[2] = 0x80;
    tx_frame.tail[3] = 0x7F;
    initialized = 1U;
    return HAL_OK;
}

void VOFA_TelemetrySend(void)
{
    FOC_Feedback_t feedback;
    HAL_StatusTypeDef status;

    if (!initialized) return;
    /* Persistent frame storage is untouched until the previous DMA is done. */
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        vofa_frames_skipped++;
        return;
    }
    FOC_GetFeedback(&feedback);
    tx_frame.channel[1] = feedback.mechanical_speed;
    tx_frame.channel[4] = feedback.mechanical_angle;
    tx_frame.channel[6] = feedback.electrical_angle;
    tx_frame.channel[8] = (float)feedback.state;
    tx_frame.channel[9] = (float)feedback.sensor_fail_count;
    tx_frame.channel[10] = (float)feedback.sensor_status;
    tx_frame.channel[11] = (float)feedback.sensor_error;
    tx_frame.channel[12] = (float)feedback.sensor_fault_stage;
    tx_frame.channel[13] = (float)haptic_volume;
    tx_frame.channel[14] = (float)haptic_next_song_count;
    tx_frame.channel[15] = (float)haptic_previous_song_count;
		
    tx_frame.channel[0] = feedback.speed_reference * 9.5492966f;
    tx_frame.channel[1] *= 9.5492966f;
    tx_frame.channel[2] = feedback.applied_uq;
    tx_frame.channel[3] = feedback.position_reference * 0.15915494f;
    tx_frame.channel[4] *= 0.15915494f;
    tx_frame.channel[5] = (feedback.mode == FOC_MODE_POSITION)
        ? tx_frame.channel[3] - tx_frame.channel[4] : 0.0f;
    tx_frame.channel[7] = (float)feedback.mode;
    status = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&tx_frame, sizeof(tx_frame));
    if (status == HAL_OK) vofa_frames_started++;
    else if (status == HAL_BUSY) vofa_frames_skipped++;
    else vofa_errors++;
}

void DMA2_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
