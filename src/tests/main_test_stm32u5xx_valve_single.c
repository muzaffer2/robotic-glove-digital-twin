#define HAL_SPI_MODULE_ENABLED
#include "stm32u5xx_hal.h"

#include <stdlib.h>
#include <string.h>

#include "clock_config.h"
#include "power_config.h"
#include "cache_enable.h"
#include "handlers/error.h"
#include "msp/uart_dma_app.h"
#include "msp/pressure_sensor_spi.h"

#define LED2_GPIO_PORT                          GPIOA
#define LED2_PIN                                GPIO_PIN_5
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOA_CLK_DISABLE()

/* =========================================================
 * USER CONFIGURATION (SPI BASINÇ SENSÖRLERİ)
 * ========================================================= */
#define PRESSURE_SAMPLE_PERIOD_MS        20U  // 50 Hz örnekleme hızı
#define PRESSURE_CONVERSION_WAIT_MS       5U  // 5 ms ABP2 dönüşüm bekleme süresi

/*
 * PA0 ve PA1 pinleri TIM2 PWM (MOSFET Gate) çıkışlarına ayrıldı.
 * İki adet Honeywell ABP2 sensörünün Pin 3 (CS) uçları PA2 ve PA3'e atandı.
 */
static PressureSensor_t pressure_sensors[] =
{
    {
        .cs_port = GPIOA,
        .cs_pin  = GPIO_PIN_2  // 1. Sensör (Üstteki) Pin 3 (CS) -> Kartta A2 yuvası
    },
    {
        .cs_port = GPIOA,
        .cs_pin  = GPIO_PIN_3  // 2. Sensör (Hortumlu) Pin 3 (CS) -> Kartta A3 yuvası
    }
};

#define NUMBER_OF_PRESSURE_SENSORS  ((uint8_t)(sizeof(pressure_sensors) / sizeof(pressure_sensors[0])))

static PressureSensorManager_t pressure_manager;

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_uart1_rx;
DMA_HandleTypeDef hdma_uart1_tx;
SPI_HandleTypeDef hspi2;

// Fonksiyon Prototipleri
static void GPIO_Init(void);
static void TIM2_PWM_Init(void);
static void SPI2_Init(void);
static void USART1_UART_Init(void);
static void USART1_GPIO_Init(void);
static void USART1_DMA_RX_Init(void);
static void USART1_DMA_TX_Init(void);
static void USART1_NVIC_Init(void);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);

#define PWM_MAX_COMPARE                  999U
#define DEFAULT_MAX_SAFE_PRESSURE_KPA    250.0f
#define ABSOLUTE_MAX_PRESSURE_KPA        1000.0f
#define TARGET_HYSTERESIS_KPA            3.0f

static void ApplicationProcess(void);
static void ProcessCommand(const char *command);
static void Valve1SetState(bool enabled);
static void Valve2SetState(bool enabled);
static void ValveSetPwm(uint8_t valve_id, uint16_t duty);
static void UpdateClosedLoopPressure(float p1_kpa, bool s1_connected);
static void ApplyOverpressureSafety(float p1_kpa, bool s1_connected);
static void SendStatus(void);

// Valf ve Kontrol Durum Değişkenleri
static bool valve1_state = false;
static bool valve2_state = false;
static uint16_t valve1_pwm = 0U;
static uint16_t valve2_pwm = 0U;

static float s0_zero_offset_kpa = 0.0f;
static float s1_zero_offset_kpa = 0.0f;

static bool auto_pressure_enabled = false;
static float target_pressure_kpa = 0.0f;
static float max_safe_pressure_kpa = DEFAULT_MAX_SAFE_PRESSURE_KPA;
static bool overpressure_tripped = false;

int main(void)
{
    HAL_Init();
    SystemClock_Config(); // SYSCLK = 160 MHz
    SystemPower_Config();
    CACHE_Enable();

    // 1. GPIO ve TIM2 PWM (PA0 & PA1 MOSFET Sürücüleri) Başlatma
    GPIO_Init();
    TIM2_PWM_Init();

    // 2. SPI2 (PB13, PB14, PB15 -> 625 kHz) ve Basınç Sensörlerini Başlatma
    SPI2_Init();
    HAL_SPI_MspInit(&hspi2);

    PressureSensor_Init(
        &pressure_manager,
        &hspi2,
        pressure_sensors,
        NUMBER_OF_PRESSURE_SENSORS,
        PRESSURE_SAMPLE_PERIOD_MS,
        PRESSURE_CONVERSION_WAIT_MS);

    // 3. UART1 (ST-LINK Virtual COM Port - 115200 Baud) ve DMA Başlatma
    USART1_GPIO_Init();
    USART1_UART_Init();
    USART1_DMA_RX_Init();
    USART1_DMA_TX_Init();
    USART1_NVIC_Init();

    // Başlangıçta her iki MOSFET'i (Valf/Pompa) güvenli şekilde kapat
    Valve1SetState(false);
    Valve2SetState(false);

    if ((huart1.hdmarx == NULL) || (huart1.hdmatx == NULL))
    {
        Error_Handler();
    }

    if (UART_AppInit() != UART_APP_OK)
    {
        Error_Handler();
    }

    (void)UART_AppSendStringAsync("SYSTEM,READY\r\n");

    uart_debug_enabled = false; // Python veri akışını bozmamak için debug logları kapalı

    while (1)
    {
        // A. Bilgisayardan (Python Digital Twin) gelen komutları işle
        ApplicationProcess();

        // B. SPI durum makinesini çalıştır
        PressureSensor_Task(&pressure_manager);

        // C. Yeni basınç verisi hazır olduğunda kontrol ve seri telemetri (50 Hz)
        if (PressureSensor_NewDataAvailable(&pressure_manager))
        {
            PressureSensor_t *s0 = PressureSensor_Get(&pressure_manager, 0);
            PressureSensor_t *s1 = PressureSensor_Get(&pressure_manager, 1);

            bool s0_ok = (s0 != NULL) && (s0->connected != 0U);
            bool s1_ok = (s1 != NULL) && (s1->connected != 0U);

            float p0 = s0_ok ? (s0->pressure - s0_zero_offset_kpa) : -1.0f;
            float p1 = s1_ok ? (s1->pressure - s1_zero_offset_kpa) : -1.0f;
            if (s0_ok && (p0 < -0.5f)) p0 = -0.5f;
            if (s1_ok && (p1 < -0.5f)) p1 = -0.5f;
            float t1 = s1_ok ? s1->temperature : -1.0f;

            if (auto_pressure_enabled)
            {
                UpdateClosedLoopPressure(p1, s1_ok);
            }

            ApplyOverpressureSafety(p1, s1_ok);

            UART_AppPrintf("DATA,S0:%.4f,S1:%.4f,V1:%u,V2:%u,PWM1:%u,PWM2:%u,T1:%.2f\r\n",
                           p0, p1,
                           valve1_state ? 1U : 0U,
                           valve2_state ? 1U : 0U,
                           valve1_pwm,
                           valve2_pwm,
                           t1);

            PressureSensor_ClearNewDataFlag(&pressure_manager);
        }
    }
}

/* =========================================================
 * KOMUT VE VALF KONTROL FONKSİYONLARI
 * ========================================================= */

static void ApplicationProcess(void)
{
    char command[UART_COMMAND_MAX_LENGTH];

    while (UART_AppGetCommand(command, sizeof(command)))
    {
        ProcessCommand(command);
    }
}

static void ProcessCommand(const char *command)
{
    if (command == NULL) return;

    if (strcmp(command, "PING") == 0)
    {
        (void)UART_AppSendStringAsync("PONG\r\n");
    }
    else if (strcmp(command, "VALVE,1,ON") == 0)
    {
        auto_pressure_enabled = false;
        Valve1SetState(true);
        (void)UART_AppSendStringAsync("ACK,VALVE,1,ON\r\n");
    }
    else if (strcmp(command, "VALVE,1,OFF") == 0)
    {
        auto_pressure_enabled = false;
        Valve1SetState(false);
        (void)UART_AppSendStringAsync("ACK,VALVE,1,OFF\r\n");
    }
    else if (strcmp(command, "VALVE,2,ON") == 0)
    {
        auto_pressure_enabled = false;
        Valve2SetState(true);
        (void)UART_AppSendStringAsync("ACK,VALVE,2,ON\r\n");
    }
    else if (strcmp(command, "VALVE,2,OFF") == 0)
    {
        auto_pressure_enabled = false;
        Valve2SetState(false);
        (void)UART_AppSendStringAsync("ACK,VALVE,2,OFF\r\n");
    }
    else if (strncmp(command, "PWM,", 4) == 0)
    {
        char *end1 = NULL;
        long id = strtol(command + 4, &end1, 10);
        if ((end1 != (command + 4)) && (*end1 == ','))
        {
            char *end2 = NULL;
            long duty = strtol(end1 + 1, &end2, 10);
            if ((end2 != (end1 + 1)) && (*end2 == '\0') && ((id == 1L) || (id == 2L)))
            {
                if (duty < 0L) duty = 0L;
                if (duty > (long)PWM_MAX_COMPARE) duty = (long)PWM_MAX_COMPARE;
                auto_pressure_enabled = false;
                ValveSetPwm((uint8_t)id, (uint16_t)duty);
                (void)UART_AppPrintf("ACK,PWM,%ld,%ld\r\n", id, duty);
                return;
            }
        }
        (void)UART_AppPrintf("ERROR,INVALID_PWM,%s\r\n", command);
    }
    else if (strcmp(command, "TARGET,OFF") == 0)
    {
        auto_pressure_enabled = false;
        Valve1SetState(false);
        Valve2SetState(false);
        (void)UART_AppSendStringAsync("ACK,TARGET,OFF\r\n");
    }
    else if (strncmp(command, "TARGET,", 7) == 0)
    {
        char *end = NULL;
        float req_kpa = strtof(command + 7, &end);
        if ((end != (command + 7)) && (*end == '\0'))
        {
            if (req_kpa < 0.0f) req_kpa = 0.0f;
            if (req_kpa > max_safe_pressure_kpa) req_kpa = max_safe_pressure_kpa;
            target_pressure_kpa = req_kpa;
            auto_pressure_enabled = true;
            (void)UART_AppPrintf("ACK,TARGET,%.2f\r\n", target_pressure_kpa);
        }
        else
        {
            (void)UART_AppPrintf("ERROR,INVALID_TARGET,%s\r\n", command);
        }
    }
    else if (strcmp(command, "ZERO") == 0)
    {
        PressureSensor_t *s0 = PressureSensor_Get(&pressure_manager, 0);
        PressureSensor_t *s1 = PressureSensor_Get(&pressure_manager, 1);
        if ((s0 != NULL) && (s0->connected != 0U)) s0_zero_offset_kpa = s0->pressure;
        if ((s1 != NULL) && (s1->connected != 0U)) s1_zero_offset_kpa = s1->pressure;
        (void)UART_AppPrintf("ACK,ZERO,%.4f,%.4f\r\n", s0_zero_offset_kpa, s1_zero_offset_kpa);
    }
    else if (strcmp(command, "ZERO,RESET") == 0)
    {
        s0_zero_offset_kpa = 0.0f;
        s1_zero_offset_kpa = 0.0f;
        (void)UART_AppSendStringAsync("ACK,ZERO,RESET\r\n");
    }
    else if (strncmp(command, "MAXP,", 5) == 0)
    {
        char *end = NULL;
        float limit = strtof(command + 5, &end);
        if ((end != (command + 5)) && (*end == '\0') && (limit > 0.0f) && (limit <= ABSOLUTE_MAX_PRESSURE_KPA))
        {
            max_safe_pressure_kpa = limit;
            if (target_pressure_kpa > max_safe_pressure_kpa)
            {
                target_pressure_kpa = max_safe_pressure_kpa;
            }
            (void)UART_AppPrintf("ACK,MAXP,%.2f\r\n", max_safe_pressure_kpa);
        }
        else
        {
            (void)UART_AppPrintf("ERROR,INVALID_MAXP,%s\r\n", command);
        }
    }
    else if (strcmp(command, "STATUS") == 0)
    {
        SendStatus();
    }
    else
    {
        (void)UART_AppPrintf("ERROR,UNKNOWN_COMMAND,%s\r\n", command);
    }
}

static void ValveSetPwm(uint8_t valve_id, uint16_t duty)
{
    if (duty > PWM_MAX_COMPARE)
    {
        duty = PWM_MAX_COMPARE;
    }

    if (valve_id == 1U)
    {
        valve1_pwm = duty;
        valve1_state = (duty > 0U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, valve1_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else if (valve_id == 2U)
    {
        valve2_pwm = duty;
        valve2_state = (duty > 0U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty);
    }
}

// 1. MOSFET Kontrolü (PA0 / A0 Yuvası -> TIM2_CH1)
static void Valve1SetState(bool enabled)
{
    ValveSetPwm(1U, enabled ? PWM_MAX_COMPARE : 0U);
}

// 2. MOSFET Kontrolü (PA1 / A1 Yuvası -> TIM2_CH2)
static void Valve2SetState(bool enabled)
{
    ValveSetPwm(2U, enabled ? PWM_MAX_COMPARE : 0U);
}

static void UpdateClosedLoopPressure(float p1_kpa, bool s1_connected)
{
    if (!s1_connected)
    {
        auto_pressure_enabled = false;
        Valve1SetState(false);
        Valve2SetState(false);
        (void)UART_AppSendStringAsync("ERROR,TARGET_ABORT_NO_SENSOR\r\n");
        return;
    }

    if (p1_kpa < (target_pressure_kpa - TARGET_HYSTERESIS_KPA))
    {
        Valve1SetState(true);
        Valve2SetState(false);
    }
    else if (p1_kpa > (target_pressure_kpa + TARGET_HYSTERESIS_KPA))
    {
        Valve1SetState(false);
        Valve2SetState(true);
    }
    else
    {
        Valve1SetState(false);
        Valve2SetState(false);
    }
}

static void ApplyOverpressureSafety(float p1_kpa, bool s1_connected)
{
    if (!s1_connected) return;

    if (p1_kpa > max_safe_pressure_kpa)
    {
        auto_pressure_enabled = false;
        Valve1SetState(false);
        Valve2SetState(true);
        if (!overpressure_tripped)
        {
            overpressure_tripped = true;
            (void)UART_AppPrintf("WARN,OVERPRESSURE,%.2f\r\n", p1_kpa);
        }
    }
    else if (overpressure_tripped && (p1_kpa <= (max_safe_pressure_kpa - TARGET_HYSTERESIS_KPA)))
    {
        overpressure_tripped = false;
        Valve2SetState(false);
    }
}

static void SendStatus(void)
{
    (void)UART_AppPrintf("STATUS,VALVE1=%u,VALVE2=%u,PWM1=%u,PWM2=%u,TARGET=%.2f,AUTO=%u,MAXP=%.2f\r\n",
                         valve1_state ? 1U : 0U,
                         valve2_state ? 1U : 0U,
                         valve1_pwm,
                         valve2_pwm,
                         target_pressure_kpa,
                         auto_pressure_enabled ? 1U : 0U,
                         max_safe_pressure_kpa);
}

/* =========================================================
 * DONANIM (GPIO, TIM2, SPI2, UART1) BAŞLATMA FONKSİYONLARI
 * ========================================================= */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    LED2_GPIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Kart üzerindeki bildirim LED'i (PA5)
    GPIO_InitStruct.Pin = LED2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);

    // PWM Çıkışları: PA0 (TIM2_CH1 -> 1. MOSFET) ve PA1 (TIM2_CH2 -> 2. MOSFET)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void TIM2_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    // 160 MHz / ((159 + 1) * (999 + 1)) = 1 kHz PWM Frekansı
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 159;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

static void USART1_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;

    // 160 MHz / 256 = 625 kHz (Honeywell ABP2 50-800 kHz sınırı içinde güvenli hız)
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;

    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;

#if defined(SPI_NSS_PULSE_DISABLE)
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
#endif
#if defined(SPI_NSS_POLARITY_LOW)
    hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
#endif
#if defined(SPI_FIFO_THRESHOLD_01DATA)
    hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
#endif
#if defined(SPI_MASTER_SS_IDLENESS_00CYCLE)
    hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
#endif
#if defined(SPI_MASTER_INTERDATA_IDLENESS_00CYCLE)
    hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
#endif
#if defined(SPI_MASTER_RX_AUTOSUSP_DISABLE)
    hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
#endif
#if defined(SPI_MASTER_KEEP_IO_STATE_ENABLE)
    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
#endif
#if defined(SPI_IO_SWAP_DISABLE)
    hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
#endif

    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hspi->Instance == SPI2)
    {
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        // PB13 -> SPI2_SCK, PB14 -> SPI2_MISO, PB15 -> SPI2_MOSI
        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI2_IRQn);
    }
}

static void USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200U;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void USART1_DMA_RX_Init(void)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    hdma_uart1_rx.Instance = GPDMA1_Channel0;
    hdma_uart1_rx.Init.Request = GPDMA1_REQUEST_USART1_RX;
    hdma_uart1_rx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    hdma_uart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart1_rx.Init.SrcInc = DMA_SINC_FIXED;
    hdma_uart1_rx.Init.DestInc = DMA_DINC_INCREMENTED;
    hdma_uart1_rx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    hdma_uart1_rx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    hdma_uart1_rx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    hdma_uart1_rx.Init.SrcBurstLength = 1U;
    hdma_uart1_rx.Init.DestBurstLength = 1U;
    hdma_uart1_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
    hdma_uart1_rx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    hdma_uart1_rx.Init.Mode = DMA_NORMAL;

    if (HAL_DMA_Init(&hdma_uart1_rx) != HAL_OK) Error_Handler();
    if (HAL_DMA_ConfigChannelAttributes(&hdma_uart1_rx, DMA_CHANNEL_NPRIV) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(&huart1, hdmarx, hdma_uart1_rx);
}

static void USART1_DMA_TX_Init(void)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    hdma_uart1_tx.Instance = GPDMA1_Channel1;
    hdma_uart1_tx.Init.Request = GPDMA1_REQUEST_USART1_TX;
    hdma_uart1_tx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    hdma_uart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart1_tx.Init.SrcInc = DMA_SINC_INCREMENTED;
    hdma_uart1_tx.Init.DestInc = DMA_DINC_FIXED;
    hdma_uart1_tx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    hdma_uart1_tx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    hdma_uart1_tx.Init.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
    hdma_uart1_tx.Init.SrcBurstLength = 1U;
    hdma_uart1_tx.Init.DestBurstLength = 1U;
    hdma_uart1_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
    hdma_uart1_tx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    hdma_uart1_tx.Init.Mode = DMA_NORMAL;

    if (HAL_DMA_Init(&hdma_uart1_tx) != HAL_OK) Error_Handler();
    if (HAL_DMA_ConfigChannelAttributes(&hdma_uart1_tx, DMA_CHANNEL_NPRIV) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(&huart1, hdmatx, hdma_uart1_tx);
}

static void USART1_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void GPDMA1_Channel0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_uart1_rx); }
void GPDMA1_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_uart1_tx); }
void USART1_IRQHandler(void)          { HAL_UART_IRQHandler(&huart1); }
void SPI2_IRQHandler(void)            { HAL_SPI_IRQHandler(&hspi2); }

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    PressureSensor_TxCpltCallback(&pressure_manager, hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    PressureSensor_TxRxCpltCallback(&pressure_manager, hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        for (uint8_t i = 0U; i < NUMBER_OF_PRESSURE_SENSORS; i++)
        {
            HAL_GPIO_WritePin(pressure_sensors[i].cs_port, pressure_sensors[i].cs_pin, GPIO_PIN_SET);
        }
    }
}

#include "handlers/impl/default.h"
#include "clock_config/impl/hal/default.h"
#include "power_config/impl/hal/default.h"
#include "cache_enable/impl/hal/default.h"
#include "hal/impl/timebase_tim6.h"
#include "msp/global_default.h"
#include "msp/uart_generic.h"
#include "msp/uart1_default.h"