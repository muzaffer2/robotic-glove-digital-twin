#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H



#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_spi.h"
#include <stdint.h>
#include <stddef.h>


/* =========================================================
 * ABP2 CONSTANTS (T150PGSA3XX: 0-150 psi Gage, SPI, 10-90%, 3.3V)
 * ========================================================= */

#define PRESSURE_SENSOR_OUTPUT_MIN       1677722.0f
#define PRESSURE_SENSOR_OUTPUT_MAX      15099494.0f

#define PRESSURE_SENSOR_P_MIN                  0.0f
#define PRESSURE_SENSOR_P_MAX               1034.2136f  // 150 psi = 1034.2136 kPa (Gage)

#define PRESSURE_SENSOR_FULL_SCALE      16777215.0f


/* =========================================================
 * SENSOR DATA
 * ========================================================= */

typedef struct
{
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    uint8_t rx_data[7];

    uint8_t status;

    uint32_t pressure_counts;
    uint32_t temperature_counts;

    float pressure;
    float temperature;
    float percentage;

    uint8_t connected;
    uint32_t last_probe_time;

} PressureSensor_t;


/* =========================================================
 * STATE MACHINE
 * ========================================================= */

typedef enum
{
    PRESSURE_STATE_IDLE = 0,

    PRESSURE_STATE_START_COMMAND,
    PRESSURE_STATE_WAIT_COMMAND,

    PRESSURE_STATE_WAIT_CONVERSION,

    PRESSURE_STATE_START_READ,
    PRESSURE_STATE_WAIT_READ

} PressureSensor_State_t;


/* =========================================================
 * SENSOR MANAGER
 * ========================================================= */

typedef struct
{
    SPI_HandleTypeDef *hspi;

    PressureSensor_t *sensors;
    uint8_t sensor_count;

    volatile uint8_t current_sensor;
    volatile PressureSensor_State_t state;

    uint32_t sample_period_ms;
    uint32_t conversion_wait_ms;

    uint32_t sample_start_time;
    uint32_t conversion_start_time;

    uint32_t disconnected_probe_period_ms;

    volatile uint8_t new_data_available;

    uint32_t active_sensor_mask;

} PressureSensorManager_t;


/* =========================================================
 * SPI BUFFERS
 * ========================================================= */

/*
 * ABP2 Output Measurement Command
 */
static uint8_t pressure_sensor_measurement_command[3] =
{
    0xAA,
    0x00,
    0x00
};


/*
 * NOP + dummy bytes.
 *
 * SPI is full duplex.
 * Sending these bytes generates the clock required
 * to receive Status + Pressure + Temperature.
 */
static uint8_t pressure_sensor_read_command[7] =
{
    0xF0,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
};


/* =========================================================
 * CHIP SELECT
 * ========================================================= */

static inline void PressureSensor_CS_Low(
        PressureSensorManager_t *manager,
        uint8_t index)
{
    HAL_GPIO_WritePin(
            manager->sensors[index].cs_port,
            manager->sensors[index].cs_pin,
            GPIO_PIN_RESET);
}


static inline void PressureSensor_CS_High(
        PressureSensorManager_t *manager,
        uint8_t index)
{
    HAL_GPIO_WritePin(
            manager->sensors[index].cs_port,
            manager->sensors[index].cs_pin,
            GPIO_PIN_SET);
}

/* =========================================================
 * Find next available sensor
 * ========================================================= */

static inline uint8_t PressureSensor_FindNextActiveSensor(
        PressureSensorManager_t *manager,
        uint8_t start_index)
{
    for (uint8_t i = start_index;
         i < manager->sensor_count;
         i++)
    {
        if ((manager->active_sensor_mask &
             (1UL << i)) != 0U)
        {
            return i;
        }
    }

    return manager->sensor_count;
}


/* =========================================================
 * Check The Data is Valid
 * ========================================================= */


static inline uint8_t PressureSensor_IsResponseValid(
        PressureSensor_t *sensor)
{
    uint8_t all_zero = 1U;
    uint8_t all_ff   = 1U;

    for (uint8_t i = 0U; i < sizeof(sensor->rx_data); i++)
    {
        if (sensor->rx_data[i] != 0x00U)
        {
            all_zero = 0U;
        }

        if (sensor->rx_data[i] != 0xFFU)
        {
            all_ff = 0U;
        }
    }

    if ((all_zero != 0U) || (all_ff != 0U))
    {
        return 0U;
    }

    return 1U;
}


/* =========================================================
 * PROCESS RAW DATA
 * ========================================================= */

static inline void PressureSensor_ProcessData(
        PressureSensorManager_t *manager,
        uint8_t index)
{
    PressureSensor_t *sensor =
            &manager->sensors[index];

    uint8_t *data =
            sensor->rx_data;


    /* -----------------------------------------------------
     * Status Byte
     * ----------------------------------------------------- */

    sensor->status = data[0];


    /* -----------------------------------------------------
     * 24-bit Pressure
     *
     * Byte 1 = MSB
     * Byte 2
     * Byte 3 = LSB
     * ----------------------------------------------------- */

    sensor->pressure_counts =
            ((uint32_t)data[1] << 16) |
            ((uint32_t)data[2] << 8)  |
            ((uint32_t)data[3]);


    /* -----------------------------------------------------
     * 24-bit Temperature
     *
     * Byte 4 = MSB
     * Byte 5
     * Byte 6 = LSB
     * ----------------------------------------------------- */

    sensor->temperature_counts =
            ((uint32_t)data[4] << 16) |
            ((uint32_t)data[5] << 8)  |
            ((uint32_t)data[6]);


    /* -----------------------------------------------------
     * Percentage of full 24-bit range
     * ----------------------------------------------------- */

    sensor->percentage =
            ((float)sensor->pressure_counts /
             PRESSURE_SENSOR_FULL_SCALE)
            * 100.0f;


    /* -----------------------------------------------------
     * Pressure conversion
     * ----------------------------------------------------- */

    sensor->pressure =
            (((float)sensor->pressure_counts -
              PRESSURE_SENSOR_OUTPUT_MIN)
             *
             (PRESSURE_SENSOR_P_MAX -
              PRESSURE_SENSOR_P_MIN))
            /
            (PRESSURE_SENSOR_OUTPUT_MAX -
             PRESSURE_SENSOR_OUTPUT_MIN)
            +
            PRESSURE_SENSOR_P_MIN;


    /* -----------------------------------------------------
     * Temperature conversion
     * ----------------------------------------------------- */

    sensor->temperature =
            ((float)sensor->temperature_counts *
             200.0f /
             PRESSURE_SENSOR_FULL_SCALE)
            - 50.0f;
}

static inline void PressureSensor_EnableGPIOClock(
        GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
#ifdef GPIOD
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
#endif
#ifdef GPIOE
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
#endif
#ifdef GPIOF
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
#endif
#ifdef GPIOG
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
#endif
#ifdef GPIOH
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
#endif
}

static inline void PressureSensor_InitCSPins(
        PressureSensorManager_t *manager)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    for (uint8_t i = 0U;
         i < manager->sensor_count;
         i++)
    {
        GPIO_TypeDef *port =
                manager->sensors[i].cs_port;

        uint16_t pin =
                manager->sensors[i].cs_pin;


        /*
         * Enable GPIO peripheral clock
         */
        PressureSensor_EnableGPIOClock(port);


        /*
         * CS is active LOW.
         * Set inactive state before GPIO initialization.
         */
        HAL_GPIO_WritePin(
                port,
                pin,
                GPIO_PIN_SET);


        /*
         * Configure CS pin as GPIO output
         */
        GPIO_InitStruct.Pin = pin;

        HAL_GPIO_Init(
                port,
                &GPIO_InitStruct);
    }
}


/* =========================================================
 * INITIALIZATION
 * ========================================================= */
static inline void PressureSensor_Init(
        PressureSensorManager_t *manager,
        SPI_HandleTypeDef *hspi,
        PressureSensor_t *sensors,
        uint8_t sensor_count,
        uint32_t sample_period_ms,
        uint32_t conversion_wait_ms)
{
    if ((manager == NULL) ||
        (hspi == NULL) ||
        (sensors == NULL) ||
        (sensor_count == 0U))
    {
        return;
    }


    manager->hspi = hspi;

    manager->sensors = sensors;
    manager->sensor_count = sensor_count;

    manager->sample_period_ms =
            sample_period_ms;

    manager->conversion_wait_ms =
            conversion_wait_ms;
        
    manager->disconnected_probe_period_ms =
        2000U;

    manager->current_sensor = 0U;

    manager->conversion_start_time = 0U;

    manager->new_data_available = 0U;

    for (uint8_t i = 0U; i < sensor_count; i++)
    {
        sensors[i].connected = 1U;
        sensors[i].last_probe_time = 0U;
    }


    /*
     * Automatically configure all CS GPIO pins
     * defined inside the sensor array.
     */
    PressureSensor_InitCSPins(manager);


    /*
     * Start first measurement immediately.
     */
    manager->sample_start_time =
            HAL_GetTick() - sample_period_ms;


    manager->state =
            PRESSURE_STATE_IDLE;
}


/* =========================================================
 * MAIN NON-BLOCKING TASK
 * ========================================================= */

static inline void PressureSensor_Task(
        PressureSensorManager_t *manager)
{

    if ((manager == NULL) ||
        (manager->sensor_count == 0U))
    {
        return;
    }

    switch (manager->state)
    {

        /* =================================================
         * IDLE
         *
         * Wait until next sampling period.
         * ================================================= */
        case PRESSURE_STATE_IDLE:
        {
            uint32_t now = HAL_GetTick();

            if ((uint32_t)(now -
                        manager->sample_start_time)
                >= manager->sample_period_ms)
            {
                manager->sample_start_time = now;

                /*
                * Build active sensor list for this cycle.
                *
                * This mask MUST remain unchanged during
                * COMMAND -> WAIT_CONVERSION -> READ.
                */
                manager->active_sensor_mask = 0U;

                for (uint8_t i = 0U;
                    i < manager->sensor_count;
                    i++)
                {
                    if (manager->sensors[i].connected != 0U)
                    {
                        /*
                        * Connected sensors are always active.
                        */
                        manager->active_sensor_mask |=
                                (1UL << i);
                    }
                    else
                    {
                        /*
                        * Disconnected sensors are only
                        * periodically probed.
                        */
                        if ((uint32_t)(
                                now -
                                manager->sensors[i].last_probe_time)
                            >= manager->disconnected_probe_period_ms)
                        {
                            manager->active_sensor_mask |=
                                    (1UL << i);

                            manager->sensors[i].last_probe_time =
                                    now;
                        }
                    }
                }

                manager->new_data_available = 0U;


                /*
                * Find first active sensor.
                */
                manager->current_sensor =
                        PressureSensor_FindNextActiveSensor(
                                manager,
                                0U);


                /*
                * At least one sensor selected?
                */
                if (manager->current_sensor <
                    manager->sensor_count)
                {
                    manager->state =
                            PRESSURE_STATE_START_COMMAND;
                }
                else
                {
                    /*
                    * No sensor needs to be accessed
                    * during this cycle.
                    */
                    manager->state =
                            PRESSURE_STATE_IDLE;
                }
            }

            break;
        }
        /* =================================================
         * START MEASUREMENT COMMAND
         * ================================================= */

        case PRESSURE_STATE_START_COMMAND:
        {
            DEBUG_INFO(
        " %d : PRESSURE_STATE_START_COMMAND\r\n",
        manager->current_sensor);
            uint8_t index =
                    manager->current_sensor;


            PressureSensor_CS_Low(
                    manager,
                    index);
            
            HAL_StatusTypeDef ret =HAL_SPI_Transmit_IT(
                    manager->hspi,
                    pressure_sensor_measurement_command,
                    sizeof(pressure_sensor_measurement_command));
            if (ret
                    == HAL_OK)
            {
                manager->state =
                        PRESSURE_STATE_WAIT_COMMAND;
            }
            else
            {
                /*
                 * SPI is busy.
                 *
                 * Release CS and try again
                 * during next task iteration.
                 */
                PressureSensor_CS_High(
                        manager,
                        index);
            }
            break;
        }


        /* =================================================
         * WAIT FOR COMMAND TRANSMISSION
         * ================================================= */

        case PRESSURE_STATE_WAIT_COMMAND:
        {
            DEBUG_INFO(" %d : PRESSURE_STATE_WAIT_COMMAND\n",manager->current_sensor);

            /*
             * Nothing here.
             *
             * State is advanced from
             * PressureSensor_TxCpltCallback().
             */
            break;
        }


        /* =================================================
         * WAIT FOR SENSOR CONVERSION
         * ================================================= */

        case PRESSURE_STATE_WAIT_CONVERSION:
        {
            if ((uint32_t)(HAL_GetTick() -
                           manager->conversion_start_time)
                >= manager->conversion_wait_ms)
            {
                manager->current_sensor =
                PressureSensor_FindNextActiveSensor(
                        manager,
                        0U);

                manager->state =
                        PRESSURE_STATE_START_READ;
            }

            break;
        }


        /* =================================================
         * START SENSOR READ
         * ================================================= */

        case PRESSURE_STATE_START_READ:
        {
            DEBUG_INFO(" %d : PRESSURE_STATE_START_READ\n",manager->current_sensor);
            
            uint8_t index =
                    manager->current_sensor;


            PressureSensor_CS_Low(
                    manager,
                    index);


            if (HAL_SPI_TransmitReceive_IT(
                    manager->hspi,
                    pressure_sensor_read_command,
                    manager->sensors[index].rx_data,
                    sizeof(pressure_sensor_read_command))
                    == HAL_OK)
            {
                manager->state =
                        PRESSURE_STATE_WAIT_READ;
            }
            else
            {
                /*
                 * SPI is busy.
                 */
                PressureSensor_CS_High(
                        manager,
                        index);
            }

            break;
        }


        /* =================================================
         * WAIT FOR SENSOR READ
         * ================================================= */

        case PRESSURE_STATE_WAIT_READ:
        {
            DEBUG_INFO(" %d : PRESSURE_STATE_WAIT_READ\n",manager->current_sensor);

            /*
             * Nothing here.
             *
             * State is advanced from
             * PressureSensor_TxRxCpltCallback().
             */

            break;
        }


        /* =================================================
         * DEFAULT
         * ================================================= */

        default:
        {
            DEBUG_INFO(" %d : PRESSURE_DEFAULT\n",manager->current_sensor);

            manager->current_sensor = 0U;

            manager->state =
                    PRESSURE_STATE_IDLE;

            break;
        }
    }
}


/* =========================================================
 * SPI TRANSMIT COMPLETE CALLBACK
 *
 * Called after measurement command has been sent.
 * ========================================================= */

static inline void PressureSensor_TxCpltCallback(
        PressureSensorManager_t *manager,
        SPI_HandleTypeDef *hspi)
{
    if ((manager == NULL) ||
        (hspi != manager->hspi))
    {
        return;
    }


    /*
     * Ignore unrelated SPI TX callbacks.
     */
    if (manager->state !=
        PRESSURE_STATE_WAIT_COMMAND)
    {
        return;
    }


    uint8_t index =
            manager->current_sensor;


    /*
     * Release current sensor.
     */
    PressureSensor_CS_High(
            manager,
            index);


    /*
 * Select next ACTIVE sensor.
 */
    manager->current_sensor =
            PressureSensor_FindNextActiveSensor(
                    manager,
                    index + 1U);


    /*
    * All active sensors received measurement command?
    */
    if (manager->current_sensor >=
        manager->sensor_count)
    {
        /*
        * Conversion timer starts after the
        * last active sensor receives command.
        */
        manager->conversion_start_time =
                HAL_GetTick();

        manager->state =
                PRESSURE_STATE_WAIT_CONVERSION;
    }
    else
    {
        manager->state =
                PRESSURE_STATE_START_COMMAND;
    }
}


/* =========================================================
 * SPI TRANSMIT/RECEIVE COMPLETE CALLBACK
 *
 * Called after Status + Pressure + Temperature
 * have been received.
 * ========================================================= */

static inline void PressureSensor_TxRxCpltCallback(
        PressureSensorManager_t *manager,
        SPI_HandleTypeDef *hspi)
{
    if ((manager == NULL) ||
        (hspi != manager->hspi))
    {
        return;
    }


    /*
     * Ignore unrelated SPI callbacks.
     */
    if (manager->state !=
        PRESSURE_STATE_WAIT_READ)
    {
        return;
    }


    uint8_t index =
            manager->current_sensor;

    PressureSensor_t *sensor =
            &manager->sensors[index];


    /*
     * Release current sensor.
     */
    PressureSensor_CS_High(
            manager,
            index);


    /*
     * Check received SPI data.
     */
    if (PressureSensor_IsResponseValid(sensor) != 0U)
    {
        /*
         * Sensor has returned valid data.
         */

        if (sensor->connected == 0U)
        {
            DEBUG_INFO(
                    "Sensor %d CONNECTED\r\n",
                    index);
        }

        sensor->connected = 1U;


        /*
         * Convert raw data only if
         * received data is valid.
         */
        PressureSensor_ProcessData(
                manager,
                index);
    }
    else
    {
        /*
         * Invalid response.
         *
         * Typical disconnected sensor:
         *
         * 00 00 00 00 00 00 00
         *
         * or
         *
         * FF FF FF FF FF FF FF
         */

        if (sensor->connected != 0U)
        {
            DEBUG_ERROR(
                    "Sensor %d DISCONNECTED\r\n",
                    index);
        }

        sensor->connected = 0U;

        sensor->last_probe_time =
                HAL_GetTick();
    }


    /*
    * Move to next ACTIVE sensor.
    */
    manager->current_sensor =
            PressureSensor_FindNextActiveSensor(
                    manager,
                    index + 1U);


    /*
    * All active sensors read?
    */
    if (manager->current_sensor >=
        manager->sensor_count)
    {
        manager->current_sensor = 0U;

        manager->new_data_available = 1U;

        manager->state =
                PRESSURE_STATE_IDLE;
    }
    else
    {
        manager->state =
                PRESSURE_STATE_START_READ;
    }
}
/* =========================================================
 * GET SENSOR
 * ========================================================= */

static inline PressureSensor_t *
PressureSensor_Get(
        PressureSensorManager_t *manager,
        uint8_t index)
{
    if ((manager == NULL) ||
        (index >= manager->sensor_count))
    {
        return NULL;
    }


    return &manager->sensors[index];
}


/* =========================================================
 * CHECK NEW DATA
 * ========================================================= */

static inline uint8_t PressureSensor_NewDataAvailable(
        PressureSensorManager_t *manager)
{
    if (manager == NULL)
    {
        return 0U;
    }

    return manager->new_data_available;
}


/* =========================================================
 * CLEAR NEW DATA FLAG
 * ========================================================= */

static inline void PressureSensor_ClearNewDataFlag(
        PressureSensorManager_t *manager)
{
    if (manager != NULL)
    {
        manager->new_data_available = 0U;
    }
}


#endif /* PRESSURE_SENSOR_H */