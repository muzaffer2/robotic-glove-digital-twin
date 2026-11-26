//Kadir Ozlem

#ifndef UART_APP_H
#define UART_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/*
 * RX configuration
 */
#define UART_RX_DMA_BUFFER_SIZE       128U
#define UART_RX_LINE_BUFFER_SIZE      128U

/*
 * Command queue configuration.
 *
 * A ring buffer with N slots can store at most N - 1 commands.
 */
#define UART_COMMAND_QUEUE_LENGTH     8U
#define UART_COMMAND_MAX_LENGTH       128U

/*
 * TX queue configuration.
 *
 * A ring buffer with N slots can store at most N - 1 packets.
 */
#define UART_TX_QUEUE_LENGTH          16U
#define UART_TX_PACKET_MAX_LENGTH     256U

typedef enum
{
    UART_APP_OK = 0,
    UART_APP_ERROR,
    UART_APP_INVALID_ARGUMENT,
    UART_APP_QUEUE_FULL,
    UART_APP_DATA_TOO_LONG
} UART_AppStatus_t;

typedef struct
{
    uint32_t rx_event_count;
    uint32_t rx_byte_count;
    uint32_t rx_line_overflow_count;
    uint32_t command_drop_count;
    uint32_t tx_packet_count;
    uint32_t tx_drop_count;
    uint32_t uart_error_count;
} UART_AppStatistics_t;

/**
 * @brief Initializes the UART application layer and starts RX DMA.
 *
 * @return UART application status.
 */
UART_AppStatus_t UART_AppInit(void);

/**
 * @brief Restarts the Receive-to-Idle DMA operation.
 *
 * This function normally does not need to be called by the application.
 *
 * @return UART application status.
 */
UART_AppStatus_t UART_AppStartReception(void);

/**
 * @brief Adds binary data to the asynchronous TX queue.
 *
 * The input data is copied into the internal queue. Therefore, the caller
 * may safely reuse or release its buffer after this function returns.
 *
 * @param data Pointer to the data to transmit.
 * @param length Number of bytes to transmit.
 *
 * @return UART application status.
 */
UART_AppStatus_t UART_AppSendAsync(
    const uint8_t *data,
    uint16_t length
);

/**
 * @brief Adds a null-terminated string to the asynchronous TX queue.
 *
 * @param text Null-terminated string.
 *
 * @return UART application status.
 */
UART_AppStatus_t UART_AppSendStringAsync(
    const char *text
);

/**
 * @brief Formats a string and adds it to the asynchronous TX queue.
 *
 * The formatted output must fit into UART_TX_PACKET_MAX_LENGTH bytes.
 *
 * @param format printf-style format string.
 *
 * @return UART application status.
 */
UART_AppStatus_t UART_AppPrintf(
    const char *format,
    ...
);

/**
 * @brief Checks whether at least one complete command is available.
 *
 * @return true if a command is available, otherwise false.
 */
bool UART_AppCommandAvailable(void);

/**
 * @brief Retrieves the next complete command from the RX queue.
 *
 * The line terminator is not included in the returned command.
 *
 * @param destination Destination buffer.
 * @param destination_size Size of the destination buffer.
 *
 * @return true if a command was retrieved, otherwise false.
 */
bool UART_AppGetCommand(
    char *destination,
    uint16_t destination_size
);

/**
 * @brief Returns true while a TX DMA transfer is active.
 */
bool UART_AppTxBusy(void);

/**
 * @brief Returns the current UART statistics.
 *
 * @param statistics Destination statistics structure.
 */
void UART_AppGetStatistics(
    UART_AppStatistics_t *statistics
);

/**
 * @brief Handles the HAL Receive-to-Idle event.
 *
 * Call this function from HAL_UARTEx_RxEventCallback().
 */
void UART_AppRxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t size
);

/**
 * @brief Handles the HAL TX complete callback.
 *
 * Call this function from HAL_UART_TxCpltCallback().
 */
void UART_AppTxCompleteCallback(
    UART_HandleTypeDef *huart
);

/**
 * @brief Handles the HAL UART error callback.
 *
 * Call this function from HAL_UART_ErrorCallback().
 */
void UART_AppErrorCallback(
    UART_HandleTypeDef *huart
);

#ifdef __cplusplus
}
#endif

#endif /* UART_APP_H */