#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     GPIO_NUM_17
#define UART_RX_PIN     GPIO_NUM_16
#define BUF_SIZE        256
#define BAUD_RATE       115200

static const char *PING_MSG = "PING\n";
static const char *PONG_MSG = "PONG\n";

static volatile bool pong_received = false;
static TickType_t last_ping_time = 0; // timestamp del último PING enviado

/*
 * UART RX Task
*/
static void uart_rx_task(void *arg)
{
    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            printf("RX: %s", data);

            if (strncmp((char*)data, "PING", 4) == 0) {
                uart_write_bytes(UART_PORT, PONG_MSG, strlen(PONG_MSG));
                printf("TX: PONG\n");
            } 
            else if (strncmp((char*)data, "PONG", 4) == 0) {
                pong_received = true;
                printf("✔ Comunicación OK (PONG recibido)\n");
            }
        }
    }
}

/*
 * UART TX Task con timeout
*/
static void uart_tx_task(void *arg)
{
    const TickType_t timeout_ticks = pdMS_TO_TICKS(2000); // 2s
    last_ping_time = xTaskGetTickCount();

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Si nunca recibimos PONG y pasó el timeout → reenvío
        if (!pong_received && (now - last_ping_time >= timeout_ticks)) {
            uart_write_bytes(UART_PORT, PING_MSG, strlen(PING_MSG));
            printf("TX: PING (reintento)\n");
            last_ping_time = now;
        }

        // Si ya recibimos PONG → mandar otro Ping cada 1s para seguir la prueba
        if (pong_received) {
            uart_write_bytes(UART_PORT, PING_MSG, strlen(PING_MSG));
            printf("TX: PING\n");
            pong_received = false; // Esperamos el próximo PONG
            last_ping_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    uart_config_t config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, 4, NULL);
}
