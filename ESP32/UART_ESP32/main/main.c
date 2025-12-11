#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

/** UART **/
#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     GPIO_NUM_17
#define UART_RX_PIN     GPIO_NUM_16
#define BUF_SIZE        256
#define BAUD_RATE       115200

static const char *PING_MSG = "PING\n";
static const char *PONG_MSG = "PONG\n";

static volatile bool pong_received = false;
static TickType_t last_ping_time = 0; // timestamp del último PING enviado

/** I2C MPU **/
#define I2C_SDA         GPIO_NUM_21
#define I2C_SCL         GPIO_NUM_22
#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     100000

#define MPU_ADDR        0x68
#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43
#define GYRO_CONFIG     0x1B


/* rad->deg */
#define RAD2DEG (180.0 / M_PI)

/* Variables globales de orientación */
static float yaw = 0.0f;
static float yaw_acc = 0.0f;

/*
 * I2C Init
*/
void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };

    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

/*
 * Escritura en MPU6050
*/
esp_err_t mpu_write(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * Lectura desde MPU6050
*/
esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Seleccionar registro
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // Leer datos
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * Leer acelerómetro y calcular pitch y roll
*/
void mpu_read_accel(float *pitch, float *roll)
{
    uint8_t data[6];
    if (mpu_read(ACCEL_XOUT_H, data, 6) != ESP_OK) return;

    int16_t ax_raw = (data[0] << 8) | data[1];
    int16_t ay_raw = (data[2] << 8) | data[3];
    int16_t az_raw = (data[4] << 8) | data[5];

    float Ax = ax_raw / 16384.0f;
    float Ay = ay_raw / 16384.0f;
    float Az = az_raw / 16384.0f;

    *pitch = atan2f(Ax, sqrtf(Ay*Ay + Az*Az)) * RAD2DEG;
    *roll  = atan2f(Ay, sqrtf(Ax*Ax + Az*Az)) * RAD2DEG;
}

/*
 * MPU6050 Init
*/
void mpu_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    // Salir del modo sleep
    mpu_write(PWR_MGMT_1, 0x00);

    // Configurar rango del giroscopio
    // 0x00 → ±250 °/s
    // 0x08 → ±500 °/s
    // 0x10 → ±1000 °/s
    // 0x18 → ±2000 °/s
    mpu_write(GYRO_CONFIG, 0x08); // O subir a 0x10
}

static void mpu_task(void *arg)
{
    uint8_t gyro_raw[2];
    int16_t gz_raw;
    float gz_dps;

    float pitch = 0, roll = 0;

    TickType_t last = xTaskGetTickCount();
    const TickType_t dt_ticks = pdMS_TO_TICKS(10);
    const float dt = 0.010f;

    while (1)
    {
        /* Leer acelerómetro */
        mpu_read_accel(&pitch, &roll);

        /* Leer giroscopio (solo Z = yaw) */
        if (mpu_read(GYRO_XOUT_H + 4, gyro_raw, 2) == ESP_OK) {

            gz_raw = (gyro_raw[0] << 8) | gyro_raw[1];
            gz_dps = gz_raw / 65.5f; // ±500 °/s escala

            // Integración del giro
            yaw += gz_dps * dt;

            // Normalización
            if (yaw >= 360) yaw -= 360;
            if (yaw < 0)   yaw += 360;

            printf("[MPU] yaw=%.2f° | gz=%.2f °/s | pitch=%.2f | roll=%.2f\n",
                    yaw, gz_dps, pitch, roll);

            // UTILIZAR VALOR DE YAW PARA DETERMINAR HACIA DÓNDE APUNTA EL ROBOT, LUEGO CON ESO DETERMINO LA DIRECCIÓN A SEGUIR
        }
        else {
            printf("[MPU] ERROR gyro\n");
        }

        vTaskDelayUntil(&last, dt_ticks);
    }
}

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
    /* UART */
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

    /* I2C + MPU */
    i2c_init();
    mpu_init();

    /* Tareas */
    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, 4, NULL);
    xTaskCreate(mpu_task,    "mpu_task",    4096, NULL, 6, NULL);
}
