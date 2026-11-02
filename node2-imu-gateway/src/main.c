/**
 * @file    main.c
 * @brief   Node 2: IMU Leader & micro-ROS Gateway for Seeed Studio XIAO STM32C5
 *          Part of the MyroNode System (PlatformIO + Zephyr RTOS)
 *
 * Operations:
 *  - Initializes I2C2 and onboard ST LSM6DS3TR-C 6-axis IMU (0x6A).
 *  - Samples Accelerometer & Gyroscope at 50 Hz to compute real-time Pitch and Roll.
 *  - In MIMIC MODE: Maps tilt angle directly to servo target (0-300 deg / 0-1023 counts).
 *  - In MANUAL MODE: Follows dashboard slider command directly.
 *  - Transmits target position over CAN FD (ID 0x210) to Node 1 (non-blocking).
 *  - On "CAN_TEST" from serial/dashboard: Transmits CAN Test ping (ID 0x212) to Node 1.
 *  - Receives live servo actual position and status over CAN FD (ID 0x110) from Node 1.
 *  - Receives CAN Test ACK (ID 0x112) from Node 1 and counts successful round trips.
 *  - Streams structured JSON telemetry over USB CDC-ACM at 25 Hz to the Dashboard.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CAN_ID_SERVO_DATA       0x110u
#define CAN_ID_CAN_TEST_ACK     0x112u
#define CAN_ID_SERVO_CMD        0x210u
#define CAN_ID_CAN_TEST         0x212u

/* Minimum deflection required to transmit CAN command to Node 1 in mimic mode */
#define MIMIC_THRESHOLD_COUNTS  6u    /* 6 counts ≈ 1.75 degrees */
#define MANUAL_THRESHOLD_COUNTS 2u    /* 2 counts ≈ 0.58 degrees */

/* LSM6DS3TR-C Registers */
#define LSM6DS3_I2C_ADDR        0x6Au
#define LSM6DS3_WHO_AM_I        0x0Fu
#define LSM6DS3_CTRL1_XL        0x10u   /* Accel ODR & scale */
#define LSM6DS3_CTRL2_G         0x11u   /* Gyro ODR & scale */
#define LSM6DS3_OUTX_L_G        0x22u   /* Gyro output starting address */
#define LSM6DS3_OUTX_L_XL       0x28u   /* Accel output starting address */

/* Operating Modes */
#define MODE_MIMIC              0u      /* Servo mirrors IMU tilt */
#define MODE_MANUAL             1u      /* Servo follows dashboard/ROS slider */
#define MODE_HOLD_CENTER        2u      /* Servo holds 150 deg (512 counts) */

/* 20-Byte Packed Telemetry Frame (From Node 1) */
typedef struct __attribute__((packed)) {
    uint8_t  servo_id;
    uint8_t  status_flags;    /* 0 = OK, 1 = Timeout / Simulated */
    int16_t  current_pos;     /* 0 to 1023 */
    float    current_angle;   /* 0.0 to 300.0 degrees */
    int16_t  target_pos;      /* 0 to 1023 */
    float    target_angle;    /* 0.0 to 300.0 degrees */
    uint32_t uptime_ms;
} ServoTelemetry_t;

/* 7-Byte Packed Command Frame (To Node 1) */
typedef struct __attribute__((packed)) {
    uint8_t  servo_id;
    int16_t  target_pos;      /* 0 to 1023 */
    uint16_t move_time_ms;    /* Transition time in ms */
    uint16_t move_speed;      /* 0 = max speed */
} ServoCommand_t;

static const struct device *can_dev;
static const struct device *i2c_dev;
static const struct device *uart_dev;
static const struct gpio_dt_spec led_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(node_led), gpios);

/* Shared System State */
static volatile uint8_t  g_mode = MODE_MIMIC;
static volatile int16_t  g_cmd_pos = 512;
static volatile float    g_cmd_angle = 150.0f;
static volatile float    g_pitch = 0.0f;
static volatile float    g_roll = 0.0f;
static volatile float    g_ax = 0.0f, g_ay = 0.0f, g_az = 1.0f;
static volatile float    g_gx = 0.0f, g_gy = 0.0f, g_gz = 0.0f;
static volatile uint32_t g_can_test_ack_count = 0;
static volatile bool     g_trigger_can_test = false;
static bool              g_imu_online = false;

static ServoTelemetry_t  g_latest_servo_telemetry = {
    .servo_id = 1,
    .status_flags = 0,
    .current_pos = 512,
    .current_angle = 150.0f,
    .target_pos = 512,
    .target_angle = 150.0f,
    .uptime_ms = 0
};

#define RX_BUF_SIZE 128
static char rx_buf[RX_BUF_SIZE];
static size_t rx_idx = 0;

/* ============================================================================
 * COMMAND PARSER (DISPATCHES SERIAL INGESTION TO SYSTEM MODES)
 * ============================================================================ */
static void process_serial_command(const char *cmd)
{
    if (!cmd || cmd[0] == '\0') return;

    if (strstr(cmd, "CAN_TEST") || strstr(cmd, "TOGGLE_LED")) {
        g_trigger_can_test = true;
    } else if (strstr(cmd, "MIMIC") || strstr(cmd, "\"mode\":0")) {
        g_mode = MODE_MIMIC;
    } else if (strstr(cmd, "MANUAL") || strstr(cmd, "\"mode\":1")) {
        g_mode = MODE_MANUAL;
    } else if (strstr(cmd, "HOLD") || strstr(cmd, "\"mode\":2")) {
        g_mode = MODE_HOLD_CENTER;
        g_cmd_pos = 512;
        g_cmd_angle = 150.0f;
    } else if (strstr(cmd, "TARGET:") || strstr(cmd, "\"target\":")) {
        const char *p = strstr(cmd, "TARGET:");
        if (p) {
            p += 7;
        } else {
            p = strstr(cmd, "\"target\":");
            if (p) p += 9;
        }
        if (p) {
            while (*p && (*p < '0' || *p > '9') && *p != '-' && *p != '.') p++;
            float tgt = (float)atof(p);
            if (tgt >= 0.0f && tgt <= 300.0f) {
                g_mode = MODE_MANUAL;
                g_cmd_angle = tgt;
                g_cmd_pos = (int16_t)((tgt / 300.0f) * 1023.0f);
            }
        }
    }
}

/* ============================================================================
 * CAN RX CALLBACK (RECEIVES TELEMETRY & TEST ACKS FROM NODE 1)
 * ============================================================================ */
static void can_servo_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    /* Use can_dlc_to_bytes to correctly convert CAN FD DLC to byte length */
    if (frame->id == CAN_ID_SERVO_DATA && can_dlc_to_bytes(frame->dlc) >= sizeof(ServoTelemetry_t)) {
        memcpy(&g_latest_servo_telemetry, frame->data, sizeof(ServoTelemetry_t));
    } else if (frame->id == CAN_ID_CAN_TEST_ACK) {
        g_can_test_ack_count++;
        gpio_pin_toggle_dt(&led_gpio);
    }
}

/* ============================================================================
 * ONBOARD LSM6DS3TR-C ACCELEROMETER + GYRO DRIVER
 * ============================================================================ */
static bool imu_init(void)
{
    if (!device_is_ready(i2c_dev)) {
        printk("[IMU] I2C device not ready\r\n");
        return false;
    }

    /* Check WHO_AM_I (expected 0x6A for LSM6DS3TR-C) */
    uint8_t who = 0;
    uint8_t reg = LSM6DS3_WHO_AM_I;
    if (i2c_write_read(i2c_dev, LSM6DS3_I2C_ADDR, &reg, 1, &who, 1) != 0) {
        printk("[IMU] WHO_AM_I read failed\r\n");
        return false;
    }
    printk("[IMU] WHO_AM_I = 0x%02X\r\n", who);

    /* 1. Accel: 104 Hz ODR, +-2g full scale (CTRL1_XL = 0x40) */
    uint8_t ctrl1[2] = {LSM6DS3_CTRL1_XL, 0x40};
    if (i2c_write(i2c_dev, ctrl1, 2, LSM6DS3_I2C_ADDR) != 0) return false;

    /* 2. Gyro: 104 Hz ODR, 250 dps full scale (CTRL2_G = 0x40) */
    uint8_t ctrl2[2] = {LSM6DS3_CTRL2_G, 0x40};
    if (i2c_write(i2c_dev, ctrl2, 2, LSM6DS3_I2C_ADDR) != 0) return false;

    printk("[IMU] Initialized successfully at 104 Hz\r\n");
    return true;
}

static bool imu_read(float *pitch_out, float *roll_out)
{
    if (!g_imu_online || !device_is_ready(i2c_dev)) {
        return false;
    }

    uint8_t reg;
    uint8_t raw_xl[6];
    uint8_t raw_g[6];

    reg = LSM6DS3_OUTX_L_XL;
    if (i2c_write_read(i2c_dev, LSM6DS3_I2C_ADDR, &reg, 1, raw_xl, 6) != 0) return false;

    reg = LSM6DS3_OUTX_L_G;
    if (i2c_write_read(i2c_dev, LSM6DS3_I2C_ADDR, &reg, 1, raw_g, 6) != 0) return false;

    int16_t ax = (int16_t)((uint16_t)raw_xl[1] << 8 | raw_xl[0]);
    int16_t ay = (int16_t)((uint16_t)raw_xl[3] << 8 | raw_xl[2]);
    int16_t az = (int16_t)((uint16_t)raw_xl[5] << 8 | raw_xl[4]);

    int16_t gx = (int16_t)((uint16_t)raw_g[1] << 8 | raw_g[0]);
    int16_t gy = (int16_t)((uint16_t)raw_g[3] << 8 | raw_g[2]);
    int16_t gz = (int16_t)((uint16_t)raw_g[5] << 8 | raw_g[4]);

    /* Scaling: +-2g = 0.000061 g/LSB; +-250dps = 0.00875 dps/LSB */
    g_ax = (float)ax * 0.000061f;
    g_ay = (float)ay * 0.000061f;
    g_az = (float)az * 0.000061f;

    g_gx = (float)gx * 0.00875f;
    g_gy = (float)gy * 0.00875f;
    g_gz = (float)gz * 0.00875f;

    /* Euler angles from accelerometer */
    float acc_pitch = atan2f(-g_ax, sqrtf(g_ay * g_ay + g_az * g_az)) * (180.0f / 3.14159265f);
    float acc_roll  = atan2f(g_ay, g_az) * (180.0f / 3.14159265f);

    /* Complementary filter smoothing (0.15 accel, 0.85 prior) */
    g_pitch = 0.15f * acc_pitch + 0.85f * g_pitch;
    g_roll  = 0.15f * acc_roll  + 0.85f * g_roll;

    *pitch_out = g_pitch;
    *roll_out  = g_roll;
    return true;
}

/* ============================================================================
 * MAIN APPLICATION LOOP
 * ============================================================================ */
int main(void)
{
    /* Allow USB CDC enumeration */
    k_msleep(1500);

    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
    can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

    if (gpio_is_ready_dt(&led_gpio)) {
        gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_INACTIVE);
    }

    if (device_is_ready(uart_dev)) {
        /* Enable USB CDC ACM RX endpoint to accept OUT packets from host */
        uart_irq_rx_enable(uart_dev);
    }

    printk("[NODE 2] Booting MyroNode Gateway...\r\n");

    g_imu_online = imu_init();

    if (device_is_ready(can_dev)) {
        can_set_mode(can_dev, CAN_MODE_FD);
        can_start(can_dev);

        /* Filter: Accept standard 11-bit IDs in 0x110..0x11F range */
        const struct can_filter filter = {
            .id = 0x110,
            .mask = 0x7F0,
            .flags = 0  /* Standard 11-bit frame */
        };
        can_add_rx_filter(can_dev, can_servo_rx_callback, NULL, &filter);
        printk("[NODE 2] CAN FD started (500k/2M)\r\n");
    } else {
        printk("[NODE 2] CAN device not ready\r\n");
    }

    uint32_t loop_count = 0;
    static int16_t last_transmitted_pos = -999;
    static uint8_t last_transmitted_mode = 255;

    while (1) {
        k_msleep(20); /* 50 Hz control loop */
        loop_count++;

        /* 1. Drain USB CDC-ACM RX FIFO directly (Safe, Non-Blocking, 100% reliable) */
        if (device_is_ready(uart_dev)) {
            unsigned char ch;
            while (uart_poll_in(uart_dev, &ch) == 0) {
                if (ch == '\r' || ch == '\n') {
                    if (rx_idx > 0) {
                        rx_buf[rx_idx] = '\0';
                        process_serial_command(rx_buf);
                        rx_idx = 0;
                    }
                } else if (rx_idx < (RX_BUF_SIZE - 1)) {
                    rx_buf[rx_idx++] = (char)ch;
                }
            }
        }

        /* 2. Read IMU Pitch & Roll */
        float pitch = 0.0f, roll = 0.0f;
        if (!imu_read(&pitch, &roll)) {
            pitch = g_pitch;
            roll = g_roll;
        }

        /* 3. Handle CAN Test Ping (toggles Node 1 LED) - NON-BLOCKING */
        if (g_trigger_can_test) {
            g_trigger_can_test = false;
            uint8_t ping_data[4] = {'P', 'I', 'N', 'G'};
            struct can_frame test_frame = {
                .id = CAN_ID_CAN_TEST,
                .flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
                .dlc = can_bytes_to_dlc(4)
            };
            memcpy(test_frame.data, ping_data, 4);
            can_send(can_dev, &test_frame, K_NO_WAIT, NULL, NULL);
            gpio_pin_toggle_dt(&led_gpio); /* Toggle Node 2 LED as local feedback */
        }

        /* 4. Determine target position based on operating mode */
        int16_t target_pos_to_send = 512;

        if (g_mode == MODE_MIMIC) {
            /* Map Roll tilt [-60 deg, +60 deg] -> Servo Angle [0 deg, 300 deg] */
            float norm = (roll + 60.0f) / 120.0f;
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;

            target_pos_to_send = (int16_t)(norm * 1023.0f);
            g_cmd_pos = target_pos_to_send;
            g_cmd_angle = norm * 300.0f;
        } else if (g_mode == MODE_MANUAL) {
            target_pos_to_send = g_cmd_pos;
        } else {
            /* Hold center (150 deg) */
            target_pos_to_send = 512;
            g_cmd_pos = 512;
            g_cmd_angle = 150.0f;
        }

        /* 5. Transmit CAN FD Command when threshold exceeded or mode changed */
        int16_t deflection = abs(target_pos_to_send - last_transmitted_pos);
        int16_t threshold = (g_mode == MODE_MIMIC) ? MIMIC_THRESHOLD_COUNTS : MANUAL_THRESHOLD_COUNTS;

        if (deflection >= threshold || g_mode != last_transmitted_mode) {
            last_transmitted_pos = target_pos_to_send;
            last_transmitted_mode = g_mode;

            ServoCommand_t cmd;
            cmd.servo_id = 1;
            cmd.target_pos = target_pos_to_send;
            cmd.move_time_ms = 0;
            cmd.move_speed = 0;

            struct can_frame tx_frame = {
                .id = CAN_ID_SERVO_CMD,
                .flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
                .dlc = can_bytes_to_dlc(sizeof(ServoCommand_t))
            };
            memcpy(tx_frame.data, &cmd, sizeof(ServoCommand_t));
            can_send(can_dev, &tx_frame, K_NO_WAIT, NULL, NULL);
        }

        /* 6. Stream JSON Telemetry to USB CDC ACM at 25 Hz (every 2 loops) */
        if (loop_count % 2 == 0) {
            printf("{\"imu\":{\"pitch\":%.1f,\"roll\":%.1f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gz\":%.1f},"
                   "\"servo\":{\"pos\":%d,\"angle\":%.1f,\"target_pos\":%d,\"target_angle\":%.1f,\"status\":%d},"
                   "\"mode\":%d,\"seq\":%u,\"can_acks\":%u}\r\n",
                   (double)pitch, (double)roll, (double)g_ax, (double)g_ay, (double)g_az, (double)g_gz,
                   g_latest_servo_telemetry.current_pos,
                   (double)g_latest_servo_telemetry.current_angle,
                   (int)g_cmd_pos,
                   (double)g_cmd_angle,
                   (int)g_latest_servo_telemetry.status_flags,
                   (int)g_mode,
                   loop_count,
                   g_can_test_ack_count);
        }
    }
    return 0;
}
