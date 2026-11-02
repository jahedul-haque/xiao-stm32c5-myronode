/**
 * @file    main.c
 * @brief   Node 1: Servo Follower Controller for Seeed Studio XIAO STM32C5
 *          Part of the MyroNode System (PlatformIO + Zephyr RTOS)
 *
 * Hardware:
 *  - Seeed Studio Bus Servo Driver Board / SCS0009 connected to USART1 (PA9 TX, PA10 RX)
 *  - Bus Servo Baudrate: 1,000,000 bps (1 Mbps)
 *  - CAN FD Bus: FDCAN2 (TCAN3403-Q1 onboard transceiver on PB5 RX, PB13 TX, PB14 STB)
 *
 * Operations:
 *  - Listens on CAN FD for Servo Commands (ID 0x210) & CAN Tests (ID 0x212) from Node 2.
 *  - Drives Feetech SCS0009 servo to mimic commanded position in real time.
 *  - Enables motor torque (0x28 = 1) at boot.
 *  - On CAN Test Ping (ID 0x212): Toggles onboard User LED and replies with ACK (ID 0x112).
 *  - Broadcasts live 20-byte telemetry frames (ID 0x110) back across CAN FD.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <string.h>

#define CAN_ID_SERVO_DATA       0x110u
#define CAN_ID_CAN_TEST_ACK     0x112u
#define CAN_ID_SERVO_CMD        0x210u
#define CAN_ID_CAN_TEST         0x212u

#define DEFAULT_SERVO_ID        1u

/* SCS0009 Protocol Constants */
#define SCS_INST_READ           0x02u
#define SCS_INST_WRITE          0x03u
#define SCS_REG_TORQUE_ENABLE   0x28u
#define SCS_REG_GOAL_POSITION   0x2Au
#define SCS_REG_PRESENT_POS     0x38u

/* 20-Byte Packed Telemetry Frame */
typedef struct __attribute__((packed)) {
    uint8_t  servo_id;
    uint8_t  status_flags;    /* 0 = OK, 1 = Timeout / Simulated */
    int16_t  current_pos;     /* 0 to 1023 */
    float    current_angle;   /* 0.0 to 300.0 degrees */
    int16_t  target_pos;      /* 0 to 1023 */
    float    target_angle;    /* 0.0 to 300.0 degrees */
    uint32_t uptime_ms;
} ServoTelemetry_t;

/* 7-Byte Packed Command Frame */
typedef struct __attribute__((packed)) {
    uint8_t  servo_id;
    int16_t  target_pos;      /* 0 to 1023 */
    uint16_t move_time_ms;    /* Transition time in ms */
    uint16_t move_speed;      /* 0 = max speed */
} ServoCommand_t;

static const struct device *can_dev;
static const struct device *uart_servo_dev;
static const struct gpio_dt_spec led_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(node_led), gpios);

static volatile int16_t g_target_pos = 512;
static volatile int16_t g_current_pos = 512;
static volatile uint8_t g_status_flags = 0;
static volatile bool    g_new_cmd_received = false;
static volatile bool    g_trigger_ack = false;
static volatile bool    g_led_toggle_state = false;

/* ============================================================================
 * SCS0009 PROTOCOL ENGINES
 * ============================================================================ */
static void scs_set_torque(const struct device *dev, uint8_t id, uint8_t enable)
{
    if (!dev) return;
    uint8_t pkt[8];
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = 0x04; /* Length: 1 param + 1 reg + 2 = 4 */
    pkt[4] = SCS_INST_WRITE;
    pkt[5] = SCS_REG_TORQUE_ENABLE;
    pkt[6] = enable;
    uint8_t sum = pkt[2] + pkt[3] + pkt[4] + pkt[5] + pkt[6];
    pkt[7] = ~sum;

    for (int i = 0; i < 8; i++) {
        uart_poll_out(dev, pkt[i]);
    }
}

static void scs_write_pos(const struct device *dev, uint8_t id, int16_t position, uint16_t time_ms, uint16_t speed)
{
    if (!dev) return;
    if (position < 0) position = 0;
    if (position > 1023) position = 1023;

    uint8_t pkt[13];
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = 0x09; /* Length: parameters + 2 */
    pkt[4] = SCS_INST_WRITE;
    pkt[5] = SCS_REG_GOAL_POSITION;
    pkt[6] = (uint8_t)((position >> 8) & 0xFF); /* Big-endian */
    pkt[7] = (uint8_t)(position & 0xFF);
    pkt[8] = (uint8_t)((time_ms >> 8) & 0xFF);
    pkt[9] = (uint8_t)(time_ms & 0xFF);
    pkt[10] = (uint8_t)((speed >> 8) & 0xFF);
    pkt[11] = (uint8_t)(speed & 0xFF);

    uint8_t sum = 0;
    for (int i = 2; i <= 11; i++) {
        sum += pkt[i];
    }
    pkt[12] = ~sum;

    for (int i = 0; i < 13; i++) {
        uart_poll_out(dev, pkt[i]);
    }
}

static bool scs_read_present_pos(const struct device *dev, uint8_t id, int16_t *pos_out)
{
    if (!dev) return false;
    uint8_t drain;
    while (uart_poll_in(dev, &drain) == 0) {
        /* Flush stale incoming buffer */
    }

    uint8_t req[8];
    req[0] = 0xFF;
    req[1] = 0xFF;
    req[2] = id;
    req[3] = 0x04;
    req[4] = SCS_INST_READ;
    req[5] = SCS_REG_PRESENT_POS;
    req[6] = 0x02; /* Read 2 bytes */
    req[7] = ~(req[2] + req[3] + req[4] + req[5] + req[6]);

    for (int i = 0; i < 8; i++) {
        uart_poll_out(dev, req[i]);
    }

    /* Wait for response (up to 4 ms at 1 Mbps) */
    uint8_t buf[24];
    int n = 0;
    int64_t start_time = k_uptime_get();

    while ((k_uptime_get() - start_time) < 4 && n < 24) {
        uint8_t byte;
        if (uart_poll_in(dev, &byte) == 0) {
            buf[n++] = byte;
        } else {
            k_busy_wait(15);
        }
    }

    /* Find valid Feetech status packet: 0xFF, 0xFF, id, len(4), err, posH, posL, csum */
    for (int i = 0; i <= (n - 8); i++) {
        if (buf[i] == 0xFF && buf[i+1] == 0xFF && buf[i+2] == id && buf[i+3] == 0x04) {
            uint8_t err = buf[i+4];
            uint8_t high = buf[i+5];
            uint8_t low = buf[i+6];
            uint8_t csum = buf[i+7];
            uint8_t expected = ~(buf[i+2] + buf[i+3] + buf[i+4] + buf[i+5] + buf[i+6]);
            if (csum == expected && err == 0) {
                *pos_out = (int16_t)(((uint16_t)high << 8) | low);
                return true;
            }
        }
    }

    return false;
}

/* ============================================================================
 * CAN RX CALLBACK FOR COMMANDS & TEST FRAMES FROM NODE 2
 * ============================================================================ */
static void can_cmd_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (frame->id == CAN_ID_SERVO_CMD && can_dlc_to_bytes(frame->dlc) >= sizeof(ServoCommand_t)) {
        ServoCommand_t cmd;
        memcpy(&cmd, frame->data, sizeof(ServoCommand_t));

        if (cmd.target_pos >= 0 && cmd.target_pos <= 1023) {
            g_target_pos = cmd.target_pos;
            g_new_cmd_received = true;
        }
    } else if (frame->id == CAN_ID_CAN_TEST) {
        /* CAN Test requested by Dashboard: signal main loop to toggle LED and send ACK */
        g_led_toggle_state = !g_led_toggle_state;
        g_trigger_ack = true;
    }
}

/* ============================================================================
 * MAIN APPLICATION LOOP
 * ============================================================================ */
int main(void)
{
    printk("\r\n===================================================\r\n");
    printk(" MyroNode Node 1: Servo Follower Controller\r\n");
    printk(" Target: Seeed Studio XIAO STM32C5 (SCS0009 @ 1 Mbps)\r\n");
    printk("===================================================\r\n");

    uart_servo_dev = DEVICE_DT_GET(DT_ALIAS(servo_uart));
    can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

    if (gpio_is_ready_dt(&led_gpio)) {
        gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_INACTIVE);
    }

    if (!device_is_ready(uart_servo_dev)) {
        printk("[ERROR] Servo UART not ready!\r\n");
    }

    /* Enable motor torque on SCS0009 (both unicast ID 1 and broadcast 0xFE) */
    k_msleep(100);
    scs_set_torque(uart_servo_dev, DEFAULT_SERVO_ID, 1);
    k_msleep(10);
    scs_set_torque(uart_servo_dev, 0xFE, 1);
    printk("[SERVO] Torque Enabled (Reg 0x28 = 1)\r\n");

    if (device_is_ready(can_dev)) {
        can_set_mode(can_dev, CAN_MODE_FD);
        can_start(can_dev);

        /* Filter: Accept standard 11-bit IDs in 0x210..0x21F range */
        const struct can_filter filter = {
            .id = 0x210,
            .mask = 0x7F0,
            .flags = 0  /* Standard 11-bit frame */
        };
        can_add_rx_filter(can_dev, can_cmd_rx_callback, NULL, &filter);
        printk("[FDCAN] Online - Listening for commands & tests (ID 0x210 - 0x21F)\r\n");
    } else {
        printk("[ERROR] CAN device not ready!\r\n");
    }

    /* Initial center position (512 counts = 150 deg) */
    scs_write_pos(uart_servo_dev, DEFAULT_SERVO_ID, 512, 500, 0);
    scs_write_pos(uart_servo_dev, 0xFE, 512, 500, 0);

    uint32_t loop_count = 0;
    static int16_t last_applied_pos = -1;

    while (1) {
        k_msleep(20); /* 50 Hz loop */
        loop_count++;

        /* 1. Handle CAN Test Ping ACK safely in thread context */
        if (g_trigger_ack) {
            g_trigger_ack = false;
            uint8_t ack_payload[4] = {'A', 'C', 'K', 0x01};
            struct can_frame ack_frame = {
                .id = CAN_ID_CAN_TEST_ACK,
                .flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
                .dlc = can_bytes_to_dlc(4)
            };
            memcpy(ack_frame.data, ack_payload, 4);
            can_send(can_dev, &ack_frame, K_NO_WAIT, NULL, NULL);
        }

        /* 2. Execute new command if received from Node 2 over CAN */
        if (g_new_cmd_received) {
            g_new_cmd_received = false;
            if (abs(g_target_pos - last_applied_pos) >= 2) {
                last_applied_pos = g_target_pos;
                scs_write_pos(uart_servo_dev, DEFAULT_SERVO_ID, g_target_pos, 0, 0);
                scs_write_pos(uart_servo_dev, 0xFE, g_target_pos, 0, 0);
            }
        }

        /* 3. Read present position from servo */
        int16_t pos_read = 0;
        if (scs_read_present_pos(uart_servo_dev, DEFAULT_SERVO_ID, &pos_read)) {
            g_current_pos = pos_read;
            g_status_flags = 0;
        } else {
            /* If physical servo not responding, smoothly simulate towards target */
            if (g_current_pos < g_target_pos) g_current_pos += 4;
            else if (g_current_pos > g_target_pos) g_current_pos -= 4;
            g_status_flags = 1; /* Timeout / Simulated */
        }

        /* 4. Update Onboard User LED:
         * Retains state from CAN Test LED Toggle; pulses when actively moving
         */
        bool led_state = g_led_toggle_state;
        if (abs(g_current_pos - g_target_pos) > 8) {
            if ((loop_count % 4) < 2) {
                led_state = !led_state; /* Activity blink during motion */
            }
        }
        gpio_pin_set_dt(&led_gpio, led_state ? 1 : 0);

        /* 5. Transmit telemetry over CAN FD at 25 Hz (every 2 loops) - NON-BLOCKING */
        if (loop_count % 2 == 0) {
            ServoTelemetry_t telem;
            telem.servo_id = DEFAULT_SERVO_ID;
            telem.status_flags = g_status_flags;
            telem.current_pos = g_current_pos;
            telem.current_angle = ((float)g_current_pos / 1023.0f) * 300.0f;
            telem.target_pos = g_target_pos;
            telem.target_angle = ((float)g_target_pos / 1023.0f) * 300.0f;
            telem.uptime_ms = (uint32_t)k_uptime_get();

            struct can_frame tx_frame = {
                .id = CAN_ID_SERVO_DATA,
                .flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
                .dlc = can_bytes_to_dlc(sizeof(ServoTelemetry_t))
            };
            memcpy(tx_frame.data, &telem, sizeof(ServoTelemetry_t));
            can_send(can_dev, &tx_frame, K_NO_WAIT, NULL, NULL);
        }
    }
    return 0;
}
