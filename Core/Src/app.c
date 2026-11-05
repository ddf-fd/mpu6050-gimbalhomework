#include "app.h"
#include "mpu6050.h"
#include "oled.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart1;

#define TRIM_P_US    1500      // 俯仰中位，装好后微调（±20 约 ±2°）
#define TRIM_Y_US    1500
#define Y_MIN_US      800
#define Y_MAX_US     2200

#define ALPHA        0.98f     // 互补滤波系数
#define US_PER_DEG   11.11f    // 2000us / 180°
#define RX_SIZE        64      // 串口环形缓冲

static MPU6050_Data imu;
static float pitch, roll, yaw;
static float cmd_p, cmd_y;

static int servo_p_us = TRIM_P_US;
static int servo_y_us = TRIM_Y_US;

static uint8_t rx_byte;
static volatile uint8_t rx_buf[RX_SIZE];
static volatile uint8_t rx_head = 0, rx_tail = 0;

static char cmd_line[24];
static uint8_t cmd_len = 0;
static uint32_t t_filter, t_servo, t_oled;
static uint32_t t_rx;                     // 最后一个字节的到达时刻

//浮点转 1 位小数字符串
static void f1(char *o, float v)
{
    int n = (int)(v * 10.0f + (v < 0 ? -0.5f : 0.5f));

    if (n < 0) {
        n = -n;
        sprintf(o, "-%d.%d", n / 10, n % 10);
    } else {
        sprintf(o, "%d.%d", n / 10, n % 10);
    }
}

static void apply_attitude(float dt)
{
    float pa, ra;

    MPU6050_Read(&imu);

    pa = atan2f(-imu.ax, sqrtf(imu.ay * imu.ay + imu.az * imu.az)) * 57.29578f;
    ra = atan2f(imu.ay, imu.az) * 57.29578f;

    pitch = ALPHA * (pitch + imu.gy * dt) + (1.0f - ALPHA) * pa;
    roll  = ALPHA * (roll  + imu.gx * dt) + (1.0f - ALPHA) * ra;
    yaw  += imu.gz * dt;
}

//舵机
static void servo_update(void)
{
    int tp = TRIM_P_US + (int)(cmd_p * US_PER_DEG);
    int ty = TRIM_Y_US + (int)(cmd_y * US_PER_DEG);

    if (ty < Y_MIN_US) ty = Y_MIN_US;
    if (ty > Y_MAX_US) ty = Y_MAX_US;

    servo_p_us = tp;
    servo_y_us = ty;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_p_us);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo_y_us);
}

//屏幕
static void draw(void)
{
    char s[12];

    OLED_Clear();
    f1(s, pitch); OLED_Puts(0, 1, s);
    f1(s, roll);  OLED_Puts(0, 3, s);
    f1(s, yaw);   OLED_Puts(0, 5, s);
    OLED_Refresh();
}

static void handle_cmd(char *s)
{
    float d = atof(s + 1);

    if (s[0] == 'P') {
        cmd_p += d;
    } else if (s[0] == 'Y') {
        cmd_y += d;
    }
}

//从环形缓冲取数据，攒成一行再处理
static void poll_rx(void)
{
    char c;

    while (rx_tail != rx_head) {
        c = (char)rx_buf[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1) % RX_SIZE);
        t_rx = HAL_GetTick();

        if (c == '\r' || c == '\n') {
            if (cmd_len) {
                cmd_line[cmd_len] = 0;
                handle_cmd(cmd_line);
                cmd_len = 0;
            }
        } else if (cmd_len < sizeof(cmd_line) - 1) {
            cmd_line[cmd_len++] = c;
        }
    }

    //30ms 内没有新字节，直接执行 【波特律动串口助手预设发送里面不能打回车:(】
    if (cmd_len && (HAL_GetTick() - t_rx >= 30)) {
        cmd_line[cmd_len] = 0;
        handle_cmd(cmd_line);
        cmd_len = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t next;

    if (huart->Instance == USART1) {
        next = (uint8_t)((rx_head + 1) % RX_SIZE);
        if (next != rx_tail) {
            rx_buf[rx_head] = rx_byte;
            rx_head = next;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void App_Init(void)
{
    OLED_Init();

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    MPU6050_Init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    draw();

    t_filter = t_servo = t_oled = HAL_GetTick();
}

void App_Loop(void)
{
    uint32_t now = HAL_GetTick();

    if (now - t_filter >= 5) {
        float dt = (now - t_filter) * 0.001f;
        t_filter = now;
        apply_attitude(dt);
    }

    poll_rx();

    if (now - t_servo >= 20) {
        t_servo = now;
        servo_update();
    }

    if (now - t_oled >= 100) {
        t_oled = now;
        draw();
    }
}
