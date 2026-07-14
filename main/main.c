#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include <stdio.h>
#include <math.h>

// ------ TB6612 Motor Pin Tanımları ------
#define PWMA_GPIO GPIO_NUM_25
#define AIN1_GPIO GPIO_NUM_26
#define AIN2_GPIO GPIO_NUM_27

#define PWMB_GPIO GPIO_NUM_14
#define BIN1_GPIO GPIO_NUM_32
#define BIN2_GPIO GPIO_NUM_33

#define STBY_GPIO GPIO_NUM_4

// ------ PWM Parametreleri ------
#define MCPWM_TIMER_RESOLUTION_HZ 10000000
#define MCPWM_PERIOD_TICKS 1000
#define MCPWM_DUTY_MAX MCPWM_PERIOD_TICKS

// ------ MPU6050 (IMU) Parametreleri ------
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_ADRESS 0x68

static const char *TAG_MOTOR = "TB6612";
static const char *TAG_IMU = "IMU_READER";
static const char *TAG_PID = "ROBOT";

typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BREAK,
    MOTOR_COAST,
} motor_dir_t;

typedef struct {
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    mcpwm_cmpr_handle_t comparator;
} motor_channel_t;

static motor_channel_t motor_a;
static motor_channel_t motor_b;

i2c_master_dev_handle_t master_dev_handle;
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;

// ------ PID Yapısı ------
typedef struct {
    float kp, ki, kd;
    float integral;
    float integral_limit;
    float output_min, output_max;
    float prev_error;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max, float integral_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->integral_limit = integral_limit;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
}

float pid_compute(pid_controller_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // Integral hesaplama
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // Türev hesaplama
    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

// ------ Motor Kontrol Fonksiyonları ------
static void motor_set_direction(motor_channel_t *m, motor_dir_t dir) {
    switch (dir) {
        case MOTOR_FORWARD:
            gpio_set_level(m->in1_gpio, 1);
            gpio_set_level(m->in2_gpio, 0);
            break;
        case MOTOR_BACKWARD:
            gpio_set_level(m->in1_gpio, 0);
            gpio_set_level(m->in2_gpio, 1);
            break;
        case MOTOR_BREAK:
            gpio_set_level(m->in1_gpio, 1);
            gpio_set_level(m->in2_gpio, 1);
            break;
        case MOTOR_COAST:
        default:
            gpio_set_level(m->in1_gpio, 0);
            gpio_set_level(m->in2_gpio, 0);
            break;
    }
}

void motor_set_speed(motor_channel_t *m, float duty_percent) {
    if (duty_percent > 100.0f) duty_percent = 100.0f;
    if (duty_percent < -100.0f) duty_percent = -100.0f;

    if (duty_percent > 0.0f) {
        motor_set_direction(m, MOTOR_FORWARD);
    } else if (duty_percent < 0.0f) {
        motor_set_direction(m, MOTOR_BACKWARD); 
        duty_percent = -duty_percent;
    } else {
        motor_set_direction(m, MOTOR_COAST);
    }

    uint32_t compare_value = (uint32_t)((duty_percent / 100.0f) * MCPWM_PERIOD_TICKS);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->comparator, compare_value));
}

static void motor_channel_init(motor_channel_t *m, gpio_num_t pwm_gpio, gpio_num_t in1_gpio, gpio_num_t in2_gpio, mcpwm_timer_handle_t g_timer) {
    m->in1_gpio = in1_gpio;
    m->in2_gpio = in2_gpio;

    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << in1_gpio) | (1ULL << in2_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    ESP_ERROR_CHECK(gpio_config(&io_config));
    gpio_set_level(in1_gpio, 0);
    gpio_set_level(in2_gpio, 0);

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };

    mcpwm_oper_handle_t oper_handle = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_handle));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_handle, g_timer));

    mcpwm_comparator_config_t comp_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_handle, &comp_config, &m->comparator));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->comparator, 0));

    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = pwm_gpio,
    };
    mcpwm_gen_handle_t gen_handle = NULL;

    ESP_ERROR_CHECK(mcpwm_new_generator(oper_handle, &gen_config, &gen_handle));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_handle,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
 
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_handle,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m->comparator, MCPWM_GEN_ACTION_LOW)));
}

void tb6612_init(void) {
    ESP_LOGI(TAG_MOTOR, "Motorlar baslatiliyor...");
    gpio_config_t stby_config = {
        .pin_bit_mask = (1ULL << STBY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&stby_config));
    gpio_set_level(STBY_GPIO, 1);

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .period_ticks = MCPWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_timer_handle_t g_timer = NULL;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &g_timer));

    motor_channel_init(&motor_a, PWMA_GPIO, AIN1_GPIO, AIN2_GPIO, g_timer);
    motor_channel_init(&motor_b, PWMB_GPIO, BIN1_GPIO, BIN2_GPIO, g_timer);

    ESP_ERROR_CHECK(mcpwm_timer_enable(g_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(g_timer, MCPWM_TIMER_START_NO_STOP));
}

// ------ IMU Başlatma Fonksiyonu ------
void imu_init() {
    ESP_LOGI(TAG_IMU, "IMU baslatiliyor...");
    i2c_master_bus_config_t master_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true }
    };
    i2c_master_bus_handle_t master_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&master_config, &master_handle));

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ 
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_handle, &device_config, &master_dev_handle));

    uint8_t write_buff[2] = {0x6B, 0x00}; 
    ESP_ERROR_CHECK(i2c_master_transmit(master_dev_handle, write_buff, sizeof(write_buff), -1));
}


// ------ Ana Döngü ------
int64_t last_time;
float filtered_angle = 0.0f;
pid_controller_t robot_pid;

void app_main(void) {
    imu_init();
    tb6612_init();
    
    last_time = esp_timer_get_time();
    
    // PID Parametreleri: kp, ki, kd, out_min (-100%), out_max (100%), integral_limit
    pid_init(&robot_pid, 5.0f, 0.1f, 1.0f, -100.0f, 100.0f, 20.0f);

    uint8_t reg_adrr = 0x3B;
    uint8_t read_buff[14];

    while(1) {
        esp_err_t reg = i2c_master_transmit_receive(master_dev_handle, &reg_adrr, 1, read_buff, sizeof(read_buff), -1);

        if (reg == ESP_OK) {
            accX = (read_buff[0] << 8) | read_buff[1];
            accY = (read_buff[2] << 8) | read_buff[3];
            accZ = (read_buff[4] << 8) | read_buff[5];
            gyroX = (read_buff[8] << 8) | read_buff[9];
            gyroY = (read_buff[10] << 8) | read_buff[11];
            gyroZ = (read_buff[12] << 8) | read_buff[13];

            int64_t now = esp_timer_get_time();
            float dt = (now - last_time) / 1000000.0f;
            last_time = now;

            float accX_g = accX / 16384.0f;
            float accZ_g = accZ / 16384.0f;
            float accel_angle = atan2f(accX_g, accZ_g) * 180.0f / M_PI;
            
            float gyro_rate = gyroY / 131.0f;  

            const float ALPHA = 0.98f;
            filtered_angle = ALPHA * (filtered_angle + gyro_rate * dt) + (1.0f - ALPHA) * accel_angle;
            
            // Setpoint 0.0f (Dik durma pozisyonu)
            float pid_output = pid_compute(&robot_pid, 0.0f, filtered_angle, dt);
            
            // DC Motorlara Hız/Yön ataması
            motor_set_speed(&motor_a, pid_output);
            
            
            motor_set_speed(&motor_b, pid_output); 

            ESP_LOGI(TAG_PID, "Aci: %.2f | PID Duty Cikisi: %.2f", filtered_angle, pid_output);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  
    }
}