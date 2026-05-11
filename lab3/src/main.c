#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"

#include "esp_timer.h"

// =======================================================
// HARDWARE MAP
// =======================================================

#define PIN_TEMP       ADC1_CHANNEL_6
#define PIN_LIGHT      ADC1_CHANNEL_7

#define OUT_HEATER     GPIO_NUM_25
#define OUT_LED        GPIO_NUM_26

#define M1_A           GPIO_NUM_19
#define M1_B           GPIO_NUM_18

#define M2_A           GPIO_NUM_22
#define M2_B           GPIO_NUM_21

#define MOTOR_ENABLE   GPIO_NUM_23
#define MOTOR_PWM_A    GPIO_NUM_5
#define MOTOR_PWM_B    GPIO_NUM_4

// =======================================================
// PWM
// =======================================================

#define PWM_MODE       LEDC_LOW_SPEED_MODE
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_BITS       LEDC_TIMER_10_BIT

// =======================================================

typedef enum {
    STOPPED,
    FORWARD,
    BACKWARD
} fan_state_t;

typedef struct {
    float targetTemp;
    float currentTemp;
    float roomLight;
} sensor_pack_t;

typedef struct {
    int speed;
    fan_state_t dir;
} motor_pack_t;

// =======================================================

static sensor_pack_t sensors;
static motor_pack_t motor;

static esp_timer_handle_t timerMotor;

static int stepPos = 0;

// =======================================================
// MOTOR PATTERN
// =======================================================

const int sequence[4][2] = {
    { 1,  1},
    {-1,  1},
    {-1, -1},
    { 1, -1}
};

// =======================================================

void writeBridge(int p1, int p2, int value)
{
    gpio_set_level(p1, value == 1);
    gpio_set_level(p2, value == -1);
}

void applyStep(int a, int b)
{
    writeBridge(M1_A, M1_B, a);
    writeBridge(M2_A, M2_B, b);
}

void nextMotorStep()
{
    applyStep(sequence[stepPos][0], sequence[stepPos][1]);

    stepPos++;

    if(stepPos >= 4)
        stepPos = 0;
}

void prevMotorStep()
{
    applyStep(sequence[stepPos][0], sequence[stepPos][1]);

    stepPos--;

    if(stepPos < 0)
        stepPos = 3;
}

// =======================================================

int analogAverage(adc1_channel_t ch)
{
    int total = 0;

    for(int i=0;i<20;i++)
    {
        total += adc1_get_raw(ch);
        vTaskDelay(1);
    }

    return total / 20;
}

// =======================================================

float mapFloat(float x, float in_min, float in_max,
               float out_min, float out_max)
{
    return (x - in_min) *
           (out_max - out_min) /
           (in_max - in_min) +
           out_min;
}

// =======================================================

float readTemperature()
{
    int raw = analogAverage(PIN_TEMP);

    float mv = raw * 3300.0f / 4095.0f;

    float sensorValue = (3300.0f - mv) / 10.0f;

    float temp = mapFloat(sensorValue,
                          315.0f,
                          100.0f,
                          20.0f,
                          60.0f);

    if(temp < 0) temp = 0;
    if(temp > 100) temp = 100;

    return temp;
}

// =======================================================

float readLight()
{
    int raw = analogAverage(PIN_LIGHT);

    float percent = 100.0f - (raw * 100.0f / 4095.0f);

    percent = mapFloat(percent, 66, 100, 0, 100);

    if(percent < 0) percent = 0;
    if(percent > 100) percent = 100;

    return percent;
}

// =======================================================

void heaterControl(int state)
{
    gpio_set_level(OUT_HEATER, state);
}

// =======================================================

void lightOutput(float value)
{
    if(value < 0) value = 0;
    if(value > 100) value = 100;

    uint32_t duty = (value / 100.0f) * 1023;

    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

// =======================================================

void IRAM_ATTR timerCallback(void *arg)
{
    if(motor.speed <= 0)
        return;

    if(motor.dir == FORWARD)
        nextMotorStep();
    else if(motor.dir == BACKWARD)
        prevMotorStep();
}

// =======================================================

void updateMotor()
{
    esp_timer_stop(timerMotor);

    if(motor.speed <= 0 || motor.dir == STOPPED)
        return;

    int period = 1000000 / motor.speed;

    esp_timer_start_periodic(timerMotor, period);
}

// =======================================================

void controlAlgorithm()
{
    float error = sensors.currentTemp - sensors.targetTemp;

    if(error < -1)
    {
        heaterControl(1);

        motor.speed = 100;
        motor.dir = FORWARD;
    }
    else if(error < 1)
    {
        heaterControl(0);

        motor.speed = 0;
        motor.dir = STOPPED;
    }
    else
    {
        heaterControl(0);

        motor.dir = BACKWARD;

        if(error < 3)
            motor.speed = 100;
        else if(error < 5)
            motor.speed = 300;
        else
            motor.speed = 600;
    }

    updateMotor();
}

// =======================================================

float autoBrightness(float env)
{
    if(env < 20) return 100;
    if(env < 30) return 80;
    if(env < 40) return 60;
    if(env < 60) return 50;
    if(env < 80) return 30;

    return 0;
}

// =======================================================

void taskMain(void *arg)
{
    while(1)
    {
        sensors.currentTemp = readTemperature();
        sensors.roomLight = readLight();

        controlAlgorithm();

        float led = autoBrightness(sensors.roomLight);

        lightOutput(led);

        printf("SET %.1f | TEMP %.1f | LIGHT %.1f | FAN %d\n",
               sensors.targetTemp,
               sensors.currentTemp,
               sensors.roomLight,
               motor.speed);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =======================================================

void serialTask(void *arg)
{
    char cmd[64];

    while(1)
    {
        if(fgets(cmd, sizeof(cmd), stdin))
        {
            if(strstr(cmd, "SET_TEMP:"))
            {
                float t = atof(cmd + 9);

                if(t > 0 && t < 80)
                {
                    sensors.targetTemp = t;

                    printf("Nueva referencia %.1f\n", t);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =======================================================

void hardwareInit()
{
    gpio_config_t out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << OUT_HEATER) |
            (1ULL << M1_A) |
            (1ULL << M1_B) |
            (1ULL << M2_A) |
            (1ULL << M2_B) |
            (1ULL << MOTOR_ENABLE) |
            (1ULL << MOTOR_PWM_A) |
            (1ULL << MOTOR_PWM_B)
    };

    gpio_config(&out);

    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(PIN_TEMP, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PIN_LIGHT, ADC_ATTEN_DB_11);

    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_BITS,
        .freq_hz = 1000
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t led = {
        .gpio_num = OUT_LED,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER
    };

    ledc_channel_config(&led);

    esp_timer_create_args_t args = {
        .callback = timerCallback,
        .name = "motor_clock"
    };

    esp_timer_create(&args, &timerMotor);
}

// =======================================================

void app_main()
{
    sensors.targetTemp = 28.0f;

    hardwareInit();

    gpio_set_level(MOTOR_ENABLE, 1);

    xTaskCreate(taskMain,
                "mainTask",
                4096,
                NULL,
                5,
                NULL);

    xTaskCreate(serialTask,
                "serialTask",
                4096,
                NULL,
                3,
                NULL);
}