#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pegues_lib.h"

// // 1st slide pot pins
// #define POT 0
// #define ENABLE 13
// #define PIN_A 14
// #define PIN_B 15

#define SUPPLY_V 12.0
#define NUM_MOTORS 3
#define MIN_V 4.9
#define MAX_V 10.0

#define PWMDIV 1.0
#define WRAP 4095

#define MIN_DUTY MIN_V / SUPPLY_V *WRAP
#define MAX_DUTY SUPPLY_V > MAX_V ? (MAX_V / SUPPLY_V * WRAP) : WRAP

#define P 0.0
#define I 0.0
#define D -1

#define A 0.1
#define B 0.9

struct repeating_timer timer1;
float goal[3] = {4095 / 2, 4095 / 2, 4095 / 2};

int POT[3] = {0, 1, 2};
int ENABLE[3] = {13, 16, 19};
int PIN_A[3] = {14, 17, 20};
int PIN_B[3] = {15, 18, 21};

int read_pot(int i)
{
    static float reads[NUM_MOTORS] = {0, 0, 0};
    adc_select_input(POT[i]);

    reads[i] = reads[i] * A + adc_read() * B;

    // if (i == 2)
    // {
    //     printf("in read %.4f %.4f %.4f\n", reads[0], reads[1], reads[2]);
    // }
    return reads[i];
}

void motor_duty(int motor, float duty)
{
    printf("duty %4f\n", duty);
    if (duty > 0)
    {
        duty = duty * (MAX_DUTY - MIN_DUTY) + MIN_DUTY;
        gpio_put(PIN_A[motor], 1);
        gpio_put(PIN_B[motor], 0);
        pwm_set_gpio_level(ENABLE[motor], duty);
    }
    else if (duty < 0)
    {
        duty = -duty * (MAX_DUTY - MIN_DUTY) + MIN_DUTY;
        gpio_put(PIN_A[motor], 0);
        gpio_put(PIN_B[motor], 1);
        pwm_set_gpio_level(ENABLE[motor], duty);
    }
    else
    {
        // pwm_set_gpio_level(ENABLE[motor], 0);
    }
    printf("duty %4f\n", duty);
}

void position_pid(int motor, int desired)
{
    int pos = read_pot(motor);
    int error = pos - desired;
    static float integral;
    static int last_pos[NUM_MOTORS] = {0, 0, 0};

    if (error < 0 && error > -0)
    {
        integral = 0;
        pwm_set_gpio_level(ENABLE[motor], 0);
        // printf("motor cur goal error duty %d %4d %4d %4d %4d *\n", motor, pos, desired, error, desired);
    }
    else
    {
        integral += error;
        integral = integral > 4095 / 2 ? 4095 / 2 : integral;

        float duty = P * error + I * integral + D * (pos - last_pos[motor]);
        motor_duty(motor, duty);
        // printf("motor cur goal error duty %d %4d %4d %4d %4d %.4f\n", motor, pos, desired, error, desired, duty);

        // printf("%f\n", duty);
    }
    last_pos[motor] = pos;
}

void init_motors()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        gpio_init(PIN_A[i]);
        gpio_init(PIN_B[i]);

        gpio_set_dir(PIN_A[i], GPIO_OUT);
        gpio_set_dir(PIN_B[i], GPIO_OUT);

        gpio_set_function(ENABLE[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(ENABLE[i]);

        pwm_set_clkdiv(slice, PWMDIV);
        pwm_set_wrap(slice, WRAP);
        pwm_set_enabled(slice, true);
    }
}

bool pid_callback(__unused struct repeating_timer *t)
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        position_pid(i, goal[i]);
    }
    // printf("\n");
    return true;
}

int main()
{
    await_usb(1, true);
    init_motors();
    adc_init();
    // add_repeating_timer_ms(1, pid_callback, NULL, &timer1);
    int step = 0;
    float change = .01;

    while (true)
    {
        step++;

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            float sin_val = sin(step * change) ;
            printf("sinval %f\n", sin_val);
            motor_duty(i, sin_val/ (i+1));
        }
        printf("%d %d %d\n\n", read_pot(0), read_pot(1), read_pot(2));
        sleep_ms(10);
    }
}