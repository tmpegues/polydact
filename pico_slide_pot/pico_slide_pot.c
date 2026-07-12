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

#define PWMDIV 1.0
#define WRAP 1000

#define SUPPLY_V 6.0
#define NUM_MOTORS 3
#define MIN_V 5.0
#define MAX_V 10.0

#define MIN_DUTY MIN_V / SUPPLY_V *WRAP
#define MAX_DUTY SUPPLY_V > MAX_V ? (MAX_V / SUPPLY_V * WRAP) : WRAP

#define P 0.5
#define I 0.0

struct repeating_timer timer1;
float goal[3] = {4095 / 2, 4095 / 2, 4095 / 2};

int POT[3] = {0, 1, 2};
int ENABLE[3] = {13, 16, 19};
int PIN_A[3] = {14, 17, 20};
int PIN_B[3] = {15, 18, 21};

int read_pot(int i)
{
    adc_select_input(POT[i]);

    return adc_read();
}

void position_pid(int motor, int desired)
{
    int pos = read_pot(motor);
    int error = pos - desired;
    static float integral;
    static int last_pos[NUM_MOTORS];

    if (error < 50 && error > -50)
    {
        integral = 0;
        pwm_set_gpio_level(ENABLE[motor], 0);
        // printf("motor cur goal error duty %d %4d %4d %4d %4d *\n", motor, pos, desired, error, desired);
    }
    else
    {
        integral += error;
        integral = integral > 4095 / 2 ? 4095 / 2 : integral;

        float duty = P * error + I * integral;

        // printf("motor cur goal error duty %d %4d %4d %4d %4d %.4f\n", motor, pos, desired, error, desired, duty);

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
    add_repeating_timer_ms(1, pid_callback, NULL, &timer1);
    int step = 0;
    float change = .01;

    while (true)
    {
        step++;
        if (step % 10 == 0)
        {
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                goal[i] = sin(step * change + i * (2 * 3.14) / 3.0) * 1800 + 4095 / 2;
            }
        }
        sleep_ms(1);

    }
}