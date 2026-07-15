#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define DELAY 10

#define SUPPLY_V 12.0
#define NUM_MOTORS 3
#define MIN_V 4.9
#define MAX_V 10.0

#define PWMDIV 1.0
#define WRAP 4095

#define MIN_DUTY MIN_V / SUPPLY_V *WRAP
#define MAX_DUTY SUPPLY_V > MAX_V ? (MAX_V / SUPPLY_V * WRAP) : WRAP

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

void individual_msgs()
{
    // Read a pot, send the pot message, wait to receive the motor
    // effort, apply that effort to the pot, then continue to the next
    for (int i = 0; i < NUM_MOTORS; i++)
    {

        adc_select_input(0);
        int pot_position = adc_read();
        printf("%d %u\n", i + 1, pot_position);

        int motor_num;
        int motor_effort;
        scanf("%d %d", &motor_num, &motor_effort);
        if (motor_num == i + 1)
        {
            motor_duty(i, 100 / motor_effort);
        }
        else
        { // If the received motor effort is invalid, pull towards the
          // middle position
            motor_duty(i, (pot_position - 4095 / 2) / (4095 / 2));
        }
    }
}

int main()
{
    stdio_init_all();

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
            float sin_val = sin(step * change);
            printf("sinval %f\n", sin_val);
            motor_duty(i, sin_val / (i + 1));
        }
        printf("%d %d %d\n\n", read_pot(0), read_pot(1), read_pot(2));
        sleep_ms(10);
    }
}