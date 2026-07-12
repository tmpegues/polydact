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
#define MIN_V 5.0
#define MAX_V 10.0

#define MIN_DUTY MIN_V / SUPPLY_V *WRAP
#define MAX_DUTY SUPPLY_V > MAX_V ? (MAX_V / SUPPLY_V * WRAP) : WRAP

int POT[3] = {0, 1, 2};
int ENABLE[3] = {13, 16, 19};
int PIN_A[3] = {14, 17, 20};
int PIN_B[3] = {15, 18, 21};

int read_pot(int i)
{
    adc_select_input(POT[i]);

    return adc_read();
}

void set_motor_pos(int motor, int goal)
{
    int pos = read_pot(motor);
    float duty = ((pos - goal) / 4095.0);

    printf("motor cur goal duty %d %4d %4d %.4f\n", motor, pos, goal, duty);

    if (duty > 0)
    {
        duty = duty  * (MAX_DUTY - MIN_DUTY) + MIN_DUTY;
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
        pwm_set_gpio_level(ENABLE[motor], 0);
    }
    printf("%f\n", duty);
}

void init_motors(int num_motors)
{
    for (int i = 0; i < num_motors; i++)
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

int main()
{
    await_usb(1, true);
    init_motors(3);
    adc_init();

    while (true)
    {
        for (int i = 0; i < 3; i++)
        {
            // set_motor_pos(i, (int)((1.0 - i / 3.0) * read_pot(0)));
            set_motor_pos(i, 0);
        }
        printf("\n");
        sleep_ms(10);
    }
}