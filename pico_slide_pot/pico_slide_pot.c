#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pegues_lib.h"

// // 1st slide pot pins
// #define POT 0
// #define ENABLE 13
// #define PIN_A 14
// #define PIN_B 15

#define PWMDIV 7.5
#define WRAP 1000

int POT[3] = {0, 1, 2};
int ENABLE[3] = {13, 16, 19};
int PIN_A[3] = {14, 17, 20};
int PIN_B[3] = {15, 18, 21};

void set_motor(int motor, int duty)
{
    duty *= 100;
    if (duty > 1000)
    {

        gpio_put(PIN_A[motor], 1);
        gpio_put(PIN_B[motor], 0);
        pwm_set_gpio_level(ENABLE[motor], duty);
    }
    else if (duty < -1000)
    {

        gpio_put(PIN_A[motor], 0);
        gpio_put(PIN_B[motor], 1);
        pwm_set_gpio_level(ENABLE[motor], -duty);

    }
    else
    {
    pwm_set_gpio_level(ENABLE[motor], 0);
    }
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

int read_pot(int i)
{
    adc_select_input(POT[i]);
    int value = 0;
    value = adc_read();
    value -= 4095 / 2;
    value *= 100;
    value /= 4095 / 2;

    return value;
}

int main()
{
    await_usb(0, true);
    init_motors(3);
    adc_init();

    int values[3];
    while (true)
    {
        for (int i = 0; i < 3; i++)
        {
            values[i] = read_pot(POT[i]);
            set_motor(i, values[i]);
        }
        printf("%d %d %d\n ", values[0], values[1], values[2]);
    }
}