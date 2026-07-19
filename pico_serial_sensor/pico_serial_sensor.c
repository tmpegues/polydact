#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define DELAY 10
#define BUTTON_1 16

void led_on(bool on)
{
    gpio_put(PICO_DEFAULT_LED_PIN, on);
}

void individual_msgs()
{
    uint16_t value0;
    uint16_t value1;
    uint16_t value2;

    adc_select_input(0);
    value0 = adc_read();
    printf("1 %u\n", value0);

    adc_select_input(1);
    value1 = adc_read();
    printf("2 %u\n", value1);

    adc_select_input(2);
    value2 = adc_read();
    printf("3 %u\n", value2);
}

bool check_button(bool mode)
{
    float button_1 = !gpio_get(BUTTON_1);
    if (button_1 > 0)
    {
        mode = !mode;
        sleep_ms(1000);
    }
    return mode;
}

void wiggle()
{
    // These values assume that sensor readings range from 0 to 4095.
    static int step = 0;
    for (int i =0; i < 3; i++)
    {
        printf("%d %f\n", i, sin(step + (i*2*3.14/3)*(4095/2)+(4095/2)));
    }
    step += 1;
}

int main()
{
    stdio_init_all();

    // At startup, ask

    gpio_init(BUTTON_1);
    gpio_set_dir(BUTTON_1, false); // Can also use GPIO_IN

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    adc_init();
    bool mode = true;
    bool led_active = true;
    while (1)
    {
        led_on(led_active);
        led_active = !led_active;

        if (mode)
        {
            individual_msgs();
        }
        else
        {
            wiggle();
        }
            mode = check_button(mode);
        sleep_ms(DELAY);
    }
}
