#include "pegues_lib.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#define BREATHE_MAX 10
#define AWAIT_SLEEP 50

int led_bright = 0;
struct repeating_timer led_timer;
bool breathe = false;

// Turn the led on or off, I use it to know that the loop is running properly
void toggle_led(int state)
{
    static bool led_on = true;
    if (state == 1)
    {
        led_on = true;
    }
    else if (state == -1)
    {
        led_on = false;
    }
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
    led_on = !led_on;
}

int init_led()
{
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;

#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    int driver_init = cyw43_arch_init();
    toggle_led(1);
    return driver_init;

#endif
}

void breathe_led(int brightness)
{
    if (breathe)
    {
        static bool increase = true;
        if (brightness <= 0)
        {
            if (led_bright > BREATHE_MAX / 2)
            {
                increase = false;
            }
            else if (led_bright < 0)
            {
                increase = true;
            }

            increase ? led_bright++ : led_bright--;
        }
        else
        {
            led_bright = brightness * BREATHE_MAX / 100;
        }
    }
    else if (brightness <= 0)
    {
        breathe = true;
    }
    else
    {
        toggle_led(brightness);
    }
}

bool breathe_timer_callback(__unused struct repeating_timer *t)
{
    static int i = 0;

    i = (i < BREATHE_MAX) ? i + 1 : 0;

    if (i <= led_bright)
    {
        toggle_led(1);
    }
    else
    {
        toggle_led(-1);
    }

    return true;
}

void await_usb(float await_seconds, bool want_breathe)
{
    stdio_init_all();
    int rc = init_led();

    bool indefinite = false;
    if (await_seconds == 0)
    {
        indefinite = true;
    }
    int max_loops = await_seconds * 1000.0 / AWAIT_SLEEP;
    int await_loop = 0;
    while (!stdio_usb_connected() && (indefinite || await_loop <= max_loops + 1))
    {
        toggle_led(0);
        sleep_ms(AWAIT_SLEEP); // Blink fast while waiting
        await_loop++;
    }
    toggle_led(-1);

    // Create brightness timer interrupt
    breathe = want_breathe;
    if (breathe)
    {
        add_repeating_timer_ms(-2, breathe_timer_callback, NULL, &led_timer);
    }
    else
    {
        // SOS pattern to indicate connected
        int blink_delay = 25;
        for (int i = 0; i <= 3; i++)
        {
            toggle_led(1);
            sleep_ms(blink_delay);
            toggle_led(-1);
            sleep_ms(blink_delay);
        }
        sleep_ms(blink_delay * 3);
        for (int i = 0; i <= 3; i++)
        {
            toggle_led(1);
            sleep_ms(blink_delay * 3);
            toggle_led(-1);
            sleep_ms(blink_delay);
        }
        sleep_ms(blink_delay * 3);
        for (int i = 0; i <= 3; i++)
        {
            toggle_led(1);
            sleep_ms(blink_delay);
            toggle_led(-1);
            sleep_ms(blink_delay);
        }
    }
    printf("USB serial connected after %d loops\n", await_loop);
}