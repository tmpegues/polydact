#ifndef PEGUES_H
#define PEGUES_H

#include <stdio.h>
#include "pico/stdlib.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif


// init built in LED
int init_led();

// state -1 or 1 sets LED to off or on. state 0 toggles current state.
void toggle_led(int state);

// Waits for USB serial connection.
// If await_seconds = 0, it will wait indefinitely
// After USB serial connection, create an interrupt that
void await_usb(float await_seconds, bool want_breathe);

// Use brightness 1 to 10 to set brightness (0 = dim, 10 = full on)
// Use brightness < 0 to auto-sweep through brightness
void breathe_led(int brightness);

#endif