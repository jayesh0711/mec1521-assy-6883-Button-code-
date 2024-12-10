/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

#define SLEEP_TIME_MS 1

/* Button and LED aliases from the devicetree */
#define SW0_NODE DT_ALIAS(sw0)         
#define LED0_NODE DT_ALIAS(led1)      

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: button alias 'sw0' not defined in the device tree"
#endif

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: LED alias 'led0' not defined in the device tree"
#endif

/* Define button and LED devices */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});

/* GPIO callback structure */
static struct gpio_callback button_cb_data;

/* Button pressed callback function */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button pressed on %s at %" PRIu32 "\n", dev->name, k_cycle_get_32());
    printk("inside the button Interuppt");
    /* Toggle the LED */
     k_msleep(SLEEP_TIME_MS);
    gpio_pin_toggle_dt(&led);       
}

int main(void)
{
    int ret;
      printk("Hello Surjit");
    /* Configure the LED as an output */
    if (!device_is_ready(led.port)) {
        printk("Error: LED device %s is not ready\n", led.port->name);
        return -1;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure LED %s pin %d\n", ret, led.port->name, led.pin);
        return ret;
    }

    /* Configure the button as an input with interrupt */
    if (!device_is_ready(button.port)) {
        printk("Error: Button device %s is not ready\n", button.port->name);
        return -1;
    }
    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure button %s pin %d\n", ret, button.port->name, button.pin);
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) { 
        printk("Error %d: failed to configure interrupt for button %s pin %d\n", ret, button.port->name, button.pin);
        return ret;
    }

    /* Initialize button callback */
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    printk("Set up button on %s pin %d\n", button.port->name, button.pin);
    printk("Set up LED on %s pin %d\n", led.port->name, led.pin);

    /* Main loop */
    while (1) {
        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}


// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/pwm.h>

// static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

// #define MIN_PERIOD PWM_MSEC(1)
// #define MAX_PERIOD PWM_SEC(1U)

// void main(void) {
//     printk("PWM LED Control Example\n");

//     if (!pwm_is_ready_dt(&pwm_led0)) {
//         printk("Error: PWM device is not ready\n");
//         return;
//     }

//     uint32_t period = MAX_PERIOD;
//     uint8_t dir = 0; /* Direction of change: 0 = decrease, 1 = increase */

//     while (1) {
//         pwm_set_dt(&pwm_led0, period, period / 2);
//         printk("PWM Period: %u\n", period);

//         if (dir) {
//             period *= 2;
//             if (period > MAX_PERIOD) {
//                 period = MAX_PERIOD / 2;
//                 dir = 0;
//             }
//         } else {
//             period /= 2;
//             if (period < MIN_PERIOD) {
//                 period = MIN_PERIOD * 2;
//                 dir = 1;
//             }
//         }

//         k_sleep(K_MSEC(500)); /* Adjust delay as needed */
//     }
// }
