#include <dk_buttons_and_leds.h>
#include <pio.h>
#include <sys/printk.h>
#include <bt_test.h>

static void button_handler_cb(uint32_t button_state, uint32_t has_changed);

static struct button_handler button = {
    .cb = button_handler_cb,
};

static void button_handler_cb(uint32_t button_state, uint32_t has_changed)
{
    int err;
    uint32_t buttons = button_state & has_changed;

    if (buttons & (DK_BTN1_MSK | DK_BTN2_MSK)) {
    #ifdef CONFIG_BOARD_PARTICLE_XENON
        printk("\nSlave role. Starting advertising\n");
        adv_start();
    #elif CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP
        printk("\nMaster role. Starting scanning\n");
        scan_start();
    #else
        printk("\nUnknown device and role");
        return;
    #endif
    } else {
        return;
    }

    /* The role has been selected, button are not needed any more. */
    err = dk_button_handler_remove(&button);
    if (err) {
        printk("Button disable error: %d\n", err);
    }
}

void buttons_init(void)
{
    int err;

    err = dk_buttons_init(NULL);
    if (err) {
        printk("Buttons initialization failed.\n");
        return;
    }

    /*
     * Add dynamic buttons handler.
     * Buttons should be activated only when
     * during the board role choosing.
     */
    dk_button_handler_add(&button);
}
