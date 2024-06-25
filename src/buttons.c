
#include "buttons.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(buttons);

static const struct gpio_dt_spec button_l = GPIO_DT_SPEC_GET(BUTTON_L, gpios);
static const struct gpio_dt_spec button_m = GPIO_DT_SPEC_GET(BUTTON_M, gpios);
static const struct gpio_dt_spec button_r = GPIO_DT_SPEC_GET(BUTTON_R, gpios);

static struct gpio_callback button_l_cb_data;
static struct gpio_callback button_m_cb_data;
static struct gpio_callback button_r_cb_data;

button_state_t button_l_at_boot = BUTTON_STATE_UNKNOWN;
button_state_t button_m_at_boot = BUTTON_STATE_UNKNOWN;
button_state_t button_r_at_boot = BUTTON_STATE_UNKNOWN;

application_button_cb my_app_button_cb = NULL;

static struct k_work button_work;

#define BUTTON_EVT_QUEUE_SIZE 10

K_MSGQ_DEFINE(button_queue, sizeof(button_evt_t), BUTTON_EVT_QUEUE_SIZE, 4);

void __attribute__((used)) button_l_isr(const struct device *dev,
                                        struct gpio_callback *cb,
                                        gpio_port_pins_t pins) {
    button_evt_t evt = {.key = BUTTON_LEFT, .state = get_button_state_l()};

    int err = k_msgq_put(&button_queue, &evt, K_NO_WAIT);
    if (err) {
        LOG_ERR("no space in queue for button event");
    }
    if (k_msgq_num_used_get(&button_queue) == 1) {
        k_work_submit(&button_work);
    }
}

void __attribute__((used)) button_m_isr(const struct device *dev,
                                        struct gpio_callback *cb,
                                        gpio_port_pins_t pins) {
    button_evt_t evt = {.key = BUTTON_MIDDLE, .state = get_button_state_m()};

    int err = k_msgq_put(&button_queue, &evt, K_NO_WAIT);
    if (err) {
        LOG_ERR("no space in queue for button event");
    }
    if (k_msgq_num_used_get(&button_queue) == 1) {
        k_work_submit(&button_work);
    }
}

void __attribute__((used)) button_r_isr(const struct device *dev,
                                        struct gpio_callback *cb,
                                        gpio_port_pins_t pins) {
    button_evt_t evt = {.key = BUTTON_RIGHT, .state = get_button_state_r()};

    int err = k_msgq_put(&button_queue, &evt, K_NO_WAIT);
    if (err) {
        LOG_ERR("no space in queue for button event");
    }
    if (k_msgq_num_used_get(&button_queue) == 1) {
        k_work_submit(&button_work);
    }
}

static void button_events_handler(struct k_work *worK) {
    button_evt_t evt;

    while (!k_msgq_get(&button_queue, &evt, K_NO_WAIT)) {
        LOG_INF("button %d: %d", evt.key, evt.state);
        if (my_app_button_cb != NULL) {
            my_app_button_cb(evt.key, evt.state);
        }
    }
}

int initialize_buttons(void) {
    k_work_init(&button_work, button_events_handler);

    if (!gpio_is_ready_dt(&button_l) || !gpio_is_ready_dt(&button_m) ||
        !gpio_is_ready_dt(&button_r)) {
        LOG_ERR("GPIO device tree entries not ready!");
        return -1;
    }

    (void)gpio_pin_configure_dt(&button_l, GPIO_INPUT | GPIO_ACTIVE_LOW);
    (void)gpio_pin_configure_dt(&button_m, GPIO_INPUT | GPIO_ACTIVE_LOW);
    (void)gpio_pin_configure_dt(&button_r, GPIO_INPUT | GPIO_ACTIVE_LOW);

    (void)gpio_pin_interrupt_configure_dt(&button_l, GPIO_INT_EDGE_BOTH);
    (void)gpio_pin_interrupt_configure_dt(&button_m, GPIO_INT_EDGE_BOTH);
    (void)gpio_pin_interrupt_configure_dt(&button_r, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&button_l_cb_data, button_l_isr, BIT(button_l.pin));
    gpio_init_callback(&button_m_cb_data, button_m_isr, BIT(button_m.pin));
    gpio_init_callback(&button_r_cb_data, button_r_isr, BIT(button_r.pin));

    gpio_add_callback_dt(&button_l, &button_l_cb_data);
    gpio_add_callback_dt(&button_m, &button_m_cb_data);
    gpio_add_callback_dt(&button_r, &button_r_cb_data);

    // record status of buttons at boot time
    button_l_at_boot = get_button_state_l();
    button_m_at_boot = get_button_state_m();
    button_r_at_boot = get_button_state_r();

    return 0;
}

button_state_t get_button_state_l(void) {
    if (!gpio_is_ready_dt(&button_l)) {
        return BUTTON_STATE_UNKNOWN;
    }

    int state = gpio_pin_get(button_l.port, button_l.pin);

    if (state == 1) {
        return BUTTON_STATE_PRESSED;
    }

    return BUTTON_STATE_NOT_PRESSED;
}

button_state_t get_button_state_m(void) {
    if (!gpio_is_ready_dt(&button_m)) {
        return BUTTON_STATE_UNKNOWN;
    }

    int state = gpio_pin_get(button_m.port, button_m.pin);

    if (state == 1) {
        return BUTTON_STATE_PRESSED;
    }

    return BUTTON_STATE_NOT_PRESSED;
}

button_state_t get_button_state_r(void) {
    if (!gpio_is_ready_dt(&button_r)) {
        return BUTTON_STATE_UNKNOWN;
    }

    int state = gpio_pin_get(button_r.port, button_r.pin);

    if (state == 1) {
        return BUTTON_STATE_PRESSED;
    }

    return BUTTON_STATE_NOT_PRESSED;
}

SYS_INIT(initialize_buttons, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#if BUTTON_ENABLE_POLL_TASK == 1
K_THREAD_DEFINE(button_task_id, 1024, button_task, NULL, NULL, NULL, 6, K_USER,
                0);

void button_task(void *, void *, void *) {
    while (1) {
        volatile button_state_t state_button_l = get_button_state_l();
        volatile button_state_t state_button_m = get_button_state_m();
        volatile button_state_t state_button_r = get_button_state_r();
        LOG_INF("gpio states are %d %d %d", state_button_l, state_button_m,
                state_button_r);

        k_msleep(5000);
    }
}

#endif // BUTTON_ENABLE_POLL_TASK == 1