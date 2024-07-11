#ifndef APPLICATION_BUTTONS_H_
#define APPLICATION_BUTTONS_H_

#include <zephyr/drivers/gpio.h>

#define BUTTON_L DT_ALIAS(sw3)
#define BUTTON_R DT_ALIAS(sw4)
#define BUTTON_M DT_ALIAS(sw5)

#define BUTTON_ENABLE_POLL_TASK 1

// Enums and typedefs

typedef enum {
    BUTTON_STATE_UNKNOWN = -1,
    BUTTON_STATE_NOT_PRESSED = 0,
    BUTTON_STATE_PRESSED = 1
} button_state_t;

typedef enum {
    BUTTON_LEFT,
    BUTTON_MIDDLE,
    BUTTON_RIGHT,
} button_key_t;

typedef struct
{
    button_key_t key;
    button_state_t state;
} button_evt_t;

typedef void (*application_button_cb)(button_key_t key, button_state_t state);

// Variable declarations

extern button_state_t button_l_at_boot;
extern button_state_t button_m_at_boot;
extern button_state_t button_r_at_boot;

extern application_button_cb my_app_button_cb;

// Function declarations

int initialize_buttons(void);

button_state_t get_button_state_l(void);
button_state_t get_button_state_m(void);
button_state_t get_button_state_r(void);

// Task declaration, not in

#if BUTTON_ENABLE_POLL_TASK == 1
void button_task(void *, void *, void *);
#endif // BUTTON_ENABLE_POLL_TASK == 1

#endif // APPLICATION_BUTTONS_H_
