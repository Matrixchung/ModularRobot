#ifndef _KEY_H
#define _KEY_H

#include "main.h"
#define KEY_SCAN_INTERVAL_MS 10
#define KEY_DEBOUNCE_MS      50
#define KEY_LONG_PRESS_MS    900
#define KEY_MULTI_TAP_MAX_MS 300
enum KEY_STATE
{
    NOT_PRESSED = 0,
    DEBOUNCED = 1,
    PRESSED = 2,
    LONG_PRESSED = 3,
    LONG_PRESS_RELEASED = 4,
    DOUBLE_TAPPED = 5,
    TRIPLE_TAPPED = 6,
};
typedef void (*Key_Callback)();
typedef struct Key_TypeDef
{
    GPIO_TypeDef *GPIO_Port;
    uint32_t GPIO_Pin;
    uint8_t active_state;
    enum KEY_STATE state;
    uint8_t tap_cnt;
    uint8_t _tap_ticks;
    uint8_t debounce_cnt;
    uint16_t long_press_cnt;
    Key_Callback callback;
}Key_TypeDef;

__STATIC_INLINE void Key_RegisterCallback(Key_TypeDef *key, Key_Callback cb)
{
    key->callback = cb;
}

extern Key_TypeDef Key_Onboard;

void Key_InstanceInit(Key_TypeDef *key, GPIO_TypeDef *port, uint32_t pin, uint8_t active);
void Key_InstanceUpdate(Key_TypeDef *key);

#endif