#include "key.h"

Key_TypeDef Key_Onboard;

void Key_InstanceInit(Key_TypeDef *key, GPIO_TypeDef *port, uint32_t pin, uint8_t active)
{
    key->GPIO_Port = port;
    key->GPIO_Pin = pin;
    key->active_state = active;
    key->state = NOT_PRESSED;
    key->debounce_cnt = 0;
    key->long_press_cnt = 0;
    key->callback = NULL;
}

// call every KEY_SCAN_INTERVAL
void Key_InstanceUpdate(Key_TypeDef *key)
{
    if(LL_GPIO_IsInputPinSet(key->GPIO_Port, key->GPIO_Pin) == key->active_state)
    {
        if(key->state == NOT_PRESSED) key->debounce_cnt++;
        if(key->debounce_cnt >= KEY_DEBOUNCE_MS / KEY_SCAN_INTERVAL_MS)
        {
            if(key->state == NOT_PRESSED) key->state = DEBOUNCED;
            if(key->state == DEBOUNCED)
            {
                key->long_press_cnt++;
                if(key->long_press_cnt >= KEY_LONG_PRESS_MS / KEY_SCAN_INTERVAL_MS)
                {
                    key->state = LONG_PRESSED;
                    if(key->callback != NULL) key->callback();
                }
            }
        }
    }
    else
    {
        key->debounce_cnt = 0;
        key->long_press_cnt = 0;
        if(key->state == DEBOUNCED)
        {
            key->tap_cnt++;
            // key->state = PRESSED;
            // if(key->callback != NULL) key->callback();
        }
        if(key->tap_cnt > 0)
        {
            if(key->_tap_ticks++ >= KEY_MULTI_TAP_MAX_MS / KEY_SCAN_INTERVAL_MS)
            {
                switch(key->tap_cnt)
                {
                    case 1:
                        key->state = PRESSED;
                        break;
                    case 2:
                        key->state = DOUBLE_TAPPED;
                        break;
                    case 3:
                        key->state = TRIPLE_TAPPED;
                        break;
                    default: break;
                }
                if(key->callback != NULL) key->callback();
                key->tap_cnt = 0;
                key->_tap_ticks = 0;
            }
        }
        else if(key->state == LONG_PRESSED)
        {
            key->state = LONG_PRESS_RELEASED;
            if(key->callback != NULL) key->callback();
        }
        key->state = NOT_PRESSED;
    }
}