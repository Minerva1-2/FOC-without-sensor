#include "key.h"
#include "driver.h"
/* ==================== 按键状态机实现 ==================== */
static Key_t g_key1; /* KEY1(PC9) 状态机实例 */
static Key_t g_key2; /* KEY2(PB12) 状态机实例 */

/**
 * @brief  按键状态机扫描，需以固定节拍周期调用（本工程在 SysTick 1ms 中断中调用）
 * @param  key:  按键状态机实例
 * @param  port: 按键所在 GPIO 端口
 * @param  pin:  按键引脚
 */
void KeyScan(Key_t *key, GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t level = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;

    switch (key->state)
    {
    case KEY_STATE_IDLE:
        if (level == KEY_PRESS_LEVEL) /* 检测到按下 */
        {
            key->state = KEY_STATE_PRESS_DEBOUNCE;
            key->debounce_cnt = 0U;
        }
        break;

    case KEY_STATE_PRESS_DEBOUNCE:
        if (level == KEY_PRESS_LEVEL)
        {
            if (++key->debounce_cnt >= KEY_DEBOUNCE_CNT) /* 连续 20ms 为按下才确认 */
            {
                key->state = KEY_STATE_PRESSED;
                key->pressed_event = 1U; /* 产生按下事件 */
            }
        }
        else
        {
            key->state = KEY_STATE_IDLE; /* 抖动，回到空闲 */
            key->debounce_cnt = 0U;
        }
        break;

    case KEY_STATE_PRESSED:
        key->state = KEY_STATE_HOLD; /* 事件已发出，进入按住态 */
        break;

    case KEY_STATE_HOLD:
        if (level != KEY_PRESS_LEVEL) /* 检测到松开 */
        {
            key->state = KEY_STATE_RELEASE_DEBOUNCE;
            key->debounce_cnt = 0U;
        }
        break;

    case KEY_STATE_RELEASE_DEBOUNCE:
        if (level != KEY_PRESS_LEVEL)
        {
            if (++key->debounce_cnt >= KEY_DEBOUNCE_CNT)
            {
                key->state = KEY_STATE_IDLE; /* 确认释放，回到空闲 */
            }
        }
        else
        {
            key->state = KEY_STATE_HOLD; /* 又按下，回到按住 */
            key->debounce_cnt = 0U;
        }
        break;

    default:
        key->state = KEY_STATE_IDLE;
        break;
    }
}
static uint8_t KeyPressed(Key_t *key)
{
    uint8_t evt = key->pressed_event;
    key->pressed_event = 0U;
    return evt;
}
void KeyScanIsr(void) /* SysTick 1ms 调用，只扫不处理 */
{
    KeyScan(&g_key1, KEY1_GPIO_Port, KEY1_Pin);
    // KeyScan(&g_key2, KEY2_GPIO_Port, KEY2_Pin);
}