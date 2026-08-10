#ifndef __DRIVER_H
#define __DRIVER_H

#include "gpio.h"
#include "globalControl.h"

/* ==================== 按键状态机 ==================== */
#define KEY_PRESS_LEVEL     (0U)        /* 低有效按键：按下=低电平；高有效改 1 */
#define KEY_DEBOUNCE_CNT    (20U)       /* 消抖计数：1ms 节拍 × 20 = 20ms */

typedef enum {
    KEY_STATE_IDLE,             /* 空闲，等待按下 */
    KEY_STATE_PRESS_DEBOUNCE,   /* 按下消抖中 */
    KEY_STATE_PRESSED,          /* 已确认按下（产生一次按下事件） */
    KEY_STATE_HOLD,             /* 按住中 */
    KEY_STATE_RELEASE_DEBOUNCE, /* 释放消抖中 */
} KeyState_t;

typedef struct {
    KeyState_t state;           /* 当前状态 */
    uint16_t   debounce_cnt;    /* 消抖计数 */
    uint8_t    pressed_event;   /* 按下事件标志（查询后自动清零） */
} Key_t;

void KeyInit(Key_t *key);
void KeyScan(Key_t *key, GPIO_TypeDef *port, uint16_t pin);
void KeyEventHandler(Motor_t *motor);
void KeyScanIsr(void);

void MotorDriverEnable(void);
void MotorDriverDisable(void);
void LedON(Motor_t *motor);
void LedOFF(void);

#endif
