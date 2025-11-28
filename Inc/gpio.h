#ifndef GPIO_H_
#define GPIO_H_

#include <stdbool.h>
#include "stm32f4xx.h"


void button_init(void);
bool get_btn_state(void);

#endif
