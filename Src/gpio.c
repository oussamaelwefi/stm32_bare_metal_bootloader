#include "gpio.h"

#define GPIOAEN			(1U<<0)
#define BTN_PIN				(1U<<0)


void button_init(void)
{
	/*Enable clock access for GPIOC*/
	RCC->AHB1ENR |=GPIOAEN;

    // Configure a pull-down resistor for PA0 to ensure a stable low state when not pressed
    GPIOA->PUPDR &= ~(1U << 0);
    GPIOA->PUPDR |= (1U << 1);

}


bool get_btn_state(void)
{

	/*Note : BTN is active low*/

	/*Check if button is pressed*/
	if(GPIOA->IDR & BTN_PIN)
	{
		return true;
	}
	else
	{
		return false;
	}

}
