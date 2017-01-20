#include "IntoBike_PWM_OUT.h"
void HAL_PWM_Write_With_Duty_Frequency_Ext(uint16_t pin, uint32_t duty, uint32_t pwm_frequency);
void outputPWM(uint16_t pin, uint32_t duty, uint32_t pwm_frequency)
{
	HAL_PWM_Write_With_Duty_Frequency_Ext(pin, duty, pwm_frequency);
}
void HAL_PWM_Write_With_Duty_Frequency_Ext(uint16_t pin, uint32_t duty, uint32_t pwm_frequency)
{
	//DEBUG("Enter HAL_PWM_Write_With_Frequency_Ext...");
	//Map the pin to the appropriate port and pin on the STM32
	//STM32_Pin_Info* PIN_MAP = HAL_Pin_Map();
	// exclude TIM1 for own use
	if(PIN_MAP[pin].timer_peripheral != NULL && PIN_MAP[pin].timer_peripheral != TIM1)
	{
		//DEBUG("PWM GPIO Configuration...");
		/* Common configuration for all channles */
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		// else if( (PIN_MAP[pin].timer_peripheral == TIM2) )
		if( (PIN_MAP[pin].timer_peripheral == TIM2) )
		{
			//DEBUG("PWM TIM2  Configuration...");
			__HAL_RCC_TIM2_CLK_ENABLE();
			GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
			GPIO_InitStruct.Pin       = PIN_MAP[pin].gpio_pin;
			/* Port Clock enable */
			if( (PIN_MAP[pin].gpio_peripheral == GPIOA) )
			{
				__HAL_RCC_GPIOA_CLK_ENABLE();
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			}
			else if( (PIN_MAP[pin].gpio_peripheral == GPIOB) )
			{
				__HAL_RCC_GPIOB_CLK_ENABLE();
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			}
		}
		else if( (PIN_MAP[pin].timer_peripheral == TIM3) )
		{
			//DEBUG("PWM TIM3  Configuration...");
			__HAL_RCC_TIM3_CLK_ENABLE();
			GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
			GPIO_InitStruct.Pin       = PIN_MAP[pin].gpio_pin;
			/* Port Clock enable */
			if( (PIN_MAP[pin].gpio_peripheral == GPIOA) )
			{
				__HAL_RCC_GPIOA_CLK_ENABLE();
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			}
			else if( (PIN_MAP[pin].gpio_peripheral == GPIOB) )
			{
				//DEBUG("PWM TIM3 GPIOB CLK ENABLE and GPIO Init");
				__HAL_RCC_GPIOB_CLK_ENABLE();
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			}
		}
		else if( (PIN_MAP[pin].timer_peripheral == TIM4) )
		{
			//DEBUG("PWM TIM4  Configuration...");
			__HAL_RCC_TIM4_CLK_ENABLE();
			GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
			GPIO_InitStruct.Pin       = PIN_MAP[pin].gpio_pin;
			/* Port Clock enable */
			if( (PIN_MAP[pin].gpio_peripheral == GPIOA) )
			{
				__HAL_RCC_GPIOA_CLK_ENABLE();
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			}
			else if( (PIN_MAP[pin].gpio_peripheral == GPIOB) )
			{
				__HAL_RCC_GPIOB_CLK_ENABLE();
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			}
		}
		else if( (PIN_MAP[pin].timer_peripheral == TIM5) )
		{
			//DEBUG("PWM TIM5  Configuration...");
			__HAL_RCC_TIM5_CLK_ENABLE();
			GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
			GPIO_InitStruct.Pin       = PIN_MAP[pin].gpio_pin;
			/* Port Clock enable */
			if( (PIN_MAP[pin].gpio_peripheral == GPIOA) )
			{
				__HAL_RCC_GPIOA_CLK_ENABLE();
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			}
			else if( (PIN_MAP[pin].gpio_peripheral == GPIOB) )
			{
				__HAL_RCC_GPIOB_CLK_ENABLE();
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			}
		}
		//DEBUG("SystemCoreClock: %ld", SystemCoreClock);
		//DEBUG("pwm_frequency: %ld", pwm_frequency);
		//DEBUG("value: %ld", value);
		#define TIM_COUNTER_CLOCK_FREQ 1000000 // in the param, the freq should be above 16 - 10000000
		// XXX:Note TIM_Prescaler and TIM_ARR and TIMCCR  should be
		// a number between 0x0000U and 0xFFFFU (0-65535)
		// XXX: Note here SystemCoreClock Should be TIM CLK, here equal
		// TIM_CLK should be set in the above code.
		/*##-1- Configure the TIM peripheral #######################################*/
		uint32_t TIM_Prescaler = (uint32_t)((SystemCoreClock / TIM_COUNTER_CLOCK_FREQ) - 1);
		uint32_t TIM_ARR;
		uint32_t TIM_CCR;
		if(pwm_frequency == 0)
		{
			TIM_ARR = 65535;
			//duty = 0;
		}
		else
		TIM_ARR = (uint32_t)((TIM_COUNTER_CLOCK_FREQ / pwm_frequency) - 1);
		if(TIM_ARR > 65535)
		{
			TIM_ARR = 65535;
			//duty    = 0;
		}
		TIM_CCR = (uint32_t)(duty * (TIM_ARR + 1) / MAX_DUTY_VALUE);
		//DEBUG("TIM_Prescaler: %d", TIM_Prescaler);
		//DEBUG("TIM_ARR: %d", TIM_ARR);
		//DEBUG("TIM_CCR: %d", TIM_CCR);
		TIM_HandleTypeDef TimHandle;
		TIM_OC_InitTypeDef sConfig;
		TimHandle.Instance = PIN_MAP[pin].timer_peripheral;
		TimHandle.Init.Prescaler         = TIM_Prescaler;
		TimHandle.Init.Period            = TIM_ARR - 1;
		TimHandle.Init.ClockDivision     = 0;
		TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
		TimHandle.Init.RepetitionCounter = 0;
		if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
		{
			// Error
			MO_DEBUG(("PWM Init Error!"));
		}
		/*##-2- Configure the PWM channels #b########################################*/
		/* Common configuration for all channels */
		sConfig.OCMode       = TIM_OCMODE_PWM1;
		sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
		sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
		sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
		/* Set the pulse value for channel 1 */
		sConfig.Pulse = TIM_CCR;
		if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, PIN_MAP[pin].timer_ch) != HAL_OK)
		{
			/* Configuration Error */
			MO_DEBUG(("PWM Configuration Error!"));
		}
		/* Start channel */
		if (HAL_TIM_PWM_Start(&TimHandle, PIN_MAP[pin].timer_ch) != HAL_OK)
		{
			/* PWM Generation Error */
			MO_DEBUG(("PWM Generation Error!"));
		}
	}
	else
	{
		// Error
		MO_DEBUG(("PWM First Error!"));
	}
}