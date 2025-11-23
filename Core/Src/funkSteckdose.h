/*
 * funkSteckdose.h
 *
 *  Created on: Nov 23, 2025
 *      Author: daniel
 */

#ifndef SRC_FUNKSTECKDOSE_H_
#define SRC_FUNKSTECKDOSE_H_
#include "main.h"
class funkSteckdose {
private:
	TIM_HandleTypeDef htim2;
public:
	funkSteckdose(TIM_HandleTypeDef htim2_);
	virtual ~funkSteckdose();
	void ein();
	void aus();









	void delayMicroseconds(uint32_t us)
	{
	    __HAL_TIM_SET_COUNTER(&htim2, 0);
	    HAL_TIM_Base_Start(&htim2);

	    while (__HAL_TIM_GET_COUNTER(&htim2) < us) {

	    }

	    HAL_TIM_Base_Stop(&htim2);
	}
	void delay350Microseconds(int n){
		delayMicroseconds(n*350U);
	}
	void transmit(int numberHighPulses,int numberLowPulses){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		delay350Microseconds(numberHighPulses);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		delay350Microseconds(numberLowPulses);

	}

	void sendSequence(uint16_t bits, uint8_t length){
		for(int i=0;i<10;++i){
			for(int j=length-1;j>=0;--j){
				transmit(1, 3);
				 uint8_t isOne = (bits >> j) & 1;
				if(isOne==1)
					transmit(1, 3);
				else transmit(3,1);
			}
			transmit(1,31);
		}
	}
};

#endif /* SRC_FUNKSTECKDOSE_H_ */
