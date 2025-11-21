/*
 * LEDArray.cpp
 *
 *  Created on: Nov 21, 2025
 *      Author: daniel
 */

#include "LEDArray.h"

LEDArray::LEDArray(GPIO_TypeDef* port,uint16_t pins[], uint8_t count):port_(port),count(count) {
	// TODO Auto-generated constructor stub
	for(int i=0;i<count;i++){
		pins_[i]=pins[i];
	}

}
void LEDArray::setnleds(int n){
	if(n>count) n=count;
	for(int i=0;i<count;i++){
	HAL_GPIO_WritePin(port_, pins_[i], GPIO_PIN_RESET);
	}
	for(int i=0;i<n;i++){
		HAL_GPIO_WritePin(port_, pins_[i], GPIO_PIN_SET);
		}
}
void LEDArray::restleds(){
	for(int i=0;i<count;i++){
		HAL_GPIO_WritePin(port_, pins_[i], GPIO_PIN_RESET);
		}
}
LEDArray::~LEDArray() {
	// TODO Auto-generated destructor stub
}

