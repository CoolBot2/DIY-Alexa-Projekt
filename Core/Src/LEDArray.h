/*
 * LEDArray.h
 *
 *  Created on: Nov 21, 2025
 *      Author: daniel
 */

#include "main.h"
#ifndef SRC_LEDARRAY_H_
#define SRC_LEDARRAY_H_

class LEDArray{
public:
	LEDArray(GPIO_TypeDef* port,uint16_t pins[], uint8_t count);
	virtual ~LEDArray();
	void setnleds(int n);
	void restleds();
private:
	int pins_[10];
	GPIO_TypeDef* port_;
	int count;

};


#endif /* SRC_LEDARRAY_H_ */
