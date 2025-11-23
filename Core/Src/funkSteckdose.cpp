/*
 * funkSteckdose.cpp
 *
 *  Created on: Nov 23, 2025
 *      Author: daniel
 */

#include "funkSteckdose.h"

funkSteckdose::funkSteckdose(TIM_HandleTypeDef htim2_):htim2(htim2_) {
	// TODO Auto-generated constructor stub

}

funkSteckdose::~funkSteckdose() {
	// TODO Auto-generated destructor stub
}

void funkSteckdose::ein(){
	sendSequence(0b111111000010, 12);
}
void funkSteckdose::aus(){
	sendSequence(0b111111000001, 12);
}
