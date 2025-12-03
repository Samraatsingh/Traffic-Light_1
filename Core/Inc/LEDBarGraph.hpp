/*
 * LEDBarGraph.hpp
 *
 *  Created on: Jun 10, 2025
 *      Author: ADE1HYD
 */

#ifndef INC_LEDBARGRAPH_HPP_
#define INC_LEDBARGRAPH_HPP_


#include "stm32f4xx_hal.h"

class LEDBarGraph {
public:
	LEDBarGraph(GPIO_TypeDef* ports[10], uint16_t pins[10]);
	void setLevel(uint8_t percent);  // 0 to 100%

private:
	GPIO_TypeDef* _ports[10];
	uint16_t _pins[10];
};


#endif /* INC_LEDBARGRAPH_HPP_ */
