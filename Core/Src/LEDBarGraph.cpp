/*
 * LEDBarGraph.cpp
 *
 *  Created on: Jun 10, 2025
 *      Author: ADE1HYD
 */

#include "LEDBarGraph.hpp"

LEDBarGraph::LEDBarGraph(GPIO_TypeDef* ports[10], uint16_t pins[10]) {
    for (int i = 0; i < 10; ++i) {
        _ports[i] = ports[i];
        _pins[i] = pins[i];
    }
}

void LEDBarGraph::setLevel(uint8_t percent) {
    uint8_t num_leds_on = (percent * 10) / 100;
    for (int i = 0; i < 10; ++i) {
        HAL_GPIO_WritePin(_ports[i], _pins[i],
            (i < num_leds_on) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
