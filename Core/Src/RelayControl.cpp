/*
 * RelayControl.cpp
 *
 *  Created on: Jul 2, 2025
 *      Author: MTHA1KOR
 */
#include "RelayControl.hpp"

RelayControl::RelayControl(GPIO_TypeDef* port, uint16_t pin) {
    _port = port;
    _pin = pin;
}
void RelayControl::on() {
    HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
}

void RelayControl::off() {
    HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
}
