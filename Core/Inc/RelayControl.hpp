/*
 * RelayControl.hpp
 *
 *  Created on: Jul 2, 2025
 *      Author: MTHA1KOR
 */

#ifndef RELAYCONTROL_HPP
#define RELAYCONTROL_HPP

#include "stm32f4xx_hal.h"

class RelayControl {
public:
    RelayControl(GPIO_TypeDef* port, uint16_t pin);

    void on();   // Turns the relay ON (pin HIGH)
    void off();  // Turns the relay OFF (pin LOW)

private:
    GPIO_TypeDef* _port;
    uint16_t _pin;
};

#endif // RELAYCONTROL_HPP
