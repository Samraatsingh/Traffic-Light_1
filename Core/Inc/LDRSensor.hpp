/*
 * LDRSensor.hpp
 *
 *  Created on: Jul 4, 2025
 *      Author: ADE1HYD
 */

#ifndef INC_LDRSENSOR_HPP_
#define INC_LDRSENSOR_HPP_

#include "stm32f4xx_hal.h"

/**
 * @class LdrSensor
 * @brief C++ driver for an LDR (Light Dependent Resistor) sensor.
 *
 * This class provides a simple interface to read analog values from an LDR
 * connected to an STM32 microcontroller's ADC.
 */
class LdrSensor {
public:
    /**
     * @brief Constructor for the LdrSensor class.
     * @param adcHandle Pointer to the HAL ADC handle (e.g., &hadc1).
     * @param adcChannel The ADC channel the LDR is connected to (e.g., ADC_CHANNEL_0).
     */
    LdrSensor(ADC_HandleTypeDef* adcHandle, uint32_t adcChannel);

    /**
     * @brief Initializes the ADC channel for the LDR sensor.
     *
     * This function should be called once before reading values.
     * @return HAL_StatusTypeDef HAL status.
     */
    HAL_StatusTypeDef init();

    /**
     * @brief Reads the raw 12-bit digital value from the ADC.
     *
     * This performs a blocking read of the configured ADC channel.
     * @return uint16_t The raw ADC value (0-4095). Returns 0 on conversion failure.
     */
    uint16_t getRawValue();

    /**
     * @brief Reads the light intensity as a percentage.
     *
     * Converts the raw ADC value to a percentage (0.0% to 100.0%).
     * @return float The light level as a percentage.
     */
    float getLightPercentage();

private:
    ADC_HandleTypeDef* _adcHandle;
    uint32_t _adcChannel;
};

#endif // LDR_SENSOR_H



