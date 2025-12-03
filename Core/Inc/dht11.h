#ifndef INC_DHT11_H_
#define INC_DHT11_H_

// Include the main HAL header for your specific STM32 series.
#include "stm32f4xx_hal.h"

/**
 * @brief Enum for DHT11 status codes.
 */
typedef enum {
    DHT11_OK = 0,
    DHT11_ERROR_TIMEOUT_START = -1,
    DHT11_ERROR_TIMEOUT_DATA = -2,
    DHT11_ERROR_CHECKSUM = -3
} DHT11_Status;

/**
 * @brief Struct to hold the configuration for a DHT11 sensor instance.
 */
typedef struct {
    GPIO_TypeDef* data_port;
    uint16_t           data_pin;
    TIM_HandleTypeDef* timer; // Timer for microsecond operations
} DHT11_t;

/**
 * @brief Initializes a DHT11 sensor instance.
 * @param dht Pointer to the DHT11_t struct to initialize.
 * @param data_port The GPIO port the DHT11 data pin is connected to (e.g., GPIOB).
 * @param data_pin The GPIO pin the DHT11 data pin is connected to (e.g., GPIO_PIN_5).
 * @param timer A pointer to a TIM_HandleTypeDef for generating microsecond delays.
 * This timer MUST be configured for a 1MHz clock (1 tick = 1us).
 */
void DHT11_Init(DHT11_t* dht, GPIO_TypeDef* data_port, uint16_t data_pin, TIM_HandleTypeDef* timer);

/**
 * @brief Reads humidity and temperature from the sensor.
 * @param dht Pointer to the initialized DHT11_t struct.
 * @param humidity Pointer to a float where the humidity value (%) will be stored.
 * @param temperature Pointer to a float where the temperature value (°C) will be stored.
 * @return A DHT11_Status code indicating the result of the operation.
 */
DHT11_Status DHT11_Read(DHT11_t* dht, float* humidity, float* temperature);


#endif /* INC_DHT11_H_ */
