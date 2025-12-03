#include "dht11.h"
#include "FreeRTOS.h"

// --- Private (static) function prototypes ---
static void set_pin_as_output(DHT11_t* dht);
static void set_pin_as_input(DHT11_t* dht);
static void delay_us(DHT11_t* dht, uint16_t us);
static DHT11_Status read_raw_data(DHT11_t* dht, uint8_t* data);

/**
 * @brief Configures the data pin as an open-drain output with pull-up.
 * @param dht Pointer to the DHT11 sensor instance.
 * @note Open-drain mode with a pull-up allows the MCU to pull the line low
 * and "release" it to let the pull-up resistor bring it high.
 */
static void set_pin_as_output(DHT11_t* dht) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = dht->data_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // Open-Drain mode
    GPIO_InitStruct.Pull = GPIO_PULLUP;         // Keep pull-up enabled
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(dht->data_port, &GPIO_InitStruct);
}

/**
 * @brief Configures the data pin as an input with a pull-up resistor.
 * @param dht Pointer to the DHT11 sensor instance.
 */
static void set_pin_as_input(DHT11_t* dht) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = dht->data_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Internal pull-up is essential
    HAL_GPIO_Init(dht->data_port, &GPIO_InitStruct);
}

/**
 * @brief Provides a blocking delay in microseconds using the provided timer.
 * @param dht Pointer to the DHT11 sensor instance.
 * @param us The delay duration in microseconds.
 * @note Assumes the timer is configured with a 1MHz clock source (1 tick = 1us).
 */
static void delay_us(DHT11_t* dht, uint16_t us) {
    __HAL_TIM_SET_COUNTER(dht->timer, 0);
    while (__HAL_TIM_GET_COUNTER(dht->timer) < us);
}

/**
 * @brief Initializes a DHT11 sensor instance.
 * @param dht Pointer to the DHT11_t struct to initialize.
 * @param data_port The GPIO port for the data pin (e.g., GPIOB).
 * @param data_pin The GPIO pin for the data pin (e.g., GPIO_PIN_5).
 * @param timer A pointer to a TIM_HandleTypeDef for microsecond operations.
 */
void DHT11_Init(DHT11_t* dht, GPIO_TypeDef* data_port, uint16_t data_pin, TIM_HandleTypeDef* timer) {
    dht->data_port = data_port;
    dht->data_pin = data_pin;
    dht->timer = timer;
}

/**
 * @brief Main public function to read humidity and temperature from the DHT11.
 * @param dht Pointer to the initialized DHT11_t struct.
 * @param humidity Pointer to a float where the humidity value (%) will be stored.
 * @param temperature Pointer to a float where the temperature value (°C) will be stored.
 * @return A DHT11_Status code indicating the result.
 */
DHT11_Status DHT11_Read(DHT11_t* dht, float* humidity, float* temperature) {
    uint8_t data[5] = {0}; // 5 bytes of data: RH_int, RH_dec, T_int, T_dec, Checksum

    // Attempt to read the raw data from the sensor
    DHT11_Status status = read_raw_data(dht, data);
    if (status != DHT11_OK) {
        return status; // Return the error status if reading failed
    }

    // Verify the checksum to ensure data integrity
    if (data[4] != (uint8_t)(data[0] + data[1] + data[2] + data[3])) {
        return DHT11_ERROR_CHECKSUM;
    }

    // Convert raw data to float values.
    // For DHT11, the decimal parts (data[1] and data[3]) are always zero.
    *humidity = (float)data[0];
    *temperature = (float)data[2];

    return DHT11_OK;
}

/**
 * @brief Handles the low-level communication protocol to read 40 bits of data.
 * @param dht Pointer to the DHT11 sensor instance.
 * @param data A buffer to store the 5 bytes of raw data.
 * @return A DHT11_Status code indicating the result.
 */
static DHT11_Status read_raw_data(DHT11_t* dht, uint8_t* data) {
    uint16_t timeout_us;

    // --- STEP 1: MCU SENDS START SIGNAL ---
    set_pin_as_output(dht);
    // Pull the line low for at least 18ms
    HAL_GPIO_WritePin(dht->data_port, dht->data_pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(18));
    // Release the line (pull high) and wait 20-40us for DHT's response
    HAL_GPIO_WritePin(dht->data_port, dht->data_pin, GPIO_PIN_SET);
    delay_us(dht, 30);
    set_pin_as_input(dht);


    // --- STEP 2: DHT SENDS RESPONSE SIGNAL ---
    // The DHT sensor responds by pulling the line low for ~80us, then high for ~80us.
    // We check for these pulses with a timeout.

    // Wait for the pin to go LOW (start of 80us low response pulse)
    timeout_us = 0;
    __HAL_TIM_SET_COUNTER(dht->timer, 0);
    while (HAL_GPIO_ReadPin(dht->data_port, dht->data_pin) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(dht->timer) > 100) return DHT11_ERROR_TIMEOUT_START;
    }

    // Wait for the pin to go HIGH (start of 80us high response pulse)
    __HAL_TIM_SET_COUNTER(dht->timer, 0);
    while (HAL_GPIO_ReadPin(dht->data_port, dht->data_pin) == GPIO_PIN_RESET) {
        if (__HAL_TIM_GET_COUNTER(dht->timer) > 100) return DHT11_ERROR_TIMEOUT_START;
    }

    // Wait for the pin to go LOW again (end of response, start of first data bit)
    __HAL_TIM_SET_COUNTER(dht->timer, 0);
    while (HAL_GPIO_ReadPin(dht->data_port, dht->data_pin) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(dht->timer) > 100) return DHT11_ERROR_TIMEOUT_START;
    }


    // --- STEP 3: DHT SENDS DATA ---
    // The sensor sends 40 bits of data. Each bit starts with a ~50us low pulse.
    // The duration of the following high pulse determines if the bit is a '0' or '1'.
    // '0': ~26-28us high pulse
    // '1': ~70us high pulse

    for (int i = 0; i < 40; i++) {
        // Wait for the pin to go HIGH (marks the start of the data pulse)
        __HAL_TIM_SET_COUNTER(dht->timer, 0);
        while (HAL_GPIO_ReadPin(dht->data_port, dht->data_pin) == GPIO_PIN_RESET) {
            if (__HAL_TIM_GET_COUNTER(dht->timer) > 70) return DHT11_ERROR_TIMEOUT_DATA;
        }

        // Measure the duration of the HIGH pulse to determine the bit value
        __HAL_TIM_SET_COUNTER(dht->timer, 0);
        while (HAL_GPIO_ReadPin(dht->data_port, dht->data_pin) == GPIO_PIN_SET) {
            if (__HAL_TIM_GET_COUNTER(dht->timer) > 100) return DHT11_ERROR_TIMEOUT_DATA;
        }
        uint16_t high_pulse_duration = __HAL_TIM_GET_COUNTER(dht->timer);

        // Shift the data byte left and add the new bit
        data[i / 8] <<= 1;
        // If the high pulse is longer than ~45us, it's a '1', otherwise it's a '0'
        if (high_pulse_duration > 45) {
            data[i / 8] |= 1;
        }
    }

    return DHT11_OK;
}
