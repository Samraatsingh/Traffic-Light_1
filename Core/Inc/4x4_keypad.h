//#include "gpio.h"

#pragma once
#include "stm32f4xx_hal.h"  // Adjust for your MCU

// Keypad structure definition
typedef struct {
    // Rows (Outputs)
    GPIO_TypeDef* row_ports[4];
    uint16_t row_pins[4];

    // Columns (Inputs)
    GPIO_TypeDef* col_ports[4];
    uint16_t col_pins[4];

    // Key layout (optional, customize per keypad)
    char keymap[4][4];
} Keypad_TypeDef;

// Function prototypes
void Keypad_Init(Keypad_TypeDef *keypad);
char Keypad_Scan(Keypad_TypeDef *keypad);
