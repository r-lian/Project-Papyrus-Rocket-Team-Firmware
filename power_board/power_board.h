#pragma once
#include "papyrus_hardware.h"

#define ADC_CHAN_5VREAD ADC_CHANNEL_2
#define ADC_CHAN_8VREAD ADC_CHANNEL_1
#define ADC_CHAN_24VREAD ADC_CHANNEL_0
#define ADC_CHAN_I3V3 ADC_CHANNEL_8
#define ADC_CHAN_I5V ADC_CHANNEL_7
#define ADC_CHAN_I8V ADC_CHANNEL_6
#define ADC_CHAN_I24VA ADC_CHANNEL_3
#define ADC_CHAN_I24VB ADC_CHANNEL_4

extern const PapyrusGPIO PB_UART_TX;
extern const PapyrusGPIO PB_UART_RX;
extern const PapyrusGPIO PB_FAULT_LED;
extern const PapyrusGPIO PB_24V_LED;
extern const PapyrusGPIO PB_8V_LED;
extern const PapyrusGPIO PB_5V_LED;
extern const PapyrusGPIO PB_5VBUCK_EN;
extern const PapyrusGPIO PB_8VBUCK_EN;
extern const PapyrusGPIO PB_24A_SW;
extern const PapyrusGPIO PB_24B_SW;
PapyrusStatus pb_hardware_init();
extern ADC_HandleTypeDef hadc;
