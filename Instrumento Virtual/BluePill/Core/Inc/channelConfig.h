#ifndef CHANNEL_CONFIG_H
#define CHANNEL_CONFIG_H

#include "stm32f1xx_hal.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin[3];
    uint8_t  value[3];
    GPIO_TypeDef* portD;
    uint16_t digitalPin;
} ChannelInConfig;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t digitalPin;
} ChannelOutConfig;


ChannelInConfig channelsIn[] = {
	{GPIOB, {S1_Pin, S2_Pin, S0_Pin}, {0,0,0}, GPIOB, D_IN1_Pin}, // Case 0
	{GPIOB, {S1_Pin, S2_Pin, S0_Pin}, {0,0,1}, GPIOA, D_IN2_Pin}, // Case 1
	{GPIOB, {S1_Pin, S2_Pin, S0_Pin}, {0,1,0}, GPIOA, D_IN3_Pin}, // Case 2
	{GPIOB, {S1_Pin, S2_Pin, S0_Pin}, {0,1,1}, GPIOA, D_IN4_Pin}, // Case 3
	{GPIOB, {S1_Pin, S2_Pin, S0_Pin}, {1,0,0}, GPIOA, D_IN5_Pin}, // Case 4
	{GPIOB, {S1_Pin, S2_Pin, S0_Pin}, {1,0,1}, GPIOA, D_IN6_Pin}, // Case 5
	{GPIOB, {S1_Pin, S2_Pin, S0_Pin}, {1,1,0}, GPIOA, D_IN7_Pin}, // Case 6
	{GPIOB, {S1_Pin, S2_Pin, S0_Pin}, {1,1,1}, GPIOB, D_IN8_Pin}, // Case 7
};

ChannelOutConfig channelsOut[] = {
	{GPIOB, D_OUT1_Pin},
	{GPIOA, D_OUT2_Pin},
	{GPIOA, D_OUT3_Pin},
	{GPIOB, D_OUT4_Pin},
	{GPIOB, D_OUT5_Pin},
	{GPIOA, D_OUT6_Pin},
	{GPIOA, D_OUT7_Pin},
	{GPIOB, D_OUT8_Pin},

};

#endif /* CHANNEL_CONFIG_H */

