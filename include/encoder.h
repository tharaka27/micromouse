/*
 ******************************************************************************
 * File Name          : encoder.h
 * Description        : This file provides code for the functions
 *                      for the encoders.
 ****************************************************************************** */

#ifndef __encoder_H
#define __encoder_H

#include "stdint.h"
#define MAX_ENCODER_VALUE 2147483648

void encoderStart(void);
uint32_t getLeftEncoderValue(void);
uint32_t getRightEncoderValue(void);
void setLeftEncoderValue(uint32_t value);
void setRightEncoderValue(uint32_t value);
void resetLeftEncoder(void);
void resetRightEncoder(void);
void advanceTicks(uint32_t ticks);
void uncontrolledAdvanceTicks(uint32_t ticks);
// left and right encoder values
extern uint32_t leftEncoderValue;
extern uint32_t rightEncoderValue;


#endif