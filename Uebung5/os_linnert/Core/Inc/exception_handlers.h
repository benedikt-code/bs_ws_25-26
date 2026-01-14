/*
 * exception_handlers.h
 *
 *  Created on: Nov 19, 2025
 *      Author: andre
 */

#pragma once
#include <stdint.h>

// (timer interrupt printed '!' und triggert PendSV)
#define SYSTICK_HZ   (1000u / OS_TIME_SLICE_MS)

void systick_init(void);

void SysTick_Handler(void);
void PendSV_Handler(void);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void SysTick_Handler(void);
