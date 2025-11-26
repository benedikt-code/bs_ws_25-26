/*
 * exception_handlers.h
 *
 *  Created on: Nov 19, 2025
 *      Author: andre
 */

#ifndef INC_EXCEPTION_HANDLERS_H_
#define INC_EXCEPTION_HANDLERS_H_

#include <stdint.h>

void init_stacks(void);

void test_exceptions(char);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);

#endif /* INC_EXCEPTION_HANDLERS_H_ */
