/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_nvmc.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_uart.h"

#define test 0

typedef  void (*pFunction)(void);
pFunction JumpToApplication;
uint32_t JumpAddress;
uint16_t checksum;

uint32_t Ver = 0x0100;

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#define UART_TX_BUF_SIZE 64                    
#define UART_RX_BUF_SIZE 64                   

#define IRQ_ENABLED             				0x01
#define MAX_NUMBER_INTERRUPTS   				32  

#define APP_ADR   						(uint32_t)0x7F000
#define APP_BIN_LENGTH  			(uint32_t)0x7F004
#define BOOT_START_ADDRESS		(uint32_t)0x00000
#define APPLICATION_ADDRESS   (uint32_t)0x06000

//#define RX_PIN_NUMBER  8
//#define TX_PIN_NUMBER  6
//#define CTS_PIN_NUMBER 7
//#define RTS_PIN_NUMBER 5

uint32_t APP_SIZE_VALUE = 0;
uint32_t APP_FLAG_VALUE   = 0;
static uint16_t tt_sum = 0;

/*CHECK FLASH IS READY STATUS*/
void wait_flash(void)
{
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {;}
}

/*FLASH READ FUNC*/
static uint32_t flash_R32_adr(uint32_t adr)
{
		return *(uint32_t *)adr;
}

/*FLASH WRITE FUNC*/
static void flash_W32_adr(uint32_t adr,uint32_t value)
{
		nrf_nvmc_write_word(adr,value);
}

//static uint16_t crc16(uint32_t address, uint32_t size, uint16_t const * p_crc)
//{
//    uint16_t crc = (p_crc == NULL) ? 0xFFFF : *p_crc;

//    for (uint32_t i = 0; i < size; i++)
//    {
//        crc  = (uint8_t)(crc >> 8) | (crc << 8);
//        crc ^= *(__IO uint32_t *)address;
//        crc ^= (uint8_t)(crc & 0xFF) >> 4;
//        crc ^= (crc << 8) << 4;
//        crc ^= ((crc & 0xFF) << 4) << 1;
//    }
//    return crc;
//}

static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq = 0;
    interrupt_setting_mask = NVIC->ISER[0];

    for (; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

static bool jump_adddress(uint32_t GOTO)
{
		bool ret;	
		if (((*(__IO uint32_t*)GOTO) & 0x2FFE0000 ) == 0x20000000)
		{
				ret = true;
				JumpAddress = *(__IO uint32_t*) (GOTO + 4);
				JumpToApplication = (pFunction) JumpAddress;
				__set_MSP(*(__IO uint32_t*) GOTO);
				interrupts_disable();
				JumpToApplication();
		}
		else 
				ret = false;
		return ret;
}

//void uart_error_handle(app_uart_evt_t * p_event)
//{
//    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_communication);
//    }
//    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_code);
//    }
//}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
		SCB->VTOR = BOOT_START_ADDRESS | 0;//offset
		uint32_t x;
		APP_FLAG_VALUE = (*(uint32_t *)APP_ADR);
		wait_flash();
		APP_SIZE_VALUE = (*(uint32_t *)APP_BIN_LENGTH);
		wait_flash();
		
		if((APP_FLAG_VALUE == 0x1) || (APP_FLAG_VALUE == 0x2))//NRF52_NODE, NRF52_GW
		{		
				for(x = 0; x < 0x1A; x++)
				{
						nrf_nvmc_page_erase(APPLICATION_ADDRESS+(0x1000*x));
						wait_flash();
				}
				for(x = 0; x < APP_SIZE_VALUE; x+=4)
				{
						flash_W32_adr((APPLICATION_ADDRESS + x),*(uint32_t *)(0x20000 + x));
						wait_flash();
				}
				for(x=0;x<APP_SIZE_VALUE;x+=4)
				{
						tt_sum ^= (uint16_t)flash_R32_adr(0x20000+x);
				}
				if(tt_sum == flash_R32_adr(0x7F008))
				{
						nrf_nvmc_page_erase(0x7F000);
						wait_flash();
						nrf_delay_ms(5);
						NVIC_SystemReset();
				}
		}
		else//JUMP TO USER ROM SPACE
		{		
				nrf_nvmc_page_erase(0x7F000);// TEST UPDATE BOOT
				wait_flash();
				jump_adddress(APPLICATION_ADDRESS);
		}
		for(;;);
}


/** @} */
