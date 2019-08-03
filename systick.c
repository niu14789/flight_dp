/*!
    \file  systick.c
    \brief the systick configuration file
    
    \version 2017-02-10, V1.0.0, demo for GD32F30x
    \version 2018-10-10, V1.1.0, demo for GD32F30x
    \version 2018-12-25, V2.0.0, demo for GD32F30x
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include "systick.h"

volatile static U32 gs_dwTickCounterMs = 0; // the unit is ms and it will overall one more 49.71 year, it the unit is us and it will overall more than 1.19hour

/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)) {
        /* capture error */
        while (1) {
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/*!
    \brief      delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void DelayMs(U32 dwTimeMs)
{
    U32 dwStart, dwEnd;

    dwStart = gs_dwTickCounterMs;
    dwEnd = dwStart;
    while ((dwEnd - dwStart) < dwTimeMs)
    {
        dwEnd = gs_dwTickCounterMs;
    }
}


U64 GetTimeStampUs(void)
{
    U64 qwTemp;
    
    qwTemp = gs_dwTickCounterMs;
    qwTemp *= 1000;
    qwTemp += ((SysTick->LOAD + 1 - SysTick->VAL) / 120);
    return qwTemp;
}



U32 GetTickCountMs(void)
{
    return gs_dwTickCounterMs;
}

/*!
    \brief      delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    gs_dwTickCounterMs++;
}


