/*******************************************************************************
 * stm32DigitalClock - a digital clock based on STM32F405 MCU
 * *****************************************************************************
 * Copyright (C) 2016-2017 Mikhail Kulesh
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#include "DigitalClock.h"

DigitalClock * appPtr = NULL;
RealTimeClock * rtcPtr = NULL;

int main(void)
{
    HAL_Init();

    IOPort defaultPortA(IOPort::PortName::A, GPIO_MODE_INPUT, GPIO_PULLDOWN);
    IOPort defaultPortB(IOPort::PortName::B, GPIO_MODE_INPUT, GPIO_PULLDOWN);
    IOPort defaultPortC(IOPort::PortName::C, GPIO_MODE_INPUT, GPIO_PULLDOWN);

    do
    {
        System::setClock(25, 200, FLASH_LATENCY_5, System::RtcType::RTC_EXT, 170);
    }
    while (System::getMcuFreq() != 100000000L);

    DigitalClock app;
    appPtr = &app;
    rtcPtr = appPtr->getRtc();
    app.run();
}

extern "C" void SysTick_Handler(void)
{
    rtcPtr->onMilliSecondInterrupt();
}

extern "C" void TIM3_IRQHandler()
{
    appPtr->onTim3Interrupt();
}


extern "C" void TIM4_IRQHandler()
{
    appPtr->onTim4Interrupt();
}

extern "C" void RTC_WKUP_IRQHandler()
{
    rtcPtr->onSecondInterrupt();
}

extern "C" void DMA2_Stream3_IRQHandler(void)
{
    Devices::SdCard::getInstance()->processDmaRxInterrupt();
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
    Devices::SdCard::getInstance()->processDmaTxInterrupt();
}

extern "C" void SDIO_IRQHandler(void)
{
    Devices::SdCard::getInstance()->processSdIOInterrupt();
}
