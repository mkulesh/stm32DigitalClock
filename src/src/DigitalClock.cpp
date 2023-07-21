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

#include <cmath>
#include <cstdlib>

#define USART_DEBUG_MODULE "CLOCK: "

DigitalClock::DigitalClock ():
    // logging
    log(Usart::USART_1, IOPort::B, GPIO_PIN_6, GPIO_PIN_7, 500000),
    pinUsbVoltage(IOPort::B, GPIO_PIN_5, GPIO_MODE_INPUT, GPIO_PULLDOWN),

    // RTC
    rtc(),

    // HMI
    pinHmiPower(IOPort::C, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN, GPIO_SPEED_HIGH, true, false),
    eventLedToggle(rtc, 500, 1),
    eventSecToggle(rtc, 1000),
    spiHmi(Spi::SPI_2,
        /*sckPort =  */ IOPort::B, GPIO_PIN_13,
        /*misoPort = */ IOPort::B, GPIO_PIN_14,
        /*mosiPort = */ IOPort::B, GPIO_PIN_15,
        GPIO_PULLUP),

    // SSD
    pinSsdCs(IOPort::B, GPIO_PIN_10, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_HIGH, true, true),
    ssd(spiHmi, pinSsdCs, true),

    // LCD
    pinLcdCs(IOPort::C, GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_HIGH, true, true),
    pinLcdRs(IOPort::C, GPIO_PIN_6, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_HIGH, true, true),
    lcd(spiHmi, pinLcdCs, pinLcdRs, false, 63),

    // DAC
    pinHmiBrightnessCs(IOPort::B, GPIO_PIN_11, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_HIGH, true, true),
    hmiBrightness(spiHmi, pinHmiBrightnessCs, Devices::Dac_MCP49x1::Resolution::BIT_12, 2500, 3000),
    adcBrightness(IOPort::A, GPIO_PIN_0, AnalogToDigitConverter::DeviceName::ADC_2, ADC_CHANNEL_0, MAIN_VOLTAGE),

    // DCF
    pinDcfInput(IOPort::A, GPIO_PIN_3, GPIO_MODE_INPUT, GPIO_NOPULL),
    pinDcfPower(IOPort::A, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP),
    dcf(rtc, pinDcfInput, pinDcfPower, Timer::TIM_4, TIM4_IRQn),
    dcfState(DcfState::NONE),
    dcfReceiverStartTime(0),
    dcfTimeReceived(false),

    // SD card
    pinSdPower(IOPort::A, GPIO_PIN_10, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN, GPIO_SPEED_HIGH, true, false),
    pinSdDetect(IOPort::A, GPIO_PIN_12, GPIO_MODE_INPUT, GPIO_PULLUP),
    portSd1(IOPort::C,
            /* mode     = */ GPIO_MODE_OUTPUT_PP,
            /* pull     = */ GPIO_PULLUP,
            /* speed    = */ GPIO_SPEED_FREQ_VERY_HIGH,
            /* pin      = */ GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12,
            /* callInit = */ false),
    portSd2(IOPort::D,
            /* mode     = */ GPIO_MODE_OUTPUT_PP,
            /* pull     = */ GPIO_PULLUP,
            /* speed    = */ GPIO_SPEED_FREQ_VERY_HIGH,
            /* pin      = */ GPIO_PIN_2,
            /* callInit = */ false),
    sdCard(pinSdDetect, portSd1, portSd2),
    sdCardInserted(false),

    // Configuration
    config(pinSdPower, sdCard, "conf.txt"),

    // Sound
    pinAmpPower(IOPort::B, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN, GPIO_SPEED_HIGH, true, false),
    pinAmpMute(IOPort::B, GPIO_PIN_1, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN, GPIO_SPEED_HIGH, true, false),
    spiWav(Spi::SPI_1,
        /*sckPort =  */ IOPort::A, GPIO_PIN_5,
        /*misoPort = */ IOPort::A, GPIO_PIN_6,
        /*mosiPort = */ IOPort::A, GPIO_PIN_7,
        GPIO_PULLUP),
    pinLeftChannel(IOPort::C, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP),
    pinRightChannel(IOPort::C, GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP),
    pinWavSample(IOPort::A, GPIO_PIN_11, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN),
    wavStreamer(sdCard, spiWav, pinLeftChannel, pinRightChannel, Timer::TIM_3, TIM3_IRQn),

    // Piezo element
    piezoAlarm(IOPort::C, GPIO_PIN_2, rtc),

    // Buttons
    bMode(IOPort::C, GPIO_PIN_1, rtc),
    bActiveElement(IOPort::B, GPIO_PIN_9, rtc),
    bPlus(IOPort::B, GPIO_PIN_2, rtc),
    bMinus(IOPort::B, GPIO_PIN_8, rtc),

    // Screens
    homeScreen(),
    timeSetting(),
    brightnessSetting(config),
    activeScreen(SCR_HOME),
    activeElementVisible(false),
    activeElementToggle(rtc, 250, 3),
    returnToHome(rtc, 30000, 1),

    // Alarms
    alarmSetting1(0, config),
    alarmSetting2(1, config),
    alarmSetting3(2, config),

    // Temperature
    adcTemperature(IOPort::A, GPIO_PIN_1, AnalogToDigitConverter::DeviceName::ADC_1, ADC_CHANNEL_1, MAIN_VOLTAGE),
    temperature(0),
    temperatureTrial(),

    // auto brightness value
    brightnessValue(0),

    // Interrupt priorities
    irqPrioSd (3, 0), // also prio 4 will be used
    irqPrioRtc(2, 0),
    irqPrioDcf(1, 0),
    irqPrioWav(0, 1)
{
    Devices::Ssd::SegmentsMask sm;
    sm.top = 3;
    sm.rightTop = 5;
    sm.rightBottom = 7;
    sm.bottom = 4;
    sm.leftBottom = 1;
    sm.leftTop = 2;
    sm.center = 6;
    sm.dot = 0;
    ssd.setSegmentsMask(sm);

    buttons[0] = &bMode;
    buttons[1] = &bActiveElement;
    buttons[2] = &bPlus;
    buttons[3] = &bMinus;

    screens[SCR_HOME] = &homeScreen;
    screens[SCR_TIME_SETTING] = &timeSetting;
    screens[SCR_BRIGHTNESS] = &brightnessSetting;
    screens[SCR_ALARM1] = &alarmSetting1;
    screens[SCR_ALARM2] = &alarmSetting2;
    screens[SCR_ALARM3] = &alarmSetting3;

    alarms[0] = &alarmSetting1;
    alarms[1] = &alarmSetting2;
    alarms[2] = &alarmSetting3;

    dayTime.tm_sec  = 0;
    dayTime.tm_min  = 0;
    dayTime.tm_hour = 0;
    dayTime.tm_mday = 0;
    dayTime.tm_mon  = 0;
    dayTime.tm_year = 0;
}


void DigitalClock::run ()
{
    updateLoggingState();

    USART_DEBUG("Oscillator frequency: " << System::getExternalOscillatorFreq()
        << ", MCU frequency: " << System::getMcuFreq());

    pinHmiPower.setHigh();
    HAL_Delay(100);

    lcd.start(2);
    spiHmi.start(SPI_DIRECTION_1LINE, SPI_BAUDRATEPRESCALER_128, SPI_DATASIZE_8BIT, SPI_PHASE_2EDGE);

    updateBrightness();
    updateSsd();

    for (auto & b : buttons)
    {
        b->setHandler(this);
    }

    sdCard.setIrqPrio(irqPrioSd);
    sdCard.initInstance();
    wavStreamer.setTestPin(&pinWavSample);
    wavStreamer.setHandler(this);

    sdCardInserted = sdCard.isCardInserted();
    if (sdCardInserted)
    {
        config.readConfiguration();
    }

    adcTemperature.start();
    adcBrightness.start();

    rtcToDayTime();
    setTime();
    rtc.start(8*2047 + 7, RTC_WAKEUPCLOCK_RTCCLK_DIV2, irqPrioRtc, NULL);

    startDcf(true);

    while(true)
    {
        periodic();
    }
}


void DigitalClock::periodic ()
{
    wavStreamer.periodic();
    dcf.periodic();
    piezoAlarm.periodic();
    for (auto & b : buttons)
    {
        b->periodic();
    }
    if (eventSecToggle.isOccured())
    {
        eventLedToggle.resetTime();
        activeElementToggle.resetTime();
        updateSdCardState();
        updateLoggingState();
        measureTemperature();
        updateBrightness();
        updateLcd(true);
        updateSsd();
        size_t alarmNumber = config.getAlarmOccured(dayTime);
        if (alarmNumber < Config::ALARMS_NUMBER)
        {
            startAlarm(alarmNumber);
        }
        if (dayTime.tm_hour == DCF_START_HOUR && dayTime.tm_min == DCF_START_MINUTES)
        {
            startDcf(false);
        }
    }
    else if (eventLedToggle.isOccured())
    {
        ssdDots[0] = (UsartLogger::getInstance() != NULL);
        ssdDots[1] = false;
        ssdDots[2] = true;
        ssdDots[3] = false;
        ssd.putDots(ssdDots, 4);
    }
    if (activeScreen != SCR_HOME)
    {
        if (activeElementToggle.isOccured())
        {
            updateLcd(true);
        }
        else if (returnToHome.isOccured())
        {
            setScreen(SCR_HOME);
        }
    }
}


void DigitalClock::resetEventTime ()
{
    for (auto & b : buttons)
    {
        b->resetTime();
    }
    eventSecToggle.resetTime();
    eventLedToggle.resetTime();
    activeElementToggle.resetTime();
}


void DigitalClock::setScreen (ScreenType scr)
{
    activeScreen = scr;
    screens[activeScreen]->setFirst();
    activeElementVisible = activeScreen != SCR_HOME;
    lcd.clear();
    updateLcd(false);
    if (activeScreen == SCR_HOME)
    {
        config.flush();
    }
}


void DigitalClock::updateBrightness ()
{
    if (config.getBrightness().isManual)
    {
        hmiBrightness.putValue(config.getBrightness().manValue);
    }
    else
    {
        int currLight = (int)(5*100.0*adcBrightness.getVoltage()/MAIN_VOLTAGE);
        brightnessValue = std::min(currLight, 100);
        hmiBrightness.putValue(100 - brightnessValue);
    }
}


void DigitalClock::updateLcd (bool changeActiveElement)
{
    rtcToDayTime();
    screens[activeScreen]->fillLine(0, this, lcdString);
    lcd.putString(0, 0, lcdString, ::strlen(lcdString));
    screens[activeScreen]->fillLine(1, this, lcdString);
    lcd.putString(0, 1, lcdString, ::strlen(lcdString));
    if (changeActiveElement)
    {
        activeElementVisible = !activeElementVisible;
    }
}


void DigitalClock::updateSsd ()
{
    ssdDots[0] = (UsartLogger::getInstance() != NULL);
    ssdDots[1] = true;
    ssdDots[2] = false;
    ssdDots[3] = false;
    sprintf(lcdString, "%02d%02d", dayTime.tm_hour, dayTime.tm_min);
    ssd.putString(lcdString, ssdDots, 4);
}


void DigitalClock::modifyActiveElement (int s)
{
    switch (activeScreen)
    {
        case SCR_HOME:
        {
            return;
        }
        case SCR_TIME_SETTING:
        {
            timeSetting.modifyValue(dayTime, s);
            setTime();
            updateSsd();
            break;
        }
        case SCR_BRIGHTNESS:
        {
            brightnessSetting.modifyValue(s);
            updateBrightness();
            break;
        }
        case SCR_ALARM1:
            alarmSetting1.modifyValue(s);
            break;
        case SCR_ALARM2:
            alarmSetting2.modifyValue(s);
            break;
        case SCR_ALARM3:
            alarmSetting3.modifyValue(s);
            break;
    }
    activeElementVisible = true;
    updateLcd(false);
}


bool DigitalClock::isAlarmActive () const
{
    return config.isAlarmActive();
}


void DigitalClock::setTime ()
{
    rtc.setTimeSec(::mktime(&dayTime));
    resetEventTime();
}


void DigitalClock::measureTemperature ()
{
    float v = adcTemperature.getVoltage()*1000.0;
    float t = (float)(30.0 + (10.888 - ::sqrt(10.888*10.888 + 4.0*0.00347*(1777.3-v)))/(-2.0*0.00347) - 7);
    temperatureArr[temperatureTrial] = t;
    ++temperatureTrial;
    if (temperatureTrial == TEMPERATURE_TRIALS)
    {
        float temperatureSum = 0.0;
        for (size_t i = 0; i < TEMPERATURE_TRIALS; ++i)
        {
            temperatureSum += temperatureArr[i];
        }
        temperature = temperatureSum / ((float)TEMPERATURE_TRIALS);
        temperatureTrial = 0;
    }
}


void DigitalClock::updateLoggingState ()
{
    if (pinUsbVoltage.getBit() && UsartLogger::getInstance() == NULL)
    {
        log.initInstance();
    }
    else if (!pinUsbVoltage.getBit() && UsartLogger::getInstance() != NULL)
    {
        log.clearInstance();
    }
}


void DigitalClock::updateSdCardState ()
{
    if (!sdCardInserted && sdCard.isCardInserted())
    {
        config.readConfiguration();
    }
    sdCardInserted = sdCard.isCardInserted();
}


void DigitalClock::startAlarm (size_t n)
{
    if (wavStreamer.isActive() || piezoAlarm.isActive())
    {
        return;
    }
    wavStreamer.setVolume((float)config.getSoundVolume()/100.0);
    bool wavStarted = wavStreamer.start(
            irqPrioWav, WavStreamer::SourceType::SD_CARD, config.getAlarm(n).sound);
    if (!wavStarted)
    {
        piezoAlarm.start(15);
    }
}


bool DigitalClock::writeLogToSd (const char * logStr)
{
    if (!sdCard.isCardInserted())
    {
        return false;
    }

    sdCard.clearPort();
    pinSdPower.setHigh();
    HAL_Delay(250);

    FIL logFile;
    if (sdCard.openAppend(6, &logFile, LOG_FILE_NAME) == FR_OK)
    {
        f_printf(&logFile, "%02d.%02d.%04d %02d:%02d:%02d: %s\n",
                dayTime.tm_mday, dayTime.tm_mon + 1, dayTime.tm_year + FIRST_CALENDAR_YEAR,
                dayTime.tm_hour, dayTime.tm_min, dayTime.tm_sec,
                logStr);
        f_close(&logFile);
    }

    sdCard.stop();
    pinSdPower.setLow();
    return true;
}


void DigitalClock::startDcf (bool alarm)
{
    if (!dcf.isActive())
    {
        dcfReceiverStartTime = rtc.getTimeSec();
        dcf.start(irqPrioDcf, this);
        if (alarm)
        {
            piezoAlarm.start(1);
        }
    }
}


void DigitalClock::onButtonPressed (const Devices::Button * b, uint32_t numOccured)
{
    if (wavStreamer.isActive())
    {
        wavStreamer.stop();
        return;
    }
    if (piezoAlarm.isActive())
    {
        piezoAlarm.stop();
        return;
    }

    if (b == &bMode)
    {
        USART_DEBUG("mode button pressed: " << numOccured);
        activeScreen = (activeScreen == (ScreenType)(SCR_NUMBER - 1))? SCR_HOME : (ScreenType)((int)activeScreen + 1);
        setScreen(activeScreen);
    }
    else if (b == &bActiveElement)
    {
        USART_DEBUG("active element button pressed: " << numOccured);
        screens[activeScreen]->setNext();
        activeElementToggle.resetTime();
        activeElementVisible = true;
        updateLcd(false);
    }
    else if (b == &bPlus)
    {
        USART_DEBUG("plus button pressed: " << numOccured);
        if (activeScreen == ScreenType::SCR_HOME)
        {
            startDcf(true);
        }
        else
        {
            modifyActiveElement(1);
            if (activeScreen == ScreenType::SCR_ALARM1 ||
                activeScreen == ScreenType::SCR_ALARM2 ||
                activeScreen == ScreenType::SCR_ALARM3)
            {
                size_t alarmNr = (size_t)activeScreen - (size_t)ScreenType::SCR_ALARM1;
                if (alarmNr < ALARM_NUMBER &&
                    alarms[alarmNr]->getActiveElement() == AlarmSetting::AS_ACTIVE &&
                    numOccured > 0)
                {
                    setScreen(SCR_HOME);
                    startAlarm(alarmNr);
                }
            }
        }
    }
    else if (b == &bMinus)
    {
        USART_DEBUG("minus button pressed: " << numOccured);
        if (activeScreen == ScreenType::SCR_HOME)
        {
            uint8_t n = lcd.getLinesNumber() == 1? 2 : 1;
            lcd.clear();
            lcd.init(n);
        }
        else
        {
            modifyActiveElement(-1);
        }
    }

    returnToHome.resetTime();
}


void DigitalClock::onDcfBit (int16_t secondNr, size_t errorNr, bool bit)
{
    if (secondNr >= 0 && errorNr == 0)
    {
        dcfState = bit? DcfState::BIT_PLUS : DcfState::BIT_MINUS;
    }
    else
    {
        dcfState = DcfState::ERROR;
    }
    /*
    Do not update LCD here since this procedure is called from an interrupt
    */
}


void DigitalClock::onDcfTimeReceived (const ::tm & dt, const char * /*dayTimeStr*/)
{
    ::tm newDt = dt;
    long diff = ::mktime(&dayTime) - ::mktime(&newDt);
    long dur = rtc.getTimeSec() - dcfReceiverStartTime;

    char logLine[512];
    if (!dcfTimeReceived ||
            (::labs(diff) < DCF_TOLERANCE) ||
            (::labs(diff) > 3600 - DCF_TOLERANCE && ::labs(diff) < 3600 + DCF_TOLERANCE))
    {
        dcfState = DcfState::READY;
        lcd.clear();
        dcf.stop();
        ::sprintf(logLine, "DCF time = %02d:%02d:%02d, PREV-NEW = %ld, DURATION = %ld -> accepted", dt.tm_hour, dt.tm_min, dt.tm_sec, diff, dur);
        writeLogToSd(logLine);
        dayTime = dt;
        setTime();
        dcfTimeReceived = true;
    }
    else
    {
        dcfState = DcfState::INVALID;
        ::sprintf(logLine, "DCF time = %02d:%02d:%02d, PREV-NEW = %ld, DURATION = %ld -> ignored", dt.tm_hour, dt.tm_min, dt.tm_sec, diff, dur);
        writeLogToSd(logLine);
    }
}


bool DigitalClock::onStartSteaming (WavStreamer::SourceType s)
{
    if (s == WavStreamer::SourceType::SD_CARD && !sdCard.isCardInserted())
    {
        USART_DEBUG("SD Card is not inserted");
        return false;
    }
    pinAmpMute.setLow();
    sdCard.clearPort();
    pinSdPower.setHigh();
    pinAmpPower.setHigh();
    HAL_Delay(250);
    pinAmpMute.setHigh();
    return true;
}


void DigitalClock::onFinishSteaming ()
{
    pinAmpMute.setLow();
    pinSdPower.setLow();
    pinAmpPower.setLow();
}
