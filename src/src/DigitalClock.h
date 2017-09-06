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

#ifndef DIGITALCLOCK_H_
#define DIGITALCLOCK_H_

#include "StmPlusPlus/Devices/Ssd.h"
#include "StmPlusPlus/Devices/Button.h"
#include "StmPlusPlus/Devices/Lcd_DOGM162.h"
#include "StmPlusPlus/Devices/Dac_MCP49x1.h"
#include "StmPlusPlus/Devices/Dcf77.h"

#include "StmPlusPlus/WavStreamer.h"
#include "StmPlusPlus/PiezoAlarm.h"

#include "Screens.h"

using namespace StmPlusPlus;

class DigitalClock : public
    Devices::Button::EventHandler,
    WavStreamer::EventHandler,
    Devices::DcfReceiver::EventHandler,
    DisplayDataProvider
{
public:

    const float MAIN_VOLTAGE = 3.3;
    static const size_t BUTTONS_NUMBER = 4;
    static const size_t SCR_NUMBER = 6;
    static const size_t ALARM_NUMBER = 3;
    static const size_t TEMPERATURE_TRIALS = 10;
    const char * LOG_FILE_NAME = "dc.log";

    enum ScreenType
    {
        SCR_HOME = 0,           // home screen
        SCR_TIME_SETTING = 1,   // time setting screen
        SCR_BRIGHTNESS = 2,     // brightness setting screen
        SCR_ALARM1 = 3,         // alarm setting screen
        SCR_ALARM2 = 4,         // alarm setting screen
        SCR_ALARM3 = 5          // alarm setting screen
    };

    DigitalClock ();

    virtual ~DigitalClock () { /* empty */ }

    inline RealTimeClock * getRtc ()
    {
        return &rtc;
    }

    inline Devices::SdCard & getSdCard ()
    {
        return sdCard;
    }

    inline void onTim3Interrupt ()
    {
        wavStreamer.onSample();
    }

    inline void onTim4Interrupt ()
    {
        dcf.onSample();
    }

    inline const ::tm & getDayTime () const
    {
        return dayTime;
    }

    inline bool isActiveElementVisible () const
    {
        return activeElementVisible;
    }

    inline DcfState getDcfState () const
    {
        return dcfState;
    }

    inline float getTemperature () const
    {
        return temperature;
    }

    inline int getBrightnessValue () const
    {
        return brightnessValue;
    }

    inline void rtcToDayTime ()
    {
        time_t timer = rtc.getTimeSec();
        ::tm * dt = ::gmtime(&timer);
        dayTime = *dt;
    }

    void run ();

protected:

    void periodic ();
    void resetEventTime ();
    void setScreen (ScreenType scr);
    void updateBrightness ();
    void updateLcd (bool changeActiveElement);
    void updateSsd ();
    void modifyActiveElement (int s);
    bool isAlarmActive () const;
    void setTime ();
    void measureTemperature ();
    void updateLoggingState ();
    void updateSdCardState ();
    void startAlarm (size_t n);
    bool writeLogToSd (const char *);

    virtual void onButtonPressed (const Devices::Button * b, uint32_t numOccured);
    virtual void onDcfBit (int16_t secondNr, size_t errorNr, bool bit);
    virtual void onDcfTimeReceived (const ::tm & dt, const char * dayTimeStr);
    virtual bool onStartSteaming (WavStreamer::SourceType s);
    virtual void onFinishSteaming ();

private:

    // Logging
    UsartLogger log;
    IOPin pinUsbVoltage;

    // RTC
    RealTimeClock rtc;

    // HMI
    IOPin pinHmiPower;
    PeriodicalEvent eventLedToggle, eventSecToggle;
    Spi spiHmi;

    // SSD
    IOPin pinSsdCs;
    Devices::Ssd_74HC595_SPI ssd;
    bool ssdDots[4]{false, false, false, false};

    // LCD
    IOPin pinLcdCs, pinLcdRs;
    Devices::Lcd_DOGM162_SPI lcd;
    char lcdString[32];

    // DAC
    IOPin pinHmiBrightnessCs;
    Devices::Dac_MCP49x1 hmiBrightness;
    AnalogToDigitConverter adcBrightness;

    // DCF77
    IOPin pinDcfInput, pinDcfPower;
    Devices::DcfReceiver dcf;
    volatile DcfState dcfState;
    time_t dcfReceiverStartTime;
    bool dcfTimeReceived;

    // SD card
    IOPin pinSdPower, pinSdDetect;
    IOPort portSd1, portSd2;
    Devices::SdCard sdCard;
    bool sdCardInserted;

    // Configuration
    Config config;

    // Sound
    IOPin pinAmpPower, pinAmpMute;
    Spi spiWav;
    IOPin pinLeftChannel;
    IOPin pinRightChannel;
    IOPin pinWavSample;
    WavStreamer wavStreamer;

    // Piezo element
    PiezoAlarm piezoAlarm;

    // Buttons
    Devices::Button bMode, bActiveElement, bPlus, bMinus;
    Devices::Button * buttons[BUTTONS_NUMBER];

    // Screens
    HomeScreen homeScreen;
    TimeSetting timeSetting;
    BrightnessSetting brightnessSetting;
    Screen * screens[SCR_NUMBER];
    ScreenType activeScreen;
    bool activeElementVisible;
    PeriodicalEvent activeElementToggle, returnToHome;

    // Alarms
    const AlarmSetting * alarms[ALARM_NUMBER];
    AlarmSetting alarmSetting1, alarmSetting2, alarmSetting3;

    // Date and time
    ::tm dayTime;

    // Temperature
    AnalogToDigitConverter adcTemperature;
    float temperature;
    size_t temperatureTrial;
    float temperatureArr[TEMPERATURE_TRIALS];

    // auto brightness value
    int brightnessValue;

    // Interrupt priorities
    InterruptPriority irqPrioSd;
    InterruptPriority irqPrioRtc;
    InterruptPriority irqPrioDcf;
    InterruptPriority irqPrioWav;
};

#endif
