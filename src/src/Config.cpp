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

#include <cstdlib>
#include <cctype>

#include <Config.h>

using namespace StmPlusPlus;

#define USART_DEBUG_MODULE "CONF: "

/************************************************************************
 * Class ConfigurationParametes
 ************************************************************************/

const char * CfgParameter::strings[] = {
    "ALARM1_ACTIVE",
    "ALARM1_HM",
    "ALARM1_DAYS",
    "ALARM1_SOUND",
    "ALARM2_ACTIVE",
    "ALARM2_HM",
    "ALARM2_DAYS",
    "ALARM2_SOUND",
    "ALARM3_ACTIVE",
    "ALARM3_HM",
    "ALARM3_DAYS",
    "ALARM3_SOUND",
    "BRIGH_MANUAL",
    "BRIGH_MANVAL",
    "SOUND_VOLUME",
    "INVALID_PARAMETER"
};

ConvertClass<CfgParameter::Type, CfgParameter::size, CfgParameter::strings>
CfgParameter::Convert;

AsStringClass<CfgParameter::Type, CfgParameter::size, CfgParameter::strings>
CfgParameter::AsString;


/************************************************************************
 * Class Config::Brightness
 ************************************************************************/

void Config::Brightness::write (FIL* fp, CfgParameter::Type n1, CfgParameter::Type n2) const
{
    f_printf(fp, "%s %c %d\n", CfgParameter::AsString(n1), SEPARATOR, isManual);
    f_printf(fp, "%s %c %d\n", CfgParameter::AsString(n2), SEPARATOR, manValue);
}


void Config::Brightness::read (CfgParameter::Type par, const char * value)
{
    size_t len = ::strlen(value);
    if (par == CfgParameter::BRIGH_MANUAL && len > 0)
    {
        isManual = ::atoi(value);
    }
    else if (par == CfgParameter::BRIGH_MANVAL && len > 0)
    {
        manValue = ::atoi(value);
    }
}


void Config::Brightness::dump () const
{
    USART_DEBUG("  Brightness: manual=" << isManual << ", manual value=" << manValue);
}


/************************************************************************
 * Class Config::Alarm
 ************************************************************************/

void Config::Alarm::write (FIL* fp, CfgParameter::Type n1, CfgParameter::Type n2, CfgParameter::Type n3, CfgParameter::Type n4) const
{
    f_printf(fp, "%s %c %d\n",       CfgParameter::AsString(n1), SEPARATOR, isActive);
    f_printf(fp, "%s %c %02d%02d\n", CfgParameter::AsString(n2), SEPARATOR, hour, min);
    f_printf(fp, "%s %c ",           CfgParameter::AsString(n3), SEPARATOR);
    for (auto & d : days)
    {
        f_printf(fp, "%d", d);
    }
    f_printf(fp, "\n");
    f_printf(fp, "%s %c %s\n", CfgParameter::AsString(n4), SEPARATOR, sound);
}


void Config::Alarm::read (CfgParameter::Type par, const char * value)
{
    size_t len = ::strlen(value);
    if (CfgParameter::isAlarmActivePar(par) && len > 0)
    {
        isActive = ::atoi(value);
    }
    else if (CfgParameter::isAlarmHmPar(par) && len >= 4)
    {
        char tmp[3] = {0,0,0};
        ::memcpy(tmp, value + 0, 2);
        hour = ::atoi(tmp);
        ::memcpy(tmp, value + 2, 2);
        min = ::atoi(tmp);
    }
    else if (CfgParameter::isAlarmDaysPar(par) && len >= sizeof(days))
    {
        char tmp[2] = {0,0};
        for (size_t i = 0; i < sizeof(days); ++i)
        {
            tmp[0] = *(value + i);
            days[i] = ::atoi(tmp);
        }
    }
    else if (CfgParameter::isAlarmSoundPar(par) && len > 0)
    {
        ::strncpy(sound, value, MAX_LINE_LENGTH);
    }
}


void Config::Alarm::dump (const char * name) const
{
    USART_DEBUG("  " << name
        << ": active=" << isActive
        << ", hour=" << hour
        << ", min=" << min
        << ", days=" << days[0] << days[1] << days[2] << days[3] << days[4] << days[5] << days[6]
        << ", sound=" << sound);
}


/************************************************************************
 * Class Config
 ************************************************************************/

Config::Config (IOPin & _pinSdPower, Devices::SdCard & _sdCard, const char * _fileName):
    fileName(_fileName),
    pinSdPower(_pinSdPower),
    sdCard(_sdCard),
    isChanged(false)
{
    alarms[0] = {true,  7, 00, {false, true, true, true, true, true, false}};
    ::strncpy(alarms[0].sound, "alarm1.wav", MAX_LINE_LENGTH);

    alarms[1] = {true,  7, 30, {false, true, true, true, true, true, false}};
    ::strncpy(alarms[1].sound, "alarm2.wav", MAX_LINE_LENGTH);

    alarms[2] = {false, 8, 00, {false, true, true, true, true, true, false}};
    ::strncpy(alarms[2].sound, "alarm3.wav", MAX_LINE_LENGTH);

    brightness = {true,  20};
    soundVolume = 25;
}


bool Config::isAlarmActive () const
{
    for (auto & a : alarms)
    {
        if (a.isActive)
        {
            return true;
        }
    }
    return false;
}


size_t Config::getAlarmOccured (const ::tm & dayTime) const
{
    for (size_t i = 0; i < ALARMS_NUMBER; ++i)
    {
        const Alarm & a = alarms[i];
        if (a.isActive &&
            a.days[dayTime.tm_wday] &&
            a.hour == dayTime.tm_hour &&
            a.min == dayTime.tm_min &&
            dayTime.tm_sec < 5)
        {
            return i;
        }
    }
    return ALARMS_NUMBER;
}


bool Config::writeConfiguration ()
{
    USART_DEBUG("Writing configuration to file: " << fileName);

    sdCard.clearPort();
    pinSdPower.setHigh();
    HAL_Delay(250);

    if (sdCard.start(6) && sdCard.mountFatFs())
    {
        FRESULT res = writeFile(fileName);
        if (res != FR_OK)
        {
            USART_DEBUG("Can not write configuration: " << res);
        }
        else
        {
            USART_DEBUG("Configuration file successfully written");
            isChanged = false;
        }
    }

    sdCard.stop();
    pinSdPower.setLow();
    return true;
}


bool Config::readConfiguration ()
{
    USART_DEBUG("Reading configuration from file: " << fileName);

    sdCard.clearPort();
    pinSdPower.setHigh();
    HAL_Delay(250);

    if (sdCard.start(6) && sdCard.mountFatFs())
    {
        FRESULT res = readFile(fileName);
        if (res != FR_OK)
        {
            USART_DEBUG("Can not read configuration: " << res);
        }
        else
        {
            USART_DEBUG("Configuration file successfully parsed:");
            alarms[0].dump("Alarm1");
            alarms[1].dump("Alarm2");
            alarms[2].dump("Alarm3");
            brightness.dump();
            isChanged = false;
        }
    }

    sdCard.stop();
    pinSdPower.setLow();
    return true;
}


FRESULT Config::writeFile (const char * fileName)
{
    FRESULT code = f_open(&cfgFile, fileName, FA_WRITE | FA_CREATE_ALWAYS);
    if (code != FR_OK)
    {
        return code;
    }

    // Alarms
    alarms[0].write(&cfgFile, CfgParameter::ALARM1_ACTIVE, CfgParameter::ALARM1_HM, CfgParameter::ALARM1_DAYS, CfgParameter::ALARM1_SOUND);
    alarms[1].write(&cfgFile, CfgParameter::ALARM2_ACTIVE, CfgParameter::ALARM2_HM, CfgParameter::ALARM2_DAYS, CfgParameter::ALARM2_SOUND);
    alarms[2].write(&cfgFile, CfgParameter::ALARM3_ACTIVE, CfgParameter::ALARM3_HM, CfgParameter::ALARM3_DAYS, CfgParameter::ALARM3_SOUND);
    brightness.write(&cfgFile, CfgParameter::BRIGH_MANUAL, CfgParameter::BRIGH_MANVAL);
    f_printf(&cfgFile, "%s %c %d\n", CfgParameter::AsString(CfgParameter::SOUND_VOLUME), SEPARATOR, soundVolume);
    f_close(&cfgFile);
    return FR_OK;
}


FRESULT Config::readFile (const char * fileName)
{
    FRESULT code = f_open(&cfgFile, fileName, FA_READ);
    if (code != FR_OK)
    {
        return code;
    }

    char buff[MAX_LINE_LENGTH + 1];
    char name[MAX_LINE_LENGTH + 1];
    char value[MAX_LINE_LENGTH + 1];

    while (f_gets(buff, MAX_LINE_LENGTH, &cfgFile) != 0)
    {
        ::memset(name, 0, sizeof(name));
        ::memset(value, 0, sizeof(value));
        char * ptr = &name[0];
        for (auto & c : buff)
        {
            if (c == 0)
            {
                break;
            }
            if (::isspace(c))
            {
                continue;
            }
            if (c == SEPARATOR)
            {
                ptr = &value[0];
                continue;
            }
            *ptr = c;
            ++ptr;
        }
        if (name[0] == 0 || value[0] == 0)
        {
            continue;
        }

        CfgParameter::Type par;
        if (!CfgParameter::Convert(name, par))
        {
            USART_DEBUG("Parameter " << name << " is not known");
            continue;
        }

        switch (par)
        {
        case CfgParameter::ALARM1_ACTIVE:
        case CfgParameter::ALARM1_HM:
        case CfgParameter::ALARM1_DAYS:
        case CfgParameter::ALARM1_SOUND:
            alarms[0].read(par, value);
            break;
        case CfgParameter::ALARM2_ACTIVE:
        case CfgParameter::ALARM2_HM:
        case CfgParameter::ALARM2_DAYS:
        case CfgParameter::ALARM2_SOUND:
            alarms[1].read(par, value);
            break;
        case CfgParameter::ALARM3_ACTIVE:
        case CfgParameter::ALARM3_HM:
        case CfgParameter::ALARM3_DAYS:
        case CfgParameter::ALARM3_SOUND:
            alarms[2].read(par, value);
            break;
        case CfgParameter::BRIGH_MANUAL:
        case CfgParameter::BRIGH_MANVAL:
            brightness.read(par, value);
            break;
        case CfgParameter::SOUND_VOLUME:
            soundVolume = ::atoi(value);
            break;
        }
    }
    f_close(&cfgFile);
    return FR_OK;
}
