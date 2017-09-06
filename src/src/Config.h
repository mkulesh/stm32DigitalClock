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

#ifndef CONFIG_H_
#define CONFIG_H_

#include <cstring>

#include "StmPlusPlus/Devices/SdCard.h"

/**
 * @brief Template class providing operator () for converting a string argument
 *        to a value of type type T
 */
template <typename T, size_t size, const char * strings[] > class ConvertClass
{
public:
    /**
     * @brief Converts a string to an enumeration literal.
     *
     * @return True if conversion was successful.
     */
    bool operator() (const char * image, T& value) const
    {
        for (size_t i = 0; i < size; ++i)
        {
            if (::strcmp(image, strings[i]) == 0)
            {
                // everything's OK:
                value = static_cast<T>(i);
                return true;
            }
        }

        // not found:
        value = static_cast<T>(0);
        return false;
    }
}; // end ConvertClass


/**
 * @brief Template class providing operator () for converting type T argument
 *        to a string
 */
template <typename T, size_t size, const char * strings[] > class AsStringClass
{
public:
    /**
     * @brief Returns identifier for given enumeration value.
     */
    const char * & operator() (const T & val) const
    {
        if (val >= 0 && val < size)
        {
            return strings[val];
        }
        else
        {
            return strings[size];
        }
    }
};


/**
 * @brief A class providing the enumeration for configuration parameters
 */
class CfgParameter
{
public:
    /**
     * @brief Set of valid enumeration values
     */
    enum Type {
        ALARM1_ACTIVE = 0,
        ALARM1_HM     = 1,
        ALARM1_DAYS   = 2,
        ALARM1_SOUND  = 3,
        ALARM2_ACTIVE = 4,
        ALARM2_HM     = 5,
        ALARM2_DAYS   = 6,
        ALARM2_SOUND  = 7,
        ALARM3_ACTIVE = 8,
        ALARM3_HM     = 9,
        ALARM3_DAYS   = 10,
        ALARM3_SOUND  = 11,
        BRIGH_MANUAL  = 12,
        BRIGH_MANVAL  = 13,
        SOUND_VOLUME  = 14
    };

    /**
     * @brief Number of enumeration values
     */
    enum {
        size = 15
    };

    /**
     * @brief String representations of all enumeration values
     */
    static const char * strings[];

    /**
     * @brief the Convert() method
     */
    static ConvertClass<Type, size, strings> Convert;

    /**
     * @brief the AsString() method
     */
    static AsStringClass<Type, size, strings> AsString;

    static bool isAlarmActivePar (Type t)
    {
        return t == ALARM1_ACTIVE || t == ALARM2_ACTIVE || t == ALARM3_ACTIVE;
    }

    static bool isAlarmHmPar (Type t)
    {
        return t == ALARM1_HM || t == ALARM2_HM || t == ALARM3_HM;
    }

    static bool isAlarmDaysPar (Type t)
    {
        return t == ALARM1_DAYS || t == ALARM2_DAYS || t == ALARM3_DAYS;
    }

    static bool isAlarmSoundPar (Type t)
    {
        return t == ALARM1_SOUND || t == ALARM2_SOUND || t == ALARM3_SOUND;
    }
};


/**
 * @brief A class providing the configuration
 */
class Config
{
public:

    static const char SEPARATOR = '=';
    static const size_t MAX_LINE_LENGTH = 32;
    static const size_t ALARMS_NUMBER = 3;

    class Brightness
    {
    public:
        bool isManual;
        uint8_t manValue;

        void write (FIL* fp, CfgParameter::Type n1, CfgParameter::Type n2) const;
        void read (CfgParameter::Type par, const char * value);
        void dump () const;
    };

    class Alarm
    {
    public:
        bool isActive;
        int8_t hour;
        int8_t min;
        bool days[7];
        char sound[MAX_LINE_LENGTH + 1];

        void write (FIL* fp, CfgParameter::Type n1, CfgParameter::Type n2, CfgParameter::Type n3, CfgParameter::Type n4) const;
        void read (CfgParameter::Type par, const char * value);
        void dump (const char * name) const;
    };

    Config (StmPlusPlus::IOPin & _pinSdPower, StmPlusPlus::Devices::SdCard & _sdCard, const char * _fileName);

    inline const Brightness & getBrightness () const
    {
        return brightness;
    }

    inline void setBrightnessManual (bool m)
    {
        brightness.isManual = m;
        isChanged = true;
    }

    inline void setBrightnessManValue (uint8_t m)
    {
        brightness.manValue = m;
        isChanged = true;
    }

    inline const Alarm & getAlarm (size_t number) const
    {
        return alarms[number];
    }

    inline void setAlarmActive (size_t number, bool m)
    {
        alarms[number].isActive = m;
        isChanged = true;
    }

    inline void setAlarmHour (size_t number, int8_t m)
    {
        alarms[number].hour = m;
        isChanged = true;
    }

    inline void setAlarmMin (size_t number, int8_t m)
    {
        alarms[number].min = m;
        isChanged = true;
    }

    inline void setAlarmDay (size_t number, size_t day, int8_t m)
    {
        alarms[number].days[day] = m;
        isChanged = true;
    }

    uint32_t getSoundVolume() const
    {
        return soundVolume;
    }

    inline void flush ()
    {
        if (isChanged)
        {
            writeConfiguration();
        }
    }

    bool isAlarmActive () const;
    size_t getAlarmOccured (const ::tm & dayTime) const;
    bool writeConfiguration ();
    bool readConfiguration ();

private:

    // File handling
    const char * fileName;
    FIL cfgFile;
    StmPlusPlus::IOPin & pinSdPower;
    StmPlusPlus::Devices::SdCard & sdCard;

    // Data containers
    bool isChanged;
    Alarm alarms[ALARMS_NUMBER];
    Brightness brightness;
    uint32_t soundVolume;

    FRESULT writeFile (const char * fileName);
    FRESULT readFile (const char * fileName);
};


#endif
