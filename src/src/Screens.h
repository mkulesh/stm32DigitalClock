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

#ifndef SCREENS_H_
#define SCREENS_H_

#include "StmPlusPlus/StmPlusPlus.h"
#include "Config.h"

/**
 * @brief Enumerated labels for the months.
 */
enum class Months
{
    JANUARY   = 0,
    FEBRUARY  = 1,
    MARCH     = 2,
    APRIL     = 3,
    MAY       = 4,
    JUNE      = 5,
    JULY      = 6,
    AUGUST    = 7,
    SEPTEMBER = 8,
    OCTOBER   = 9,
    NOVEMBER  = 10,
    DECEMBER  = 11
};


enum class DcfState
{
    NONE = 0,
    ERROR = 1,
    BIT_PLUS = 2,
    BIT_MINUS = 3,
    READY = 4,
    INVALID = 5
};

/**
 * @brief Interface for display data provider.
 */
class DisplayDataProvider
{
public:
    virtual ~DisplayDataProvider () { /* empty */ }
    virtual const ::tm & getDayTime () const = 0;
    virtual bool isAlarmActive () const = 0;
    virtual bool isActiveElementVisible () const = 0;
    virtual DcfState getDcfState() const = 0;
    virtual float getTemperature () const = 0;
    virtual int getBrightnessValue () const = 0;
};


/**
 * @brief Basic class for a display screen.
 */
class Screen
{
public:
    virtual ~Screen () { /* empty */ }
    virtual void setFirst() = 0;
    virtual void setNext() = 0;
    virtual void fillLine (int line, const DisplayDataProvider * dataProvider, char * dest) = 0;
};


/**
 * @brief Class describing the home screen.
 */
class HomeScreen : public Screen
{
public:
    HomeScreen () {};
    void setFirst () {};
    void setNext () {};
    void fillLine (int line, const DisplayDataProvider * dataProvider, char * dest);
};


/**
 * @brief Class describing the state of time setting screen.
 */
class TimeSetting : public Screen
{
public:
    enum Element
    {
        TS_DAY = 0,
        TS_MONTH = 1,
        TS_YEAR = 2,
        TS_HOUR = 3,
        TS_MIN = 4,
        TS_SEC = 5
    };
    
    TimeSetting (): activeElement(TS_DAY) {};
    inline void setFirst () { activeElement = TS_DAY; };
    inline void setActiveElement (Element element) { activeElement = element; };
    void setNext ();
    void fillLine (int line, const DisplayDataProvider * dataProvider, char * dest);
    void modifyValue (::tm & dayTime, int s);
    
private:

    static const unsigned char elementPos[6][3];
    Element activeElement;
};


/**
 * @brief Class describing the state of brightness setting screen.
 */
class BrightnessSetting : public Screen
{
public:
    enum Element
    {
        BS_MODE = 0,
        BS_MAN_VALUE = 1
    };
    
    BrightnessSetting (Config & _config);
    void setFirst ();
    void setNext ();
    void fillLine (int line, const DisplayDataProvider * dataProvider, char * dest);
    void modifyValue (int s);
        
private:

    Config & config;
    static const unsigned char elementPos[2][3];
    Element activeElement;
};


/**
 * @brief Class describing the state of alarm setting screen.
 */
class AlarmSetting : public Screen
{
public:
    enum Element
    {
        AS_ACTIVE = 0,
        AS_HOUR = 1,
        AS_MIN = 2,
        AS_DAY1 = 3,
        AS_DAY2 = 4,
        AS_DAY3 = 5,
        AS_DAY4 = 6,
        AS_DAY5 = 7,
        AS_DAY6 = 8,
        AS_DAY7 = 9
    };
    
    AlarmSetting (size_t _number, Config & _config);
    void setFirst ();
    void setNext ();
    void fillLine (int line, const DisplayDataProvider * dataProvider, char * dest);
    void modifyValue (int s);

    inline Element getActiveElement() const
    {
        return activeElement;
    }

private:

    Config & config;
    static const unsigned char elementPos[10][3];
    Element activeElement;
    static const char * activeString[2];
    size_t number;
};


#endif
