#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23x17.h>
#include "vallox_driver.h"

const int KDefaultSpeed = 1;

ValloxDriver::ValloxDriver() : m_power(false), m_speedIncoming(0), m_speedExhaust(0), m_lastUsedSpeed(-1), m_takkatehostus(false)
{
    
}
ValloxDriver::~ValloxDriver()
{
}

void ValloxDriver::begin(Adafruit_MCP23X17* pMcp)
{
    m_pMcp = pMcp;
    for (int i=0; i<8; i++) {
        m_pMcp->pinMode(i, OUTPUT);
        m_pMcp->digitalWrite(i, HIGH);
    }
}
void ValloxDriver::setPower(bool power)
{
    if (power == m_power) // power state did not change, exit
        return;
    
    m_power = power;
    if (power) {
        if (m_lastUsedSpeed > 0)
            setSpeed(m_lastUsedSpeed); // use last saved speed
        else
            setSpeed(KDefaultSpeed); // use default speed
    } else {
        setSpeedIncoming(0);
        setSpeedExhaust(0);
    }
}
void ValloxDriver::setSpeed(int speed) // set speed, range 0 to 4
{
    if (speed < 0 || speed > 4)
        return;
    
    // setting speed reset takkatehostus to off
    m_takkatehostus = false;

    setSpeedIncoming(speed);
    setSpeedExhaust(speed);
}
void ValloxDriver::setTakkatehostus(bool takkatehostus) // enable or disable takkatehostus
{
    if (takkatehostus == m_takkatehostus) // already set, do nothing
        return;
    
    m_takkatehostus = takkatehostus;
    
    if (m_takkatehostus) {
        int speed = m_speedIncoming;
        setSpeedIncoming(4); // set speed to max
        setSpeedExhaust(0); // turn off exhaust
        m_lastUsedSpeed = speed; // save last used speed
    }
    else
    {
        setSpeed(m_lastUsedSpeed);
    }
}

bool ValloxDriver::hasPower() const
{
    return m_power;
}
int ValloxDriver::speed() const
{
    return m_speedIncoming;
}
bool ValloxDriver::hasTakkatehostus() const
{
    return m_takkatehostus;
}

void ValloxDriver::setSpeedIncoming(int speed)
{
    // Set incoming speed, range 0 to 4. relays 1-4.
    switch (speed)
    {
    case 0:
        setRelay(0, false);
        setRelay(1, false);
        setRelay(2, false);
        setRelay(3, false);
        break;
    case 1:
        setRelay(1, false);
        setRelay(2, false);
        setRelay(3, false);
        delay(100);
        setRelay(0, true);
        break;
    case 2:
        setRelay(0, false);
        setRelay(2, false);
        setRelay(3, false);
        delay(100);
        setRelay(1, true);
        break;
    case 3:
        setRelay(0, false);
        setRelay(1, false);
        setRelay(3, false);
        delay(100);
        setRelay(2, true);
        break;
    case 4:
        setRelay(0, false);
        setRelay(1, false);
        setRelay(2, false);
        delay(100);
        setRelay(3, true);
        break;
    default:
        // invalid speed, do nothing
        return;
    }
    
    m_speedIncoming = speed;
    if (m_speedIncoming > 0)
        m_lastUsedSpeed = m_speedIncoming; // save as last used speed
}
void ValloxDriver::setSpeedExhaust(int speed)
{
    // Set exhaust speed, range 0 to 4. relays 5-8.
    switch (speed)
    {
    case 0: // OFF
        setRelay(4, false);
        setRelay(5, false);
        setRelay(6, false);
        setRelay(7, false);
        break;
    case 1:
        setRelay(5, false);
        setRelay(6, false);
        setRelay(7, false);
        delay(100);
        setRelay(4, true);
        break;
    case 2:
        setRelay(4, false);
        setRelay(6, false);
        setRelay(7, false);
        delay(100);
        setRelay(5, true);
        break;
    case 3:
        setRelay(4, false);
        setRelay(5, false);
        setRelay(7, false);
        delay(100);
        setRelay(6, true);
        break;
    case 4:
        setRelay(4, false);
        setRelay(5, false);
        setRelay(6, false);
        delay(100);
        setRelay(7, true);
        break;
    default:
        // invalid speed, do nothing
        return;
    }
    m_speedExhaust = speed;
}

int ValloxDriver::speedIncoming() const
{
    return m_speedIncoming;
}
int ValloxDriver::speedExhaust() const
{
    return m_speedExhaust;
}

// setRelay on/off, relayIndex starts from 0
void ValloxDriver::setRelay(uint8_t index, bool state)
{
    // works as inverted
    if (state)
        m_pMcp->digitalWrite(index, LOW);
    else
        m_pMcp->digitalWrite(index, HIGH);
}