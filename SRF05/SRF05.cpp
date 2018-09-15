/* Requirements for "SRF05.h" and "SRF05.cpp" to work:
 * In exactly one(!) translation unit there must be these lines in given order:
 *
 *   #define EI_ARDUINO_INTERRUPTED_PIN
 *   #include <EnableInterrupt.h>
 *
 *
 * All possible pins to use in the SRF05-Constructor
 * for the Arduino Mega2560:
 *
 *   External interrupts:   2,3, 18,19,20,21
 *	 Pin Change Interrupts: 10,11,12,13,14,15, 50,51,52,53, A8,A9,A10,A11,A12,A13,A14,A15
 *
 */

#include <Arduino.h>

#define LIBCALL_ENABLEINTERRUPT
#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>

#include "SRF05.h"

SRF05::SRF05(uint8_t pin, srfIsrPtr interruptHandler) {
	m_pin = pin;
	m_isrHandler = interruptHandler;

	m_pinOutReg = portOutputRegister(digitalPinToPort(pin));
	m_pinModeReg = portModeRegister(digitalPinToPort(pin));
	m_pinMask = digitalPinToBitMask(pin);

	pinMode(m_pin, OUTPUT);
	digitalWrite(m_pin, LOW);
}

void SRF05::handleInterrupt() {
	if(!m_pulseEndFound) {
		if(arduinoPinState != 0) { // LOW to HIGH (indicates pulse start)
			m_pulseStart = micros();
			m_pulseStartFound = true;
		} else { // HIGH to LOW (indicates pulse end)
			if(m_pulseStartFound) {
				m_pulseEnd = micros();
				m_pulseEndFound = true;
			}
		}
	}
}

void SRF05::startReading() {
	if(m_readingStarted) return;

	// Trigger a reading
	*m_pinModeReg |= m_pinMask; //Set pin mode to output
	*m_pinOutReg |= m_pinMask; // HIGH output
	delayMicroseconds(10);
	*m_pinOutReg &= ~m_pinMask; // LOW output
	// Save the trigger time
	m_readingStart = micros();
	// Setup interrupt handler to listen for an incoming pulse
	*m_pinModeReg &= ~m_pinMask; //Set pin mode to input
	enableInterrupt(m_pin, m_isrHandler, CHANGE);

	m_readingStarted = true;
}

void SRF05::waitForReadingEnd(uint16_t maxDistCm) {
	if(!m_readingStarted) {return;}

	m_dist = -1.0; // Nothing was found in specified range
	uint32_t maxPulseLength = maxDistCm * 58; //In microseconds
	while(!m_pulseStartFound && micros() - m_readingStart < SRF05_TIMEOUT_US);

	if(m_pulseStartFound) {
		while(!m_pulseEndFound && micros() - m_pulseStart <= maxPulseLength);

		if(m_pulseEndFound && m_pulseEnd - m_pulseStart <= maxPulseLength) {
			m_dist = (m_pulseEnd - m_pulseStart) * 0.01724;
		}
	} else {
		m_dist = -2.0; //No pulse was received
	}

	disableInterrupt(m_pin);
	m_pulseStartFound = false;
	m_pulseEndFound = false;

	m_readingStarted = false;
}

float SRF05::getDistance() {
	return m_dist;
}

bool SRF05::objectDetected() {
	return m_dist >= 0.0;
}

uint32_t SRF05::getPulseStartAppearance() {
	return m_pulseStart > m_readingStart ? (m_pulseStart - m_readingStart) : 0;
}

