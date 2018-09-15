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

#ifndef SRF05_H_
#define SRF05_H_

typedef void (*srfIsrPtr)(void);

//Abort reading if pulse didn't occur for this long
#define SRF05_TIMEOUT_US 1000 // Pulse usually appears after 760us

class SRF05 {
public:
	SRF05(uint8_t pin, srfIsrPtr interruptHandler);
	void handleInterrupt();

	void startReading();
	void waitForReadingEnd(uint16_t maxDistCm);
	float getDistance();
	bool objectDetected();

	uint32_t getPulseStartAppearance();
private:
	uint8_t m_pin;
	volatile uint8_t* m_pinOutReg;
	volatile uint8_t* m_pinModeReg;
	uint8_t m_pinMask;

	srfIsrPtr m_isrHandler;

	bool m_readingStarted = false;
	uint32_t m_readingStart = 0;

	volatile bool m_pulseStartFound = false, m_pulseEndFound = false;
	volatile uint32_t m_pulseStart, m_pulseEnd;

	float m_dist = -1.0;
};

#endif /* SRF05_H_ */
