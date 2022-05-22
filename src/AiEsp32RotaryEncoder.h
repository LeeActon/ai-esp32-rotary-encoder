// AiEsp32RotaryEncoder.h
// based on https://github.com/marcmerlin/IoTuz code - extracted and modified Encoder code

#ifndef _AIESP32ROTARYENCODER_h
#define _AIESP32ROTARYENCODER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Rotary Encocer
#define AIESP32ROTARYENCODER_DEFAULT_A_PIN 25
#define AIESP32ROTARYENCODER_DEFAULT_B_PIN 26
#define AIESP32ROTARYENCODER_DEFAULT_VCC_PIN -1
#define AIESP32ROTARYENCODER_DEFAULT_STEPS 2

class AiEsp32RotaryEncoder
{

protected:
#if defined(ESP8266)
#else
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif
	volatile long encoder0Pos = 0;

	volatile int8_t lastMovementDirection = 0; //1 right; -1 left
	volatile unsigned long lastMovementAt = 0;
	unsigned long rotaryAccelerationCoef = 150;

	bool _circleValues = false;
	bool isEnabled = true;
	bool wasTimeouted = false;

	uint8_t encoderAPin = AIESP32ROTARYENCODER_DEFAULT_A_PIN;
	uint8_t encoderBPin = AIESP32ROTARYENCODER_DEFAULT_B_PIN;
	uint8_t encoderVccPin = AIESP32ROTARYENCODER_DEFAULT_VCC_PIN;
	long encoderSteps = AIESP32ROTARYENCODER_DEFAULT_STEPS;

	long _minEncoderValue = -1 << 15;
	long _maxEncoderValue = 1 << 15;

	uint8_t old_AB;
	long lastReadEncoder0Pos;
	bool previous_butt_state;

	int8_t enc_states[16] =
		{
		    // BA BA - Old -- > New - Zero indicates switch closure
		 0,	// 00 00 -       AB --> AB      - No change
		-1,	// 00 01 -       AB --> B       - Counter Clockwise (A leads B )
		 1,	// 00 10 -       AB --> A       - Clockwise (B leads A)
		 0,	// 00 11 -       AB --> Nothing - Invalid
		 1,	// 01 00 -        B --> AB      - Clockwise (B leads A)
		 0,	// 01 01 -        B --> B       - No change
		 0,	// 01 10 -        B --> A       - Invalid
		-1,	// 01 11 -        B --> Nothing - Counter Clockwise (A leads B)
		-1,	// 10 00 -        A --> AB      - Counter Clockwise (A leads B)
		 0,	// 10 01 -        A --> B       - Invalid
		 0,	// 10 10 -        A --> A       - No change
		 1,	// 10 11 -        A --> Nothing - Clockwise (B leads A)
		 0,	// 11 00 - Nothing --> AB       - Invalid
		 1,	// 11 01 - Nothing --> B        - Clockwise (B leads A)
		-1,	// 11 10 - Nothing --> A        - Counter Clockwise (A leads B)
		 0	// 11 11 - Nothing --> Nothing  - No change
		};

public:
	AiEsp32RotaryEncoder(
		uint8_t encoderAPin = AIESP32ROTARYENCODER_DEFAULT_A_PIN,
		uint8_t encoderBPin = AIESP32ROTARYENCODER_DEFAULT_B_PIN,
		uint8_t encoderVccPin = AIESP32ROTARYENCODER_DEFAULT_VCC_PIN,
		uint8_t encoderSteps = AIESP32ROTARYENCODER_DEFAULT_STEPS);
	void setBoundaries(long minValue = -100, long maxValue = 100, bool circleValues = false);
#if defined(ESP8266)
	ICACHE_RAM_ATTR void readEncoder_ISR();
#else
	void IRAM_ATTR readEncoder_ISR();
#endif

	void setup(void (*ISR_callback)(void));
	void begin();
	void reset(long newValue = 0);
	void enable();
	void disable();
	long readEncoder();
	void setEncoderValue(long newValue);
	long encoderChanged();
	unsigned long getAcceleration() { return this->rotaryAccelerationCoef; }
	void setAcceleration(unsigned long acceleration) { this->rotaryAccelerationCoef = acceleration; }
	void disableAcceleration() { setAcceleration(0); }
};
#endif
