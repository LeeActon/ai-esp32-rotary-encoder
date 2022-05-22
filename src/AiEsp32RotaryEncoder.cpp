// based on https://github.com/marcmerlin/IoTuz code - extracted and modified Encoder code
//
//

#if defined(ESP8266)
#else
#include "esp_log.h"
#define LOG_TAG "AiEsp32RotaryEncoder"
#endif

#include "AiEsp32RotaryEncoder.h"

#if defined(ESP8266)
ICACHE_RAM_ATTR void AiEsp32RotaryEncoder::readEncoder_ISR()
#else
void IRAM_ATTR AiEsp32RotaryEncoder::readEncoder_ISR()
#endif
{

	unsigned long now = millis();
#if defined(ESP8266)
#else
	portENTER_CRITICAL_ISR(&(this->mux));
#endif
	if (this->isEnabled)
	{
		// code from https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino/
		/**/
		this->old_AB <<= 2; //remember previous state

		int8_t ENC_PORT = ((digitalRead(this->encoderBPin)) ? (1 << 1) : 0) | ((digitalRead(this->encoderAPin)) ? (1 << 0) : 0);

		this->old_AB |= (ENC_PORT & 0x03); //add current state

		//this->encoder0Pos += ( this->enc_states[( this->old_AB & 0x0f )]);
		int8_t currentDirection = (this->enc_states[(this->old_AB & 0x0f)]); //-1,0 or 1

		if (currentDirection != 0)
		{
			long prevRotaryPosition = this->encoder0Pos / this->encoderSteps;
			this->encoder0Pos += currentDirection;
			long newRotaryPosition = this->encoder0Pos / this->encoderSteps;

			if (newRotaryPosition != prevRotaryPosition && rotaryAccelerationCoef > 1)
			{
				//additional movements cause acceleration?
				// at X ms, there should be no acceleration.
				unsigned long accelerationLongCutoffMillis = 200;
				// at Y ms, we want to have maximum acceleration
				unsigned long accelerationShortCutffMillis = 4;

				// compute linear acceleration
				if (currentDirection == lastMovementDirection &&
					currentDirection != 0 &&
					lastMovementDirection != 0)
				{
					// ... but only of the direction of rotation matched and there
					// actually was a previous rotation.
					unsigned long millisAfterLastMotion = now - lastMovementAt;

					if (millisAfterLastMotion < accelerationLongCutoffMillis)
					{
						if (millisAfterLastMotion < accelerationShortCutffMillis)
						{
							millisAfterLastMotion = accelerationShortCutffMillis; // limit to maximum acceleration
						}
						if (currentDirection > 0)
						{
							this->encoder0Pos += rotaryAccelerationCoef / millisAfterLastMotion;
						}
						else
						{
							this->encoder0Pos -= rotaryAccelerationCoef / millisAfterLastMotion;
						}
					}
				}
				this->lastMovementAt = now;
				this->lastMovementDirection = currentDirection;
			}

			//respect limits
			if (this->encoder0Pos > (this->_maxEncoderValue))
				this->encoder0Pos = this->_circleValues ? this->_minEncoderValue : this->_maxEncoderValue;
			if (this->encoder0Pos < (this->_minEncoderValue))
				this->encoder0Pos = this->_circleValues ? this->_maxEncoderValue : this->_minEncoderValue;
		}
	}
#if defined(ESP8266)
#else
	portEXIT_CRITICAL_ISR(&(this->mux));
#endif
}

AiEsp32RotaryEncoder::AiEsp32RotaryEncoder(uint8_t encoder_APin, uint8_t encoder_BPin, uint8_t encoder_VccPin, uint8_t encoderSteps)
{
	this->old_AB = 0;

	this->encoderAPin = encoder_APin;
	this->encoderBPin = encoder_BPin;
	this->encoderVccPin = encoder_VccPin;
	this->encoderSteps = encoderSteps;

#if defined(ESP8266)
	pinMode(this->encoderAPin, INPUT_PULLUP);
	pinMode(this->encoderBPin, INPUT_PULLUP);
#else
	pinMode(this->encoderAPin, INPUT_PULLDOWN);
	pinMode(this->encoderBPin, INPUT_PULLDOWN);
#endif
}

void AiEsp32RotaryEncoder::setBoundaries(long minEncoderValue, long maxEncoderValue, bool circleValues)
{
	this->_minEncoderValue = minEncoderValue * this->encoderSteps;
	this->_maxEncoderValue = maxEncoderValue * this->encoderSteps;

	this->_circleValues = circleValues;
}

long AiEsp32RotaryEncoder::readEncoder()
{
	return (this->encoder0Pos / this->encoderSteps);
}

bool AiEsp32RotaryEncoder::readEncoder(long *pEncoderPos)
{
	bool fNew = false;
	portENTER_CRITICAL(&(this->mux));
	long curPos = (this->encoder0Pos / this->encoderSteps);
	if (curPos != this->lastReadEncoder0Pos)
		{
		this->lastReadEncoder0Pos = _encoder0Pos;
		fNew = true
		}
	portEXIT_CRITICAL(&(this->mux));
	return fNew;
}

void AiEsp32RotaryEncoder::setEncoderValue(long newValue)
{
	reset(newValue);
}

long AiEsp32RotaryEncoder::encoderChanged()
{
	long _encoder0Pos = readEncoder();
	long encoder0Diff = _encoder0Pos - this->lastReadEncoder0Pos;

	this->lastReadEncoder0Pos = _encoder0Pos;

	return encoder0Diff;
}

void AiEsp32RotaryEncoder::setup(void (*ISR_callback)(void))
{
	attachInterrupt(digitalPinToInterrupt(this->encoderAPin), ISR_callback, CHANGE);
	attachInterrupt(digitalPinToInterrupt(this->encoderBPin), ISR_callback, CHANGE);
}

void AiEsp32RotaryEncoder::begin()
{
	this->lastReadEncoder0Pos = 0;
	if (this->encoderVccPin >= 0)
	{
		pinMode(this->encoderVccPin, OUTPUT);
		digitalWrite(this->encoderVccPin, 1); //Vcc for encoder
	}
}

void AiEsp32RotaryEncoder::reset(long newValue_)
{
	portENTER_CRITICAL(&(this->mux));
	newValue_ = newValue_ * this->encoderSteps;
	this->encoder0Pos = newValue_;
	if (this->encoder0Pos > this->_maxEncoderValue)
		this->encoder0Pos = this->_circleValues ? this->_minEncoderValue : this->_maxEncoderValue;
	if (this->encoder0Pos < this->_minEncoderValue)
		this->encoder0Pos = this->_circleValues ? this->_maxEncoderValue : this->_minEncoderValue;
	this->lastReadEncoder0Pos = this->encoder0Pos;
	portEXIT_CRITICAL(&(this->mux));
}

void AiEsp32RotaryEncoder::enable()
{
	this->isEnabled = true;
}
void AiEsp32RotaryEncoder::disable()
{
	this->isEnabled = false;
}
