// Adapted from STAR firmware
#define F_CPU 4800000UL

#define MODE_MOON 3
#define MODE_LOW 14
#define MODE_HIGH 255

#define FAST_PWM_START 8
#define WDT_TIMEOUT 2

#define BATTERY_LOW 130
#define BATTERY_CRITICAL 120

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>	
#include <avr/eeprom.h>
#include <avr/sleep.h>

#define STAR3_PIN 
#define PWM_PIN PB1
#define VOLTAGE_PIN PB2
#define ADC_CHANNEL 0x01
#define ADC_DIGITAL_DISABLED ADC1D
#define ADC_PRESCALER 0x06

#define PWM_LEVEL OCR0B
#define ALT_PWM_LEVEL OCR0A

uint8_t eepos = 0;
uint8_t eep[32];

static uint8_t modes[10];
volatile uint8_t modeIndex = 0;
uint8_t modeCount = 0;

uint8_t lowBatteryCounter = 0;

inline void saveMode(uint8_t level) 
{
	uint8_t oldpos = eepos;
	eepos=(eepos+1)&31;

	EEARL = eepos;
	EEDR = level;
	EECR = 32 + 4;
	EECR = 32 + 4 + 2;
	while(EECR & 2);
	// Erase the last mode
	EEARL = oldpos;
	EECR = 16 + 4; 
	EECR = 16 + 4 + 2;
}

inline void getMode() 
{
	eeprom_read_block(&eep, 0, 32);
	while ((eep[eepos] == 0xff) && (eepos < 32)) eepos++;
	if (eepos < 32) modeIndex = eep[eepos];
	else eepos=0;
}

inline void getNextMode() 
{
	modeIndex += 1;
	if (modeIndex > (modeCount - 1))
	{
		modeIndex = 0;
	}
}

inline void startWatchdog() 
{
	cli();
	wdt_reset();
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = (1<<WDTIE) | (1<<WDP2) | (1<<WDP0);
	sei();
}

inline void stopWatchdog()
{
	cli();
	wdt_reset();
	MCUSR &= ~(1<<WDRF);
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = 0x00;
	sei();
}

inline void startADC() 
{
	ADMUX  = (1 << REFS0) | (1 << ADLAR) | ADC_CHANNEL;
    DIDR0 |= (1 << ADC_DIGITAL_DISABLED);
	ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRESCALER;
}

inline void stopADC() 
{
	ADCSRA &= ~(1<<7);
}

inline void setOutput(uint8_t pwmLevel) 
{
	PWM_LEVEL = pwmLevel;
	ALT_PWM_LEVEL = pwmLevel;
}

inline uint8_t hasLowVoltage(uint8_t voltage) 
{
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	if (ADCH < voltage) 
	{
		if (++lowBatteryCounter > 8) 
		{
			lowBatteryCounter = 0;
			return 1;
		}
	} 
	else 
		lowBatteryCounter = 0;

	return 0;
}

ISR(WDT_vect) 
{
	static uint8_t ticks = 0;
	if (ticks < 255) ticks++;

	if (ticks == WDT_TIMEOUT) 
		saveMode(modeIndex);
}

inline void initPins() 
{
	PORTB = (1 << PB0) | (1 << PB4) | (1 << PB3);
	DDRB = (1 << PWM_PIN);
}

static void setupModes()
{
	modes[modeCount++] = MODE_MOON;
	modes[modeCount++] = MODE_LOW;
	modes[modeCount++] = MODE_HIGH;
}

inline void shutDown() 
{
	setOutput(0);
	stopWatchdog();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

inline void setFlash(uint8_t holdPWM) 
{
	setOutput(0);
	_delay_ms(250);
	setOutput(holdPWM);
	_delay_ms(500);
}

static void checkVoltageAndPWM() 
{
	uint8_t i = 0, holdPWM;
	while (1) 
	{
		if (hasLowVoltage(BATTERY_LOW)) 
		{
			if (modeIndex == 0 && ALT_PWM_LEVEL <= modes[modeIndex]) 
			{
				while (!hasLowVoltage(BATTERY_CRITICAL));
				i = 0;
				while (i++<10) 
					setFlash(holdPWM);

				shutDown();
			} 
			else 
			{
				holdPWM = ALT_PWM_LEVEL;
				i = 0;
				while (i++ < 3) 
					setFlash(holdPWM);

				if ((ALT_PWM_LEVEL >> 1) < modes[0]) 
				{
					setOutput(modes[0]);
					modeIndex = 0;
				} 
				else 
					setOutput(ALT_PWM_LEVEL >> 1);

				if (ALT_PWM_LEVEL < modes[modeIndex]) 
					modeIndex--;
			}
			_delay_ms(3000);
		}
		sleep_mode();
	}
}

inline void setPWMTiming(uint8_t currentLevel) 
{
	if (currentLevel > FAST_PWM_START) 
		TCCR0A = 0b00100011;
	else 
		TCCR0A = 0b00100001;
	TCCR0B = 0x01;
}

int main(void)
{	
	initPins();	
	startADC();
	ACSR |= (1<<7);
	setupModes();
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	getMode();
	if (modeIndex&0x10) 
	{
		modeIndex &= 0x0f;
		getNextMode();
	}
	saveMode(modeIndex|0x10);
	startWatchdog();
	
	setPWMTiming(modes[modeIndex]);
	
	setOutput(modes[modeIndex]);
	checkVoltageAndPWM();

    return 0;
}