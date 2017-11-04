/* 
 *  Firmware for an analog speed/cadence indicator built from an analog voltage meter, a Sigma speed/cadence sensor and ATtiny85
 *  
 *  Various parts adapted from: 
 * - https://embeddedthoughts.com/2016/06/06/attiny85-introduction-to-pin-change-and-timer-interrupts/ 
 * - https://learn.sparkfun.com/tutorials/h2ohno/low-power-attiny
 * - http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
 * 
 */

#include <avr/sleep.h> //Needed for sleep_mode

//#define SPEED_MODE

// Speed-specific constants:
const unsigned long WHEEL_CIRCUMFERENCE = 2224; // [mm]
const unsigned int MAX_SPEED = 11;          // 11 mm/ms = 39.6 km/h 

// Cadence-specific constants:
const unsigned long MINUTE_MILLIS = 60000;  // [ms]
const unsigned int MAX_CADENCE = 120;       // [RPM]

// Generic constants:
const unsigned int MAX_OUTPUT_VALUE = 255;  // Less than 255 if the meter's max voltage is less than 3.3V
const unsigned int PIN_METER = PB0;         // PWM pin on ATtiny85
const unsigned int PIN_REGULATOR = PB1;     // Pin controls the voltage regulator
const unsigned int PIN_SENSOR = PB3;        // Input pin for sensor signal (corresponds to PCINT3)
const unsigned int PCINT_SENSOR = PCINT3; 
const unsigned int MAX_DT = 2500;           // [ms]
const unsigned int MIN_DT = 100;            // [ms]  

// Volatile variables:
volatile bool signalDetected = false;
volatile unsigned long signalDetectedMillis = 0; //[ms]

// Current state variables:
unsigned long t0 = 0;  // [ms]
int prevOutputValue = 0;

#if defined SPEED_MODE
  const unsigned long OUTPUT_COEFF = MAX_OUTPUT_VALUE*round(1.0*WHEEL_CIRCUMFERENCE/MAX_SPEED);  
#else 
  const unsigned long OUTPUT_COEFF = MAX_OUTPUT_VALUE*MINUTE_MILLIS/MAX_CADENCE;
#endif


void setup() {
  pinMode(PIN_METER, OUTPUT);
  pinMode(PIN_REGULATOR, OUTPUT);
  digitalWrite(PIN_REGULATOR, LOW);
  pinMode(PIN_SENSOR, INPUT_PULLUP);
  
  ADCSRA &= ~(1<<ADEN); // Disable ADC, saves ~230uA
  GIMSK |= (1 << PCIE);  // Pin change interrupt enable
  PCMSK |= (1 << PCINT_SENSOR); // Pin change interrupt enabled for sensor pin

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power down everything, wake up from WDT
  sleep_enable();
  setup_watchdog(9);
  
  sei();  // Enable interrupts
}

void loop() {
  if ( signalDetected ) {
    signalDetected = false;
    if ( t0 > 0 ) {
      unsigned long dt = signalDetectedMillis-t0;  // [ms]
      if ( dt > MIN_DT ) {
        int newOutputValue = min( MAX_OUTPUT_VALUE, int( dt < MAX_DT ? OUTPUT_COEFF/dt : 0 ) );
        int outputValue = (prevOutputValue + newOutputValue)/2;
        digitalWrite( PIN_REGULATOR, HIGH );
        analogWrite( PIN_METER, outputValue );
        prevOutputValue = newOutputValue;
      }
    }
    t0 = signalDetectedMillis;
  } else if ( millis()-t0 > MAX_DT ) {
    analogWrite( PIN_METER, 0 );
    digitalWrite( PIN_REGULATOR, LOW );
    sleep_mode(); //Go to sleep!
  }
  delay(100);
}


void setup_watchdog(int timerPrescaler) {
  // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
  // 6=1sec, 7=2sec, 8=4sec, 9=8sec
  
  if (timerPrescaler > 9 ) timerPrescaler = 9; //Correct incoming amount if need be

  byte bb = timerPrescaler & 7;
  if (timerPrescaler > 7) bb |= (1 << 5); //Set the special 5th bit if necessary

  //This order of commands is important and cannot be combined
  MCUSR &= ~(1 << WDRF); //Clear the watch dog reset
  WDTCR |= (1 << WDCE) | (1 << WDE); //Set WD_change enable, set WD enable
  WDTCR = bb; //Set new watchdog timeout value
  WDTCR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}


ISR(PCINT0_vect) {
  if ((PINB & (1 << PIN_SENSOR)) ) { // We're interested only in rising edges
    signalDetectedMillis = millis();
    signalDetected = true;
  } 
}

//This runs each time the watch dog wakes us up from sleep
ISR(WDT_vect) {
  // Do nothing
}
