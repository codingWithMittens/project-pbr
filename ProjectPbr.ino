#include "Arduino.h"
#include "Adafruit_GPS.h"
#include "SoftwareSerial.h"
#include "TinyGPS++.h"

// Not configurable:
// GPS_RX = 18
// GPS_TX = 19

int const GREEN_LEFT = 2;
int const RED_LEFT = 3;
int const YELLOW_LEFT = 4;
int const GREEN_RIGHT = 8;
int const RED_RIGHT = 9;
int const YELLOW_RIGHT = 10;

int const RANGE_LEFT = 7;
int const RANGE_RIGHT = 6;

int const STOP_DIST_CM = 25;
int const SLOW_DIST_CM = 50;
int const READ_FREQ_MS = 300;

boolean usingInterrupt = false;

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

Adafruit_GPS GPS(&Serial1);
#define GPSECHO  true // debug?

TinyGPSPlus gps;

class Ultrasonic {
	public:
		Ultrasonic(int pin);
    void DistanceMeasure(void);
		long microsecondsToCentimeters(void);
	private:
		int _pin; //pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
    long duration; // the Pulse time received;
};

Ultrasonic::Ultrasonic(int pin) {
	_pin = pin;
}

/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void) {
  pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_pin,LOW);
	pinMode(_pin,INPUT);
	duration = pulseIn(_pin,HIGH);
}

/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void) {
	return duration/29/2;
}

Ultrasonic ultrasonic_left(RANGE_LEFT);
Ultrasonic ultrasonic_right(RANGE_RIGHT);

void setup() {
	pinMode(GREEN_LEFT, OUTPUT);
	pinMode(RED_LEFT, OUTPUT);
	pinMode(YELLOW_LEFT, OUTPUT);

  pinMode(GREEN_RIGHT, OUTPUT);
  pinMode(RED_RIGHT, OUTPUT);
  pinMode(YELLOW_RIGHT, OUTPUT);

	Serial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // Set the update rate: 1 Hz
  useInterrupt(true);
  delay(1000);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
}

uint32_t timer = millis();

void loop() {

  if (GPS.newNMEAreceived()) {

    char *lastNMEA = GPS.lastNMEA();

    int len = strlen( lastNMEA );
    for (int i = 0; i < len; i++){
      gps.encode(lastNMEA[i]);
    }

    if (!GPS.parse(lastNMEA))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  if (millis() - timer > READ_FREQ_MS) {
		if (GPS.fix) {
			printGpsInfo();
		} else {
		  Serial.print("No Fix");
		}

    ultrasonic_right.DistanceMeasure(); // get the current signal time;
    ultrasonic_left.DistanceMeasure(); // get the current signal time;

    printRangeInfo(ultrasonic_left.microsecondsToCentimeters(), ultrasonic_right.microsecondsToCentimeters());
    Serial.println();
	}
}

void updateSpeedIndicators(int pin1, int pin2) {
  analogWrite(GREEN_LEFT, 0);
  analogWrite(YELLOW_LEFT, 0);
  analogWrite(RED_LEFT, 0);
  analogWrite(GREEN_RIGHT, 0);
  analogWrite(YELLOW_RIGHT, 0);
  analogWrite(RED_RIGHT, 0);
  analogWrite(pin1, 50);
  analogWrite(pin2, 50);
}

void printRangeInfo(long rangeCmL, long rangeCmR) {
	Serial.print(", ");
	Serial.print(rangeCmL);
	Serial.print("cm, ");
	if (rangeCmL < STOP_DIST_CM && rangeCmR < STOP_DIST_CM) {
    updateSpeedIndicators(RED_LEFT, RED_RIGHT);
    Serial.print("Reverse!");
  } else if (rangeCmR < STOP_DIST_CM && rangeCmL > STOP_DIST_CM) {
    updateSpeedIndicators(GREEN_LEFT, RED_RIGHT);
    Serial.print("Hard left");
  } else if (rangeCmL < STOP_DIST_CM && rangeCmR > STOP_DIST_CM) {
		updateSpeedIndicators(RED_LEFT, GREEN_RIGHT);
		Serial.print("Hard right");
  } else if (rangeCmL < SLOW_DIST_CM && rangeCmR < SLOW_DIST_CM) {
    updateSpeedIndicators(YELLOW_LEFT, YELLOW_RIGHT);
    Serial.print("Slow down");
  } else if (rangeCmR < SLOW_DIST_CM && rangeCmL > SLOW_DIST_CM) {
    updateSpeedIndicators(GREEN_LEFT, YELLOW_RIGHT);
    Serial.print("Soft left");
  } else if (rangeCmL < SLOW_DIST_CM && rangeCmR > SLOW_DIST_CM) {
    updateSpeedIndicators(YELLOW_LEFT, GREEN_RIGHT);
    Serial.print("Soft right");
	} else {
		updateSpeedIndicators(GREEN_LEFT, GREEN_RIGHT);
		Serial.print("BALLS TO THE WALL!");
	}
}

void printGpsInfo() {
	Serial.print(GPS.latitude, 4);
	Serial.print(GPS.lat);
	Serial.print(", ");
	Serial.print(GPS.longitude, 4);
	Serial.print(GPS.lon);
	Serial.print(", ");
	Serial.print(GPS.speed);
	Serial.print(" knots,  ");
	Serial.print(GPS.angle);
	Serial.print(" degrees");
}
