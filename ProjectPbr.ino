#include <Arduino.h>
// #include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// pins:
int const GREEN_LEFT   = 2,  RED_LEFT     = 3,  YELLOW_LEFT = 4;
int const GREEN_RIGHT  = 8,  RED_RIGHT    = 9,  YELLOW_RIGHT = 10;
int const RANGE_LEFT   = 7,  RANGE_RIGHT  = 6;
int const RX = 18, TX = 19;

// configuration:
int const STOP_DIST_CM = 25, SLOW_DIST_CM = 50, READ_FREQ_MS = 300;

// Adafruit_GPS GPS(&Serial1);
// #define GPSECHO  false // debug?

TinyGPSPlus gps;
SoftwareSerial ss(RX, TX);

class Ultrasonic {
	public:
    Ultrasonic(int pin);
    void DistanceMeasure(void);
    long microsecondsToCentimeters(void);
  private:
    int _pin;
    long duration;
};

Ultrasonic::Ultrasonic(int pin) {
	_pin = pin;
}

// Begin the detection and get the pulse back signal
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

// The measured distance from the range 0 to 400 Centimeters
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

  Serial.begin(115200);
  ss.begin(9600);
  // ss.write(PMTK_SET_NMEA_UPDATE_1HZ); // Set the update rate: 1 Hz
  useInterrupt();
  delay(1000);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and feeds it to the encoder
SIGNAL(TIMER0_COMPA_vect) {
  char c = ss.read();
  gps.encode(c);
}

void useInterrupt() {
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

// uint32_t timer = millis();

void loop() {
	if (gps.location.isUpdated()) {
    Serial.print("Passed Checksum:");
    Serial.print(!gps.failedChecksum());
    Serial.print("; ");
    printGpsInfo();

    // get the current signal time;
    ultrasonic_right.DistanceMeasure();
    ultrasonic_left.DistanceMeasure();

    printRangeInfo(ultrasonic_left.microsecondsToCentimeters(), ultrasonic_right.microsecondsToCentimeters());
    Serial.println();
  }
}

void updateSpeedIndicators(int pin1, int pin2) {
  lightsOut();
  analogWrite(pin1, 50);
  analogWrite(pin2, 50);
}

void lightsOut() {
  analogWrite(GREEN_LEFT, 0);
  analogWrite(YELLOW_LEFT, 0);
  analogWrite(RED_LEFT, 0);
  analogWrite(GREEN_RIGHT, 0);
  analogWrite(YELLOW_RIGHT, 0);
  analogWrite(RED_RIGHT, 0);
}

void printRangeInfo(long rangeCmL, long rangeCmR) {
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
  Serial.print("; ");
  Serial.print("Position: ");
  Serial.print(gps.location.lat());
  // Serial.println(gps.location.rawLatDegrees()); // Raw latitude in whole degrees (no work!)
  // Serial.println(gps.location.rawLatBillionths()); // ... and billionths (i16/u32) (no work!)
  Serial.print(", ");
  Serial.print(gps.location.lng());
  // Serial.println(gps.location.rawLngDegrees()); // Raw longitude in whole degrees (no work!)
  // Serial.println(gps.location.rawLngBillionths()); // ... and billionths (i16/u32) (no work!)
  Serial.print("(within ");
    Serial.print(gps.hdop.value());
    Serial.print("); ");
  // Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32) (keeping for last read calcs)
  Serial.print("Speed (kph): ");
  Serial.print(gps.speed.kmph());
  Serial.print("; ");
  Serial.print("Heading (deg): ");
  Serial.print(gps.course.deg());
  Serial.print("; ");
  Serial.print("Altitude: ");
  Serial.print(gps.altitude.kilometers());
  Serial.print("; ");
}
