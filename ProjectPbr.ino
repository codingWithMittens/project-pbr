#include "Arduino.h"
#include "Adafruit_GPS.h"
#include "SoftwareSerial.h"
#include "TinyGPS++.h"
#include <Servo.h>

// pins:
int const GREEN_LEFT   = 2,  RED_LEFT      = 2,  YELLOW_LEFT  = 2;
int const GREEN_RIGHT  = 2,  RED_RIGHT     = 2,  YELLOW_RIGHT = 2;
int const RANGE_LEFT   = 7,  RANGE_RIGHT   = 8;
int const MOTOR_FORWARD = 4, MOTOR_REVERSE = 6, DUTY_CYCLE  = 13;
int const SERVO = 9;
// Not configurable:
// GPS_RX = 18, GPS_TX = 19

// configuration:
double const STOP_DIST_CM = 75.0, SLOW_DIST_CM = 125.0, REVERSE_DIST_CM = 125.0;
int const READ_FREQ_MS = 500;
int const WAY_POINT_RADIUS_CM = 100;
int const FORWARD = 1, STOP = 0, REVERSE = -1;
int const STRAIGHT = 90, SOFT_TURN = 10, HARD_TURN = 20;
double const MAX_SPEED = 20.0;

boolean MONITOR_OUTPUT_OVER_SERIAL = true;

float wayPoints[7][2] = {
 {-105.154025, 39.924652},
 {-105.154013, 39.924672},
 {-105.153957, 39.924659},
 {-105.153916, 39.924646},
 {-105.153897, 39.924604},
 {-105.153957, 39.924618},
 {-105.154013, 39.924629}
};

int curWayPoint = 0;
int curSpeed = 0;
int dir = 1;

Adafruit_GPS GPS(&Serial1);
// #define GPSECHO  false // debug?

TinyGPSPlus gps;
Servo myservo;


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
  digitalWrite(_pin, LOW);
  pinMode(_pin, INPUT);
  duration = pulseIn(_pin, HIGH);
}

// The measured distance from the range 0 to 400 Centimeters
long Ultrasonic::microsecondsToCentimeters(void) {
  return duration/29/2;
}

Ultrasonic ultrasonic_left(RANGE_LEFT);
Ultrasonic ultrasonic_right(RANGE_RIGHT);

void setup() {
  setPinModes();

  if (MONITOR_OUTPUT_OVER_SERIAL) {
    Serial.begin(115200);
  }

  initGPS();
}

void initGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand("$PMTK301,1*2D");
  useInterrupt();
}

void setPinModes() {
  pinMode(GREEN_LEFT, OUTPUT);
  pinMode(RED_LEFT, OUTPUT);
  pinMode(YELLOW_LEFT, OUTPUT);

  pinMode(GREEN_RIGHT, OUTPUT);
  pinMode(RED_RIGHT, OUTPUT);
  pinMode(YELLOW_RIGHT, OUTPUT);

  pinMode(MOTOR_FORWARD, OUTPUT);
  pinMode(MOTOR_REVERSE, OUTPUT);
  pinMode(DUTY_CYCLE, OUTPUT);

  myservo.attach(SERVO);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and feeds it to the encoder
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  gps.encode(c);
}

void useInterrupt() {
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

uint32_t timer = millis();

void loop() {
  ultrasonic_right.DistanceMeasure(); // get the current signal time;
  long distLeft = ultrasonic_left.microsecondsToCentimeters();

  ultrasonic_left.DistanceMeasure(); // get the current signal time;
  long distRight = ultrasonic_right.microsecondsToCentimeters();

  int curSpeed = calculateSpeed(distLeft, distRight);
  // Serial.print(curSpeed);
  // if (gps.location.isUpdated()) {
    // Serial.println(GPS.lastNMEA());
    // printGpsInfo();
    // dir = printRangeInfo(distLeft, distRight, dir);
    int angle = calculateDirection(distLeft, distRight);
    drive(1, curSpeed, angle);
    Serial.print(angle);
    Serial.println();
  // }
  delay(100);
}

void updateSpeedIndicators(int pin1, int pin2) {
  // lightsOut();
  // analogWrite(pin1, 50);
  // analogWrite(pin2, 50);
}

void drive(int dir, int spd, int degr) {
  // Serial.print("speed: ");
  // Serial.println(spd);
  float duty = (spd / 100.00) * 255.00;
  boolean forward, reverse;

  if(dir == 1)  {
    forward = HIGH;
    reverse = LOW;
  } else if(dir == -1) {
    forward = LOW;
    reverse = HIGH;
    duty = 45;
  } else if(dir = 0) {
    forward = LOW;
    reverse = LOW;
  }

  steer(degr);
  digitalWrite(MOTOR_FORWARD, forward);
  digitalWrite(MOTOR_REVERSE, reverse);
  analogWrite(DUTY_CYCLE, duty);
}

void steer(int pos_deg) {
  myservo.write(pos_deg);
}

void lightsOut() {
  analogWrite(GREEN_LEFT, 0);
  analogWrite(YELLOW_LEFT, 0);
  analogWrite(RED_LEFT, 0);
  analogWrite(GREEN_RIGHT, 0);
  analogWrite(YELLOW_RIGHT, 0);
  analogWrite(RED_RIGHT, 0);
}

double calculateSpeed(long rangeCmL, long rangeCmR) {
  if (rangeCmL > rangeCmR) {
    return speedFromDist(rangeCmR);
  } else {
    return speedFromDist(rangeCmL);
  }
}

double speedFromDist(long nextObstacleCm) {
  if (nextObstacleCm > SLOW_DIST_CM) {
    return MAX_SPEED;
  } else {
    // Serial.println(MAX_SPEED * (nextObstacleCm / SLOW_DIST_CM));
    return MAX_SPEED * (nextObstacleCm / SLOW_DIST_CM);
  }
}

int calculateDirection(long rangeCmL, long rangeCmR) {
  long nextObstacleCm = (rangeCmL > rangeCmR) ? rangeCmR : rangeCmL;
  // long rangeDiff = fabs(rangeCmL - rangeCmR);
  long nextObjDir = rangeCmL > rangeCmR ? -1 : 1;
  if (nextObstacleCm < SLOW_DIST_CM) {
    return STRAIGHT + ((HARD_TURN * ((SLOW_DIST_CM - nextObstacleCm) / SLOW_DIST_CM)) * nextObjDir);
  } else {
    return STRAIGHT;
  }
}

// int printRangeInfo(long rangeCmL, long rangeCmR, int curMode) {
//   if (curMode == -1 && (rangeCmL < REVERSE_DIST_CM) && (rangeCmR < REVERSE_DIST_CM)) {
//     steer(HARD_LEFT);
//     return -1;
//   } else if (rangeCmL < STOP_DIST_CM && rangeCmR < STOP_DIST_CM) {
//     steer(HARD_LEFT);
//     return -1;
//   } else if (rangeCmR < STOP_DIST_CM && rangeCmL > STOP_DIST_CM) {
//     steer(HARD_LEFT);
//   } else if (rangeCmL < STOP_DIST_CM && rangeCmR > STOP_DIST_CM) {
//     steer(HARD_RIGHT);
//   } else if (rangeCmL < SLOW_DIST_CM && rangeCmR < SLOW_DIST_CM) {
//     steer(STRAIGHT);
//   } else if (rangeCmR < SLOW_DIST_CM && rangeCmL > SLOW_DIST_CM) {
//     steer(HARD_LEFT);
//   } else if (rangeCmL < SLOW_DIST_CM && rangeCmR > SLOW_DIST_CM) {
//     steer(HARD_RIGHT);
//   } else {
//     steer(STRAIGHT);
//   }
//   return 1;
// }

// void printGpsInfo() {
//   // static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
//   // double distanceToLondon = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), LONDON_LAT, LONDON_LON);
//   // double courseToLondon = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), LONDON_LAT, LONDON_LON);
//   Serial.print("Checksum:");
//   if (gps.failedChecksum()) {
//     Serial.print("ok");
//   } else {
//     Serial.print("fail");
//   }
//   Serial.print("; ");

//   Serial.print("WP#");
//   Serial.print(curWayPoint);
//   Serial.print(": ");
//   Serial.print(wayPoints[curWayPoint][0], 6);
//   Serial.print(", ");
//   Serial.print(wayPoints[curWayPoint][1], 6);
//   Serial.print("; ");

//   Serial.print("Dist (m):");
//   Serial.print(distToWayPoint());
//   Serial.print("; ");

//   Serial.print("Course (deg):");
//   Serial.print(courseToWayPoint());
//   Serial.print("; ");

//   Serial.print("Heading (deg): ");
//   Serial.print(gps.course.deg());
//   Serial.print("; ");

//   Serial.print("Position: ");
//   Serial.print(gps.location.lat(), 6);
//   Serial.print(", ");
//   Serial.print(gps.location.lng(), 6);
//   Serial.print("; ");

//   Serial.print("Speed (kph): ");
//   Serial.print(gps.speed.kmph());
//   Serial.print("; ");
// }

boolean achievementUnlocked() {
  if(distToWayPoint() < WAY_POINT_RADIUS_CM) {
    curWayPoint += 1;
    return true;
  } else {
    return false;
  }
}

long distToWayPoint() {
  return TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    wayPoints[curWayPoint][1],
    wayPoints[curWayPoint][0]);
}

long courseToWayPoint() {
  return TinyGPSPlus::courseTo(
    gps.location.lat(),
    gps.location.lng(),
    wayPoints[curWayPoint][1],
    wayPoints[curWayPoint][0]);
}