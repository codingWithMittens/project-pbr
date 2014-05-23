#include "Arduino.h"
#include "Ultrasonic.h"
#include "Adafruit_GPS.h"
#include "SoftwareSerial.h"
#include "TinyGPS++.h"
#include <Servo.h>

// pins:
//
int const RANGE_LEFT   = 7,  RANGE_RIGHT   = 8;
int const MOTOR_FORWARD = 4, MOTOR_REVERSE = 6, DUTY_CYCLE  = 13;
int const SERVO = 9;
int const COMPASS_SCL = 21, COMPASS_SDA = 20;
int const REV_COUNT = 3;
// Not configurable:
// GPS_RX = 18, GPS_TX = 19

// configuration:
double const STOP_DIST_CM = 75.0, SLOW_DIST_CM = 125.0, REVERSE_DIST_CM = 125.0;
int const READ_FREQ_MS = 500;
int const WAY_POINT_RADIUS_CM = 100;
int const FORWARD = 1, STOP = 0, REVERSE = -1;
int const STRAIGHT = 90, SOFT_TURN = 10, HARD_TURN = 20;
double const MAX_SPEED = 20.0;

int const WAY_POINT_RADIUS_M = 10;

float wayPoints[7][2] = {
  {39.924052, -105.153167},
  {39.924233, -105.153139},
  {39.924391, -105.153170},
  {39.924553, -105.153291},
  {39.924552, -105.153080}
};

int curWayPoint = 0;
double nextLat = wayPoints[curWayPoint][0];
double nextLon = wayPoints[curWayPoint][1];

long obsCmLeft;
long obsCmRight;
int curSpeed = 0;
int angle = STRAIGHT;
int dir = 1;

double lastLat = 39.924053;
double lastLon = -105.153152;
double curLat = 0.0;
double curLng = 0.0;

double DIFF_THRESH = 1; //0.0001;
double gpsdiffLat = 0.0;
double gpsdiffLng = 0.0;

Adafruit_GPS GPS(&Serial1);
// #define GPSECHO  false // debug?

TinyGPSPlus gps;
Servo myservo;

Ultrasonic ultrasonic_left(RANGE_LEFT);
Ultrasonic ultrasonic_right(RANGE_RIGHT);

void setup() {
  setPinModes();
  Serial.begin(115200);
  initGPS();
}

void initGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand("$PMTK301,2*2E");
  GPS.sendCommand("$PMTK313,1*2E");
  useInterrupt();
}

void setPinModes() {
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
  updateGps();
  printGpsInfo();
  checkAchievementStatus();
  measureObsDistance();
  distToWayPointM();
  double correctCourse = courseDiff(gps.course.deg(), courseToWayPoint());
  Serial.print(", correctCourse: ");
  Serial.print(correctCourse);

  drive(1, 10, correctCourse);
  // drive(1, curSpeed, angle);

  delay(100);
  Serial.println();
}

void measureObsDistance() {
  ultrasonic_right.DistanceMeasure(); // get the current signal time;
  ultrasonic_left.DistanceMeasure(); // get the current signal time;

  obsCmLeft = ultrasonic_left.microsecondsToCentimeters();
  obsCmRight = ultrasonic_right.microsecondsToCentimeters();
  curSpeed = calculateSpeed(obsCmLeft, obsCmRight);
  angle = calculateDirection(obsCmLeft, obsCmRight);
}

void updateGps() {
  if (gps.location.isUpdated()){       //Looks for new GPS to start loop
    curLat = (gps.location.lat());    //assigns newly updated latitude to "curLat" variable
    curLng = (gps.location.lng());   // assigns newly updated longitude to "newLngg" variable

    gpsdiffLat = (lastLat - curLat);   // calculate distance between last accepted coordinates and new coordinates
    gpsdiffLng = (lastLon - curLng);

    //Check to see if new coordinates are close enough to the last
    if ( (gpsdiffLat < DIFF_THRESH) && (gpsdiffLng < DIFF_THRESH) &&
        (gpsdiffLat > -1 * DIFF_THRESH) && (gpsdiffLng > -1 * DIFF_THRESH)) {
      lastLat = curLat;  //designates new coordinates as accepted.
      lastLon = curLng;
    }
  }
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

double courseDiff(double curAngle, double destAngle) {
  double diff = destAngle - curAngle;
  double absDiff = abs(diff);

  if (destAngle > curAngle) {
    return absDiff - 360;
  } else {
    return 360 - absDiff;
  }
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

boolean checkAchievementStatus() {
  if(distToWayPointM() < WAY_POINT_RADIUS_M) {
    curWayPoint += 1;
    nextLat = wayPoints[curWayPoint][0];
    nextLon = wayPoints[curWayPoint][1];
    return true;
  } else {
    return false;
  }
}

long distToWayPointM() {
  return TinyGPSPlus::distanceBetween(lastLat, lastLon, nextLat, nextLon);
}

long courseToWayPoint() {
  return TinyGPSPlus::courseTo(lastLat, lastLon, nextLat, nextLon);
}

// logging //
void printGpsInfo() {
  Serial.print("Lat, Long): "); Serial.print(curLat , 6); Serial.print(", "); Serial.print(curLng, 6);
  Serial.print(", WP Lat, WP Long): "); Serial.print(nextLat, 6); Serial.print(", "); Serial.print(nextLon, 6);
  Serial.print(", Unlocked?:"); Serial.print(checkAchievementStatus());
  Serial.print(", WP#:"); Serial.print(curWayPoint);
  Serial.print(", Dist2WP:"); Serial.print(distToWayPointM());
  Serial.print(", Cur Angle:"); Serial.print(gps.course.deg());
  Serial.print(", Angle2WP: "); Serial.print(courseToWayPoint());
}

