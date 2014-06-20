// pbr_closedloopsteering.ino

//proportional closed loop steering control::


// pbr_everything.ino
#include "Arduino.h"
#include "Ultrasonic.h"
#include "Adafruit_GPS.h"
#include "SoftwareSerial.h"
#include "TinyGPS++.h"
#include <Servo.h>

///--compass stuff--///
#include <Wire.h>
#include "math.h"
////////////////////


// pins:
int const RANGE_LEFT = 7, RANGE_RIGHT = 8;
int const MOTOR_FORWARD = 4, MOTOR_REVERSE = 6, DUTY_CYCLE = 13;
int const SERVO = 9;
int const COMPASS_SCL = 21, COMPASS_SDA = 20;
int const REV_COUNT = 3;
// Not configurable:
// GPS_RX = 18, GPS_TX = 19

// configuration:
double const STOP_DIST_CM = 75.0, SLOW_DIST_CM = 125.0, REVERSE_DIST_CM = 125.0;
int const READ_FREQ_MS = 500;
// int const WAY_POINT_RADIUS_CM = 1;
int const FORWARD = 1, STOP = 0, REVERSE = -1;
int const STRAIGHT = 130, SOFT_TURN = 10, HARD_TURN = 10;
int const MIN_STEER_DIFF = 0;
double const MAX_SPEED = 30.0;
double const SLOW_SPEED = 25.0;
double GPS_DIFF_THRESH = 5.0;  //now in meters

// dist and angle calculations
float const LAT_TO_METERS = 111034.0, LNG_TO_METERS = 85393.0;
float const Y_SCALE = 0.0039, X_SCALE = 0.0045;
int const Y_OFF = 1379, X_OFF = -1689;
int const COMPASS_CORRECTION = 0;
double const METERS_PER_REV = 0.036;
double xCal, yCal;
double xCali, yCali; //intermediate calibrated values
double const TH_OFF = 4*(3.14/180); // physical offset of magnetometer [degrees]

int const WAY_POINT_RADIUS_M = 4; //
int const WAY_PT_Q = 5; //number of waypoints


float const WAY_POINTS[5][2] = {
{39.982355, -105.240465},
{39.982332, -105.240297},
{39.982431, -105.240281},
{39.982449, -105.240446},
{39.982355, -105.240465}
}; //summit parking lot



int curWayPoint = 0;

// float lastLat = WAY_POINTS[curWayPoint][0];
// float lastLon = WAY_POINTS[curWayPoint][1];
float lastLat = 0.0;
float lastLon = 0.0;
float nextLat = lastLat;
float nextLon = lastLon;
float lastLatRev;
float lastLonRev;
float const REV_WEIGHT = 0.0;

long obsCmLeft;
long obsCmRight;
int curSpeed = 0;
int angle = STRAIGHT;
int dir = 1;

double gpsdiffLat = 0.0;
double gpsdiffLng = 0.0;
///////////////////--compass stuff--////
#define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change

float NextVectorX;
float NextVectorY;

float distToNext;
float MagComp = 1.0;
float CompDotNext;
float AngleNext;
float DirectionNext;

double curLat = 0.0;
double curLng = 0.0;

int Steering;
int LeftLimit = 70;
int RightLimit = 110;
float angleFromStraight = 0.0;
int curAngle = STRAIGHT;
int driveAngle;


volatile int revCount = 0;

///////////////////////////

Adafruit_GPS GPS(&Serial1);
// #define GPSECHO false // debug?

TinyGPSPlus gps;
Servo myservo;

Ultrasonic ultrasonic_left(RANGE_LEFT);
Ultrasonic ultrasonic_right(RANGE_RIGHT);

void setup() {
  setPinModes();
  Serial.begin(115200);
  initGPS();
  initComp();
  attachRevInt();
  updateRevLocation();
  }
//--compass stuff--///

void initComp(){
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  initCompass();            // turn the MAG3110 on
}

////////////////Rev counter stuff////////////

void attachRevInt() {
  attachInterrupt(0, countRevs, FALLING); //enable interrupt
}

void updateRevLocation() {
  double distTraveledM = revCount * METERS_PER_REV;

  lastLat += (yCal * distTraveledM * (1 / MagComp));
  lastLon += (-1* xCal * distTraveledM * (1 / MagComp));
  lastLatRev = lastLat;
  lastLonRev = lastLon;

  revCount = 0;
}

void countRevs() { // this code will be executed every time the interrupt 0 (pin2) gets low.
  revCount++;
}

//////////////////////

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
  checkAchievementStatus();
  updateRevLocation();

  driveAngle = steerLoop();


  int steerAroundObs = overrideDir();

  double safeSpeed = MAX_SPEED;
  if (steerAroundObs > -1) {
    curAngle = steerAroundObs;
    safeSpeed = SLOW_SPEED;
   }

  drive(FORWARD, safeSpeed, driveAngle);

  logOutput();
  //delay(50);
  //Serial.println();
}

int steerLoop() {

    float const GAIN_STEER = 1; // gain to add to steering angle
    float angleToWPnow = angleToWP() ;// call angle to way point so it doesnt call it every time in calc.

    if ((curAngle > (STRAIGHT - 10)) && (curAngle < (STRAIGHT + 10))) {
    curAngle = curAngle  + GAIN_STEER * angleToWPnow;
    } else  {
    curAngle = STRAIGHT + angleToWPnow;
    }
    return curAngle;
}

int angleToWP() {
  yCali = (readY() - Y_OFF) * Y_SCALE;
  xCali = (readX() - X_OFF) * X_SCALE;

  xCal = xCali * cos(TH_OFF) - yCali * sin(TH_OFF);
  yCal = xCali * sin(TH_OFF) + yCali * cos(TH_OFF);

  //delay(20);

  NextVectorX =  (nextLon - lastLon);
  NextVectorY =  (nextLat - lastLat);

  //distance in meters from current position to next way point
  distToNext = sqrt(square(NextVectorY) + square(NextVectorX));

  //Magnitude of compass vector for finding angle
  MagComp =  sqrt(square(xCal) + square(yCal));

  // Dot product of next vector and current direction(negative 1 multiply is
  // because compass reading is “how many degrees  NORTH is FROM CAR pointing
  // direction, we want how many degree car is FROM NORTH we are pointing)..
  CompDotNext = (-1 * xCal * NextVectorX) + (yCal * NextVectorY);

  // Cos^-1((A*B)/||A|| ||B||)--- Magnitude of angle(in radians to next
  // way point from current direction car is pointed.
  AngleNext = acos(CompDotNext/(MagComp*distToNext)) * 180 / 3.14;

  //if negative turn left, if positive turn right //if negative turn left, if positive turn right
  DirectionNext = ((yCal * NextVectorX) - (-1 * xCal * NextVectorY)) + COMPASS_CORRECTION;
  DirectionNext = DirectionNext / fabs(DirectionNext) * -1;

  return (DirectionNext * AngleNext); //deleted "straight +"

}

void updateGps() {
 if (gps.location.isUpdated()){ //Looks for new GPS to start loop
   curLat = ((gps.location.lat())- WAY_POINTS[0][0]) * LAT_TO_METERS; //assigns newly updated latitude to "curLat" variable
   curLng = ((gps.location.lng())- WAY_POINTS[0][1]) * LNG_TO_METERS; // assigns newly updated longitude to "curLng" variable

   gpsdiffLat = fabs(lastLat - curLat); // calculate distance between last accepted coordinates and new coordinates
   gpsdiffLng = fabs(lastLon - curLng);

   //Check to see if new coordinates are close enough to the last
   if ( (gpsdiffLat < GPS_DIFF_THRESH) && (gpsdiffLng < GPS_DIFF_THRESH))// &&
       // (gpsdiffLat > -1 * GPS_DIFF_THRESH) && (gpsdiffLng > -1 * GPS_DIFF_THRESH))
{
    detachInterrupt(0);

     lastLat = (1-REV_WEIGHT * curLat) + (REV_WEIGHT * lastLatRev); //designates new coordinates as accepted.
     lastLon = (1-REV_WEIGHT * curLng) + (REV_WEIGHT * lastLonRev);

   attachInterrupt(0, countRevs, FALLING); //enable interrupt
   }
 }
}

////OUTPUT TO CAR/////
void drive(int dir, int spd, int degree) {
  float duty = (spd / 100.00) * 255.00;
  boolean forward, reverse;

  if(curWayPoint == WAY_PT_Q) {
    duty = 0;
  }

  if(dir == 1) {
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

  steer(degree);
  digitalWrite(MOTOR_FORWARD, forward);
  digitalWrite(MOTOR_REVERSE, reverse);
  analogWrite(DUTY_CYCLE, duty);
}

void steer(int degree) {
  // degree = degree / 2;
  if (abs(degree - STRAIGHT) > MIN_STEER_DIFF){
    if (degree > (STRAIGHT + HARD_TURN)) {
      degree = STRAIGHT + HARD_TURN;
    } else if (degree < (STRAIGHT - HARD_TURN)) {
      degree = STRAIGHT - HARD_TURN;
    }
    myservo.write(degree);
  }
}

//////////----RANGE FINDER/OBSTACLE STUFF----////////////
int overrideDir() {
 ultrasonic_right.DistanceMeasure(); // get the current signal time;
 ultrasonic_left.DistanceMeasure(); // get the current signal time;

 obsCmLeft = ultrasonic_left.microsecondsToCentimeters();
 obsCmRight = ultrasonic_right.microsecondsToCentimeters();
 return calculateDirection(obsCmLeft, obsCmRight);
}

// double calculateSpeed(long rangeCmL, long rangeCmR) {
//  if (rangeCmL > rangeCmR) {
//    return speedFromDist(rangeCmR);
//  } else {
//    return speedFromDist(rangeCmL);
//  }
// }

// double speedFromDist(long nextObstacleCm) {
//  if (nextObstacleCm > SLOW_DIST_CM) {
//    return MAX_SPEED;
//  } else {
//    // Serial.println(MAX_SPEED * (nextObstacleCm / SLOW_DIST_CM));
//    return MAX_SPEED * (nextObstacleCm / SLOW_DIST_CM);
//  }
// }

int calculateDirection(long rangeCmL, long rangeCmR) {
 long nextObstacleCm = (rangeCmL > rangeCmR) ? rangeCmR : rangeCmL;
 long nextObjDir = rangeCmL > rangeCmR ? -1 : 1;
 if (nextObstacleCm < SLOW_DIST_CM) {
   return STRAIGHT + ((HARD_TURN * ((SLOW_DIST_CM - nextObstacleCm) / SLOW_DIST_CM)) * nextObjDir);
 } else {
   return -1;
 }
}
///////////////////////////////////////////////////////////////

////////--- REACHED WAY POINT?---////////////
boolean checkAchievementStatus() {
  if(distToNext < WAY_POINT_RADIUS_M) {
    curWayPoint += 1;
    nextLat = (WAY_POINTS[curWayPoint][0] - WAY_POINTS[0][0]) * LAT_TO_METERS;
    nextLon = (WAY_POINTS[curWayPoint][1] - WAY_POINTS[0][1]) * LNG_TO_METERS;

    return true;
  } else {
    return false;
  }
}




///////////////////////////////////////////////////////////////

/////---TINY GPS DISTANCE/COURSE---/////
//long distToWayPointM() {
//  return TinyGPSPlus::distanceBetween(lastLat, lastLon, nextLat, nextLon);
//}
//
//long courseToWayPoint() {
//  return TinyGPSPlus::courseTo(lastLat, lastLon, nextLat, nextLon);
//}

//////////////////////////////////////////---compass stuff---//////////////////////////////////////////

void initCompass(void)
{
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x11);                 // cntrl register2
  Wire.write(0x80);                 // send 0x80, enable auto resets
  Wire.endTransmission();           // stop transmitting

  delay(15);

  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x10);                 // cntrl register1
  Wire.write(1);                    // send 0x01, active mode
  Wire.endTransmission();           // stop transmitting
}

void logOutput(void)
{
    int lastmilisprnt;
    if ((millis()- lastmilisprnt) > 50) {
         lastmilisprnt = millis();
  // Serial.print("x=");
  // Serial.print(readX());
  // Serial.print(",");
  // Serial.print("y=");
  // Serial.print(readY());
  // Serial.print(", ycal = ");
  // Serial.print(Ycal);
  // Serial.print(", xcal = ");
  // Serial.print(Xcal);
  // Serial.print(", angle");
  Serial.print(", angle to next : ");
  Serial.print(AngleNext);
  // Serial.print(", Steering angle : ");
  // Serial.print(Steering);
   // Serial.print(", angleFromStraight : "); //obs
   // Serial.print(angleFromStraight);
  Serial.print(",  direction l or r : ");
  Serial.print(DirectionNext);
  // Serial.print(", NextVectorY = ");
  // Serial.print(NextVectorY, 6);
  // Serial.print(", NextVectorX = ");
  // Serial.print(NextVectorX, 6);

  Serial.print(", lastlon = ");
  Serial.print(lastLon);
  Serial.print(", lastLat = ");
  Serial.print(lastLat);
  // Serial.print(", nextlon = ");
  // Serial.print(nextLon);
  // Serial.print(", nextLat = ");
  // Serial.print(nextLat);
  Serial.print(", curWayPoint = ");
  Serial.print(curWayPoint);
  //Serial.print("Lat, Long): "); Serial.print(curLat , 6); Serial.print(", "); Serial.print(curLng, 6);
  // Serial.print(", WP Lat, WP Long): "); Serial.print(nextLat, 6); Serial.print(", "); Serial.print(nextLon, 6);
  // Serial.print(", Unlocked?:"); Serial.print(checkAchievementStatus());
  // Serial.print(", WP#:"); Serial.print(curWayPoint);
  Serial.print(" , Dist2WP:"); Serial.print(distToNext);
   Serial.print(", Cur Angle:"); Serial.print(driveAngle);
  // Serial.print(", Angle2WP: "); Serial.print(courseToWayPoint());
  Serial.println();
}
}




int readX(void) {
  int xl, xh;  //define the MSB and LSB

  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x01);              // x MSB reg
  Wire.endTransmission();       // stop transmitting
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop

  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available()) {   // slave may send less than requested
    xh = Wire.read(); // receive the byte
  }
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop

  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x02);              // x LSB reg
  Wire.endTransmission();       // stop transmitting
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop

  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available()) {   // slave may send less than requested
    xl = Wire.read(); // receive the byte
  }

  int xout = (xl|(xh << 8)); //concatenate the MSB and LSB
  return xout;
}

int readY(void)
{
  int yl, yh;  //define the MSB and LSB

  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x03);              // y MSB reg
  Wire.endTransmission();       // stop transmitting

  delayMicroseconds(2); //needs at least 1.3us free time between start and stop

  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available()) {   // slave may send less than requested
    yh = Wire.read(); // receive the byte
  }

  delayMicroseconds(2); //needs at least 1.3us free time between start and stop

  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x04);              // y LSB reg
  Wire.endTransmission();       // stop transmitting

  delayMicroseconds(2); //needs at least 1.3us free time between start and stop

  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available()) {  // slave may send less than requested
    yl = Wire.read(); // receive the byte
  }

  int yout = (yl|(yh << 8)); //concatenate the MSB and LSB
  return yout;
}