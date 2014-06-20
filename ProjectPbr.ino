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
int const RANGE_LEFT = 8, RANGE_RIGHT = 7;
int const MOTOR_FORWARD = 4, MOTOR_REVERSE = 6, DUTY_CYCLE = 13;
int const SERVO = 9;
int const COMPASS_SCL = 21, COMPASS_SDA = 20;
int const REV_COUNT = 3;
// Not configurable:
// GPS_RX = 18, GPS_TX = 19

// configuration:
double const STOP_DIST_CM = 75.0, SLOW_DIST_CM = 250.0, REVERSE_DIST_CM = 25.0;
int const READ_FREQ_MS = 500;
// int const WAY_POINT_RADIUS_CM = 1;
int const FORWARD = 1, STOP = 0, REVERSE = -1;
int const STRAIGHT = 130, SOFT_TURN = 10, HARD_TURN = 10;
int const MIN_STEER_DIFF = 1;
double const MAX_SPEED = 35.0, SLOW_SPEED = 30.0;
double GPS_DIFF_THRESH = 0.0005;

// dist and angle calculations
float const LAT_TO_METERS = 111034.0, LNG_TO_METERS = 85393.0;
float const Y_SCALE = 0.00926, X_SCALE = 0.0118;
int const Y_OFF = 1456, X_OFF = -1621;
int const COMPASS_CORRECTION = 0;

int const WAY_POINT_RADIUS_M = 10; //
// float const WAY_POINTS[6][2] = {
//   // {39.92118, -105.160636},
//   {39.920994, -105.160387},
//   {39.920938, -105.160239},
//   {39.921091, -105.160126},
//   {39.92118, -105.160636}
// };
int const wayPointQuantity = 7;

// lat, lon, speed_offset
// float const WAY_POINTS[wayPointQuantity][3] = {
//   {39.921199, -105.160718, -5},
//   {39.920999, -105.160460, 10},
//   {39.920938, -105.160265, -10},
//   {39.921103, -105.160152, 5},
//   // {39.921136, -105.160441},
//   {39.921199, -105.160718, -10}
// };

float const WAY_POINTS[wayPointQuantity][3] = {
  {39.92128, -105.15998, 0},
  {39.92142, -105.16074, 0},
  {39.92128, -105.16070, 0},
  {39.92112, -105.16054, 0},
  {39.92098, -105.16036, 0},
  {39.92099, -105.15990, 0},
  {39.92136, -105.15975, 0}
};


// {39.921011, -105.1599},
  // {39.921166, -105.160492},

 // {39.921279, -105.160646},
 //  {39.92125, -105.160759},
 //  {39.920981, -105.160695},
 //  {39.920829, -105.160557},
 //  {39.920731, -105.16035},
 //  {39.920764, -105.160198},
 //  {39.92094, -105.160225},
 //  {39.921008, -105.160498},
 //  {39.92118, -105.160636}

int curWayPoint = 0;

float lastLat = WAY_POINTS[curWayPoint][0];
float lastLon = WAY_POINTS[curWayPoint][1];
float nextLat = WAY_POINTS[curWayPoint][0];
float nextLon = WAY_POINTS[curWayPoint][1];
float curSpeedOffset = WAY_POINTS[curWayPoint][2];

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
float MagComp;
float CompDotNext;
float AngleNext;
float DirectionNext;

double curLat = 0.0;
double curLng = 0.0;

int Steering;
int LeftLimit = 70;
int RightLimit = 110;
float angleFromStraight = 0.0;

///////////////////////////

Adafruit_GPS GPS(&Serial1);
// #define GPSECHO false // debug?

TinyGPSPlus gps;
Servo myservo;

Ultrasonic ultrasonic_left(RANGE_LEFT);
Ultrasonic ultrasonic_right(RANGE_RIGHT);

void setup() {
  setPinModes();
  Serial.begin(9600);
  initGPS();
  initComp();
  }
//--compass stuff--///

void initComp(){
  Wire.begin();        // join i2c bus (address optional for master)
  // Serial.begin(9600);  // start serial for output
  initCompass();            // turn the MAG3110 on
}
////////////////
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

  int curAngle = angleToWP();
  int steerAroundObs = overrideDir();

  double safeSpeed = MAX_SPEED + curSpeedOffset;
  if (steerAroundObs > -1) {
    safeSpeed = SLOW_SPEED;
    curAngle = steerAroundObs;
  }

  drive(FORWARD, safeSpeed, curAngle);
  logOutput();
  delay(50);
  Serial.println();
}


int angleToWP() {
  double yCal = (readY() - Y_OFF) * Y_SCALE;
  double xCal = (readX() - X_OFF) * X_SCALE;

  delay(20);

  NextVectorX = LNG_TO_METERS * (nextLon - lastLon);
  NextVectorY = LAT_TO_METERS * (nextLat - lastLat);

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

  return STRAIGHT + (DirectionNext * AngleNext);
}

void updateGps() {
 if (gps.location.isUpdated()){ //Looks for new GPS to start loop
   curLat = (gps.location.lat()); //assigns newly updated latitude to "curLat" variable
   curLng = (gps.location.lng()); // assigns newly updated longitude to "curLng" variable

   gpsdiffLat = (lastLat - curLat); // calculate distance between last accepted coordinates and new coordinates
   gpsdiffLng = (lastLon - curLng);

   //Check to see if new coordinates are close enough to the last
   if ( (gpsdiffLat < GPS_DIFF_THRESH) && (gpsdiffLng < GPS_DIFF_THRESH) &&
       (gpsdiffLat > -1 * GPS_DIFF_THRESH) && (gpsdiffLng > -1 * GPS_DIFF_THRESH)) {
     lastLat = curLat; //designates new coordinates as accepted.
     lastLon = curLng;
   }
 }
}

////OUTPUT TO CAR/////
void drive(int dir, int spd, int degree) {
  float duty = (spd / 100.00) * 255.00;
  boolean forward, reverse;

  if(curWayPoint == wayPointQuantity) {
    duty = 0;
    dir = 0;
  }

  if(dir == 1) {
    forward = HIGH;
    reverse = LOW;
  } else if(dir == -1) {
    forward = LOW;
    reverse = HIGH;
    duty = 45;
  } else if(dir == 0) {
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
 Serial.print("Left, Right): "); Serial.print(obsCmLeft); Serial.print(", "); Serial.print(obsCmRight);

 return calculateDirection(obsCmLeft, obsCmRight);
}

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
    nextLat = WAY_POINTS[curWayPoint][0];
    nextLon = WAY_POINTS[curWayPoint][1];
    curSpeedOffset = WAY_POINTS[curWayPoint][2];
    return true;
  } else {
    return false;
  }
}

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
  // Serial.print(", angle to next : ");
  // Serial.print(AngleNext);
  // Serial.print(", Steering angle : ");
  // Serial.print(Steering);
  // Serial.print(", angleFromStraight : ");
  // Serial.print(angleFromStraight);
  // Serial.print(",  direction l or r : ");
  // Serial.print(DirectionNext);
  // Serial.print(", NextVectorY = ");
  // Serial.print(NextVectorY, 6);
  // Serial.print(", NextVectorX = ");
  // Serial.println(NextVectorX, 6);
  // Serial.print(", curWayPoint = ");
  // Serial.println(curWayPoint);
  // Serial.print("Lat, Long): "); Serial.print(curLat , 6); Serial.print(", "); Serial.print(curLng, 6);
  // Serial.print(", WP Lat, WP Long): "); Serial.print(nextLat, 6); Serial.print(", "); Serial.print(nextLon, 6);
  // Serial.print(", Unlocked?:"); Serial.print(checkAchievementStatus());
  // Serial.print(", WP#:"); Serial.print(curWayPoint);
  // Serial.print(", Dist2WP:"); Serial.print(distToNext);
  // Serial.print(", Cur Angle:"); Serial.print(gps.course.deg());
  // Serial.print(", Angle2WP: "); Serial.print(courseToWayPoint());
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