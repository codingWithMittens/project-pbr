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