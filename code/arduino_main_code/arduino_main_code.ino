#define GATE_MOTEN 5
#define GATE_MOTO1 4
#define GATE_MOTO2 3
#define YELLOWPIN 6
#define BLUEPIN 2


/*
==========TODO:===========

- check if motor is present (don't count to infinity)
- move to visual studio code
- move to multifile setup
- fix motor angle counting (make it elegant)
- add more comments
- multiplatforming for different types of arduino, with tables in different files

*/


class LegoMotor {

  private:
  bool tacho_available;

  public:

  static LegoMotor* interuptMap[]; 
  // Maps blue port of every object to the object; ISR then adds interupt to those ports calling CountDegrees() for angle counting 
  // In future consider mapping only to function and not entire object

  // NOTE: I just realized that it may be faster to do the counting inside ISR funtion which could be just messing with object variables instead of calling another function


  volatile long absolute_degrees = 0; // Keeps track of how many degrees has the motor rotated so far
  int port_1, port_2, port_en, port_yellow, port_blue;
  

  LegoMotor(int _port_1, int _port_2, int _port_en, int _port_yellow, int _port_blue) {
    // When ports blue and yellow specified enable tachometer
    
    pinMode(_port_en, OUTPUT);
    pinMode(_port_1, OUTPUT);
    pinMode(_port_2, OUTPUT);
    pinMode(_port_yellow, INPUT);
    pinMode(_port_blue, INPUT);

    port_1 = _port_1;
    port_2 = _port_2;
    port_en = _port_en;
    port_yellow = _port_yellow;
    port_blue = _port_blue;

    tacho_available = true; 
    
    
    /*

    =========TODO:=========

    - check if port is occupied on the list and valid
    - find port group
    - enable group if not enabled already
    - enable interupt on pin
    - map to the list
    
    */
    
  }

  LegoMotor(int _port_1, int _port_2, int _port_en) {
    pinMode(_port_en, OUTPUT);
    pinMode(_port_1, OUTPUT);
    pinMode(_port_2, OUTPUT);

    port_1 = _port_1;
    port_2 = _port_2;
    port_en = _port_en;
    port_yellow = -1;
    port_blue = -1;

    tacho_available = false; 
  }

  void CountDegrees() {
  if (tacho_available) {

    // Check for direction of motion
    if (digitalRead(port_yellow) == digitalRead(port_blue)) { 
      absolute_degrees++;
    } else {
      absolute_degrees--;
    }

  }}

  // Functions for movement motors

  void move(int speed) {

    // Saturation of speed for safety
    if (speed > 100)
      speed = 100;
    if (speed < -100)
      speed = -100;

    analogWrite(port_en, int(abs(speed) * 2.55));

    if (speed > 0) {
      digitalWrite(port_1, HIGH);
      digitalWrite(port_2, LOW);
    }
    else {
      digitalWrite(port_1, LOW);
      digitalWrite(port_2, HIGH);
    }
  }

  void stop() {
    digitalWrite(port_1, LOW);
    digitalWrite(port_2, LOW);
  }

  void stop_with_brakes() {
    digitalWrite(port_1, LOW);
    digitalWrite(port_2, LOW);
    digitalWrite(port_en, LOW);
  }
  
};

LegoMotor motor = LegoMotor(GATE_MOTO1, GATE_MOTO2, GATE_MOTEN, YELLOWPIN, BLUEPIN);


void setup() {
  Serial.begin(9600);

  // attachInterrupt(digitalPinToInterrupt(motor.port_blue), motor.CountDegrees, CHANGE);

  motor.move(-80);
}

void loop() {
  Serial.println(degrees);

}


// ===================== THIS IS A CODE COPIED FROM CHATGPT SERVING AS REFERENCE =====================

// #define GATE_MOTEN 5
// #define GATE_MOTO1 4
// #define GATE_MOTO2 3

// #define YELLOWPIN 6
// #define BLUEPIN 62  // A8 on Arduino Mega (digital pin 62)

// class LegoMotor;

// // Global table mapping encoder pin to motor object
// LegoMotor* motorMap[70] = { nullptr };

// // Last known state of PORTK (pins 62–69 = A8–A15)
// volatile uint8_t lastPinKState = 0;

// class LegoMotor {
// private:
//   bool tacho_available;

// public:
//   int port_1, port_2, port_en, port_yellow, port_blue;
//   volatile long degrees;

//   LegoMotor(int _port_1, int _port_2, int _port_en, int _port_yellow, int _port_blue) {
//     port_1 = _port_1;
//     port_2 = _port_2;
//     port_en = _port_en;
//     port_yellow = _port_yellow;
//     port_blue = _port_blue;
//     degrees = 0;

//     pinMode(port_1, OUTPUT);
//     pinMode(port_2, OUTPUT);
//     pinMode(port_en, OUTPUT);
//     pinMode(port_yellow, INPUT);
//     pinMode(port_blue, INPUT);

//     tacho_available = true;

//     // Map this object to the encoder pin
//     motorMap[port_blue] = this;

//     // Enable PCINT for PCINT group 2 (PORTK, pins 62–69)
//     PCICR |= (1 << PCIE2);  // Enable PCINT2 interrupt group
//     PCMSK2 |= (1 << (port_blue - 62));  // Enable interrupt for this pin in PCMSK2

//     // Initialize lastPinKState if needed
//     lastPinKState = PIND;
//   }

//   void CountDegrees() {
//     if (tacho_available) {
//       if (digitalRead(port_yellow) == digitalRead(port_blue)) {
//         degrees++;
//       } else {
//         degrees--;
//       }
//     }
//   }

//   void move(int speed) {
//     if (speed > 100) speed = 100;
//     if (speed < -100) speed = -100;

//     analogWrite(port_en, int(abs(speed) * 2.55));

//     if (speed > 0) {
//       digitalWrite(port_1, HIGH);
//       digitalWrite(port_2, LOW);
//     } else {
//       digitalWrite(port_1, LOW);
//       digitalWrite(port_2, HIGH);
//     }
//   }

//   void stop() {
//     digitalWrite(port_1, LOW);
//     digitalWrite(port_2, LOW);
//   }

//   void stop_brakes() {
//     digitalWrite(port_1, LOW);
//     digitalWrite(port_2, LOW);
//     digitalWrite(port_en, LOW);
//   }
// };

// // Create one motor for demo
// LegoMotor motor(GATE_MOTO1, GATE_MOTO2, GATE_MOTEN, YELLOWPIN, BLUEPIN);

// void setup() {
//   Serial.begin(9600);
//   motor.move(80);
//   delay(3000);
//   motor.stop();
// }

// void loop() {
//   Serial.println(motor.degrees);
//   delay(100);
// }

// // ISR for PCINT2 group (PORTK = A8 to A15 = pins 62–69)
// ISR(PCINT2_vect) {
//   uint8_t newState = PIND;
//   uint8_t changed = newState ^ lastPinKState;
//   lastPinKState = newState;

//   for (int pin = 62; pin <= 67; pin++) {
//     uint8_t mask = 1 << (pin - 62);
//     if (changed & mask) {
//       if (motorMap[pin]) {
//         motorMap[pin]->CountDegrees();
//       }
//     }
//   }
// }