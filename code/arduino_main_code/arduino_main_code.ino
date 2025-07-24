#define GATE_MOTEN 5
#define GATE_MOTO1 4
#define GATE_MOTO2 3

#define YELLOWPIN 6
#define BLUEPIN 2

volatile long degrees = 0;


/*
==========TODO:===========

- check if motor is present (don't count to infinity)
- move to visual studio code
- move to multifile setup
- fix motor angle counting (make it elegant)

*/


class LegoMotor {

  private:
  bool tacho_available;

  public:

  int port_1, port_2, port_en, port_yellow, port_blue;

  LegoMotor(int _port_1, int _port_2, int _port_en, int _port_yellow, int _port_blue) {
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

  void move(int speed) {
    if (speed > 100)
      speed = 100;
    if (speed < -100)
      speed = -100;

    analogWrite(port_en, int(abs(speed) * 2.55));

    if (speed > 0)
    {
      digitalWrite(port_1, HIGH);
      digitalWrite(port_2, LOW);
    }
    else
    {
      digitalWrite(port_1, LOW);
      digitalWrite(port_2, HIGH);
    }
  }


  void stop() {
    digitalWrite(port_1, LOW);
    digitalWrite(port_2, LOW);
  }

  void stop_brakes() {
    digitalWrite(port_1, LOW);
    digitalWrite(port_2, LOW);
    digitalWrite(port_en, LOW);
  }
  
};

LegoMotor motor = LegoMotor(GATE_MOTO1, GATE_MOTO2, GATE_MOTEN, YELLOWPIN, BLUEPIN);

void CountDegrees() {
    if (digitalRead(motor.port_yellow) == digitalRead(motor.port_blue)) {
      degrees++;
    }
    else {
      degrees--;
    }
    
}

void setup() {
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(motor.port_blue), CountDegrees, CHANGE);

  motor.move(-80);
}

void loop() {
  Serial.println(degrees);

}