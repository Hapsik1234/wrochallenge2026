#define GATE_MOTEN 5
#define GATE_MOTO1 4
#define GATE_MOTO2 3
#define YELLOWPIN 6
#define BLUEPIN 2

#define pin 10

/*
==========TODO:===========

- DONE: check if motor is present (don't count to infinity) - propably by hardware
- move to visual studio code
- move to multifile setup
- DONE: fix motor angle counting (make it elegant)
- multiplatforming for different types of arduino, with tables in different files
- add debounce to ISR for antighosting

*/

int pinToPCINTbit[60];
int pinToPCIE[60];
uint8_t *pinToPCMSK[60]; // Unsure if this should be volatile
const int PCINTG0ports[8][2] = {
  { PCINT0, 53 },
  { PCINT1, 52 },
  { PCINT2, 51 },
  { PCINT3, 50 },
  { PCINT4, 10 },
  { PCINT5, 11 },
  { PCINT6, 12 },
  { PCINT7, 13 }
};

volatile uint8_t lastPINB = 0;
volatile uint8_t lastPIND = 0;

volatile uint8_t snapshotPINB = 0;
volatile uint8_t snapshotPIND = 0;
volatile uint8_t snapshotPINH = 0;

// TODO: make it static member of LegoMotor
uint8_t* regToSnapshot(volatile uint8_t* reg) {
  if (reg == &PINB) { return &snapshotPINB; }
  if (reg == &PIND) { return &snapshotPIND; }
  if (reg == &PINH) { return &snapshotPINH; }
  else { return nullptr; }
}


class LegoMotor {

private:
  
  bool motor_available;

  bool validate_tacho(uint8_t _pin) {

    // int map_length = sizeof(interuptMap) / sizeof(interuptMap[0]); // error: invalid application of 'sizeof' to incomplete type 'LegoMotor* []'
    int map_length = 8;
    if (_pin > map_length) { return false; }
    if (_pin <= 0) { return false; }  // Check if pin is outside of the map's range

    // TODO: check if port on list of possible ports, we can omit this - nothing will happen

    if (interuptMap[_pin] != nullptr) { return false; }  // Check if port is occupied

    return true;
  }

public:
  bool tacho_available; // TODO: Make private and add a getter
  // uint8_t *reg;
  // uint8_t bit;


  // Maps blue port of every object to the object; ISR then adds interupt to those ports calling count_degrees() for angle counting
  static LegoMotor* interuptMap[];
  // volatile static int maskPINB; // To be used in future, NOTE: I have no idea what I have meant by that comment



  volatile long absolute_degrees;  // Keeps track of how many degrees has the motor rotated so far

  int port_1, port_2, port_en, port_yellow, port_blue;  // TODO: change to uint8_t ?


  LegoMotor(int _port_1, int _port_2, int _port_en, int _port_yellow, int _port_blue) {
    // When ports blue and yellow are specified enable tachometer

    // Appending maps
    // TODO: predefine entire lookup table
    fill_lookups();

    lastPINB = PINB;
    lastPIND = PIND;


    pinMode(_port_en, OUTPUT);
    pinMode(_port_1, OUTPUT);
    pinMode(_port_2, OUTPUT);
    pinMode(_port_yellow, INPUT_PULLUP);
    pinMode(_port_blue, INPUT_PULLUP);

    port_1 = _port_1;
    port_2 = _port_2;
    port_en = _port_en;
    port_yellow = _port_yellow;
    port_blue = _port_blue;

    // uint8_t port = digitalPinToPort(_port_blue);
    // reg = portInputRegister(port);

    // bit = digitalPinToBitMask(_port_blue);

    absolute_degrees = 0;

    // Tacho setup:

    /*

    =========TODO:=========

    - check if port is occupied on the list and valid - DONE
    - find port group
    - enable group if not enabled already
    - enable interupt on pin
    - map to the list
    
    */

    validate_tacho(port_blue);

    if (port_blue == 10) {
      // Pin change interrupt request 0
      // Port group D

      PCICR |= (1 << pinToPCIE[port_blue]);  // Enable interrupt group 0 (PCINT0_vect)
      
      *pinToPCMSK[port_blue] |= (1 << pinToPCINTbit[port_blue]);  // Enable pin D10

      interuptMap[port_blue] = this;

      sei();
    }

    // After setting up tachometer

    tacho_available = true;
  }


  LegoMotor(int _port_1, int _port_2, int _port_en) {

    // Appending maps
    // TODO: predefine entire lookup table
    fill_lookups();

    pinMode(_port_en, OUTPUT);
    pinMode(_port_1, OUTPUT);
    pinMode(_port_2, OUTPUT);

    port_1 = _port_1;
    port_2 = _port_2;
    port_en = _port_en;
    port_yellow = -1;
    port_blue = -1;

    absolute_degrees = 0;

    tacho_available = false;
  }

  void fill_lookups() {
    pinToPCINTbit[53] = PCINT0;
    pinToPCINTbit[52] = PCINT1;
    pinToPCINTbit[51] = PCINT2;
    pinToPCINTbit[50] = PCINT3;

    pinToPCINTbit[10] = PCINT4;
    pinToPCINTbit[11] = PCINT5;
    pinToPCINTbit[12] = PCINT6;
    pinToPCINTbit[13] = PCINT7;
    
    pinToPCIE[53] = PCIE0;
    pinToPCIE[52] = PCIE0;
    pinToPCIE[51] = PCIE0;
    pinToPCIE[50] = PCIE0;

    pinToPCIE[10] = PCIE0;
    pinToPCIE[11] = PCIE0;
    pinToPCIE[12] = PCIE0;
    pinToPCIE[13] = PCIE0;

    pinToPCMSK[53] = &PCMSK0;
    pinToPCMSK[52] = &PCMSK0;
    pinToPCMSK[51] = &PCMSK0;
    pinToPCMSK[50] = &PCMSK0;

    pinToPCMSK[10] = &PCMSK0;
    pinToPCMSK[11] = &PCMSK0;
    pinToPCMSK[12] = &PCMSK0;
    pinToPCMSK[13] = &PCMSK0;
  }

  // This function is called every time digitalRead(this->port_blue) changes, indicating that motor has rotated by one degree in any direction
  void handle_rotation() {
    if (tacho_available) {

      // Check for direction of motion

      // TODO: define as attributes of LegoMotor
      uint8_t port = digitalPinToPort(port_blue);
      uint8_t *register_blue = portInputRegister(port);
      
      uint8_t *snapshot_blue = regToSnapshot(register_blue);
      uint8_t bit_blue = pinToPCINTbit[10];

      bool l_h = *snapshot_blue & (1<<bit_blue);
      bool b = snapshotPINH & (1<<3);

      bool h_l = !l_h;

      if (h_l ^ b) {
        absolute_degrees++;
      } else {
        absolute_degrees--;
      }

      Serial.println("degrees:" + String(10*absolute_degrees));
    }
  }


  // Functions for movement of the motor

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
    } else {
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

// Last known state of PIND (pins 0-7)


LegoMotor* LegoMotor::interuptMap[60] = { nullptr };

LegoMotor motor = LegoMotor(GATE_MOTO1, GATE_MOTO2, GATE_MOTEN, YELLOWPIN, pin);

void setup() {

  Serial.begin(2000000);
  // Serial.begin(9600);

  Serial.println(PCINTG0ports[0][0]);

  

  pinMode(10, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  // attachInterrupt(digitalPinToInterrupt(motor.port_blue), motor.CountDegrees, CHANGE); // DEPRECATED (found to be impractical to do with objects)

  motor.move(-30);
  delay(1000);
  motor.stop();
}

void loop() {
  // Serial.println(motor.absolute_degrees);
  // Serial.println("  " + String(PINB, BIN));
  // Serial.println(motor.tacho_available);


  // Toggle pin D9
  PORTH |= (1 << PH6);
  PORTH &= ~(1 << PH6);

  // Serial.println("PINH:" + String(PINH));
  // Serial.println("PINB:" + String(PINB));
  // Serial.println("degrees:" + String(10*motor.absolute_degrees));

  delay(50);
}


ISR(PCINT0_vect) {

  // Snapshot all registers
  snapshotPINB = PINB;
  snapshotPIND = PIND;
  snapshotPINH = PINH;

  // Calculate change mask
  int changePINB = lastPINB ^ snapshotPINB;
  changePINB = changePINB & PCMSK0;
  
  // Update last state
  lastPINB = snapshotPINB;
  // Serial.println("Change:" + String(changePINB, BIN));

  // TODO: add a for and lookup table for all pins
  
  for(int i=0;i<8;i++) {
    // Serial.print(PCINTG0ports[i][0]);
    // Serial.print(" ");
    // Serial.println(PCINTG0ports[i][1]);

    // TODO: expand to more lines to make it readable
    if (changePINB & (1 << PCINTG0ports[i][0])) { LegoMotor::interuptMap[PCINTG0ports[i][1]]->handle_rotation(); }
  }

  // if (changePINB & (1 << PCINT0)) { LegoMotor::interuptMap[4]->count_degrees(); }
  // if (changePINB & (1 << PCINT1)) { LegoMotor::interuptMap[4]->count_degrees(); }
  // if (changePINB & (1 << PCINT2)) { LegoMotor::interuptMap[4]->count_degrees(); }
  // if (changePINB & (1 << PCINT3)) { LegoMotor::interuptMap[4]->count_degrees(); }
  // if (changePINB & (1 << PCINT4)) { LegoMotor::interuptMap[10]->count_degrees(); } // else {Serial.println("NOW"); } // D10
  // if (changePINB & (1 << PCINT5)) { LegoMotor::interuptMap[4]->count_degrees(); }
  // if (changePINB & (1 << PCINT6)) { LegoMotor::interuptMap[4]->count_degrees(); }
  // if (changePINB & (1 << PCINT7)) { LegoMotor::interuptMap[4]->count_degrees(); }

}