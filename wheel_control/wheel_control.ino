#include <NewPing.h>

// sudo chmod 666 /dev/ttyACM0
// sudo rfcomm bind 0 98:D3:31:FB:60:59 1
// to connect to my_BT003
// start cutecom
// pair with device and send commands 
// disconnect from device when done
// sudo rfcomm release rfcomm0
// to release my_BT003 from pair

const int Trig_1 = 22;
const int Echo_1 = 23;
bool pinged_1 = false;
float distance_1 = -99.0;
const int Trig_2 = 24;
const int Echo_2 = 25;
bool pinged_2 = false;
float distance_2 = -99.0;
const int Trig_3 = 26;
const int Echo_3 = 27;
bool pinged_3 = false;
float distance_3 = -99.0;
const int Trig_4 = 28;
const int Echo_4 = 29;
bool pinged_4 = false;
float distance_4 = -99.0;
const int max_dist = 100; // maximum distance in cm
int between_time_sonar = 41; //ms to wait between sonar readings
unsigned long last_ping_time = 0; // time the last sonar was pinged

NewPing sonar_1(Trig_1, Echo_1, max_dist);
NewPing sonar_2(Trig_2, Echo_2, max_dist);
NewPing sonar_3(Trig_3, Echo_3, max_dist);
NewPing sonar_4(Trig_4, Echo_4, max_dist);


const int L_Vin1 = 2;      
const int L_Vin2 = 3;
const int L_clk = 11;
const int L_dt = 10;    
const int R_Vin1 = 4;      
const int R_Vin2 = 5; 
const int R_clk = 9;
const int R_dt = 8;
bool is_moving = false; // boolean to control movement command loop
int move_loop_time = 180; // time in ms to run last control signal
unsigned long last_move_time = 0;

int L_counter = 0;
int R_counter = 0;
int L_revs = 0;
int R_revs = 0;

int currentState_L_clk;
int lastState_L_clk;
int currentState_R_clk;
int lastState_R_clk;
String L_dir = "";
String R_dir = "";

unsigned long last_write_time = 0;
int write_time_incr = move_loop_time; // num of ms to wait to write rotary encoder counter


void setup() {
  // initialize the serial communication:
  Serial3.begin(9600);
  Serial3.setTimeout(25);
  // initialize pins 
  pinMode(L_Vin1, OUTPUT);
  pinMode(L_Vin2, OUTPUT);
  pinMode(R_Vin1, OUTPUT);
  pinMode(R_Vin2, OUTPUT);
  pinMode(L_clk, INPUT);
  pinMode(L_dt, INPUT);
  pinMode(R_clk, INPUT);
  pinMode(R_dt, INPUT);
  // set all to low to start with
  digitalWrite(L_Vin1, LOW);
  digitalWrite(L_Vin2, LOW);
  digitalWrite(R_Vin1, LOW);
  digitalWrite(R_Vin1, LOW);
  // read the initial state of CLK
  lastState_L_clk = digitalRead(L_clk);
  lastState_R_clk = digitalRead(R_clk);
}

void loop() {
  check_control_loop(); // start by checking to see if motion should be stopped
  
  char control;
  control = 0;

   // check if data has been sent from the computer:
  if (Serial3.available()) {
    // read the most recent byte (which will be from 0 to 255):
    control = Serial3.read();
  }
  // FORWARD MOVEMENT 
  if (control == 'F') { // slow
    reset_control_loop();
    analogWrite(L_Vin1, 0);
    analogWrite(R_Vin1, 110);
    analogWrite(L_Vin2, 110);
    analogWrite(R_Vin2, 0);
  }
  if (control == 'G') { // medium
    reset_control_loop();
    analogWrite(L_Vin1, 0);
    analogWrite(R_Vin1, 150);
    analogWrite(L_Vin2, 150);
    analogWrite(R_Vin2, 0);
  }
  if (control == 'H') { // fast
    reset_control_loop();
    analogWrite(L_Vin1, 0);
    analogWrite(R_Vin1, 200);
    analogWrite(L_Vin2, 200);
    analogWrite(R_Vin2, 0);
  }
  if (control == 'J') { // ultra-fast
    reset_control_loop();
    analogWrite(L_Vin1, 0);
    analogWrite(R_Vin1, 255);
    analogWrite(L_Vin2, 255);
    analogWrite(R_Vin2, 0);
  }

  // BACKWARD MOVEMENT
  if (control == 'B') { // slow
    reset_control_loop();
    analogWrite(L_Vin1, 110);
    analogWrite(R_Vin1, 0);
    analogWrite(L_Vin2, 0);
    analogWrite(R_Vin2, 110);
  }
  if (control == 'V') { // medium
    reset_control_loop();
    analogWrite(L_Vin1, 150);
    analogWrite(R_Vin1, 0);
    analogWrite(L_Vin2, 0);
    analogWrite(R_Vin2, 150);
  }
  if (control == 'C') { // fast
    reset_control_loop();
    analogWrite(L_Vin1, 200);
    analogWrite(R_Vin1, 0);
    analogWrite(L_Vin2, 0);
    analogWrite(R_Vin2, 200);
  }
  if (control == 'X') { // ultra-fast
    reset_control_loop();
    analogWrite(L_Vin1, 255);
    analogWrite(R_Vin1, 0);
    analogWrite(L_Vin2, 0);
    analogWrite(R_Vin2, 255);
  }

  // RIGHT TURNING MOVEMENT
  if (control == 'P') { // slow 
    reset_control_loop();
    analogWrite(L_Vin1, 110);
    analogWrite(R_Vin1, 110);
    analogWrite(L_Vin2, 0);
    analogWrite(R_Vin2, 0);
  }
  if (control == 'O') { // medium
    reset_control_loop();
    analogWrite(L_Vin1, 150);
    analogWrite(R_Vin1, 150);
    analogWrite(L_Vin2, 0);
    analogWrite(R_Vin2, 0);
  }
  if (control == 'I') { // fast
    reset_control_loop();
    analogWrite(L_Vin1, 200);
    analogWrite(R_Vin1, 200);
    analogWrite(L_Vin2, 0);
    analogWrite(R_Vin2, 0);
  }
  if (control == 'U') { // ultra-fast
    reset_control_loop();
    analogWrite(L_Vin1, 255);
    analogWrite(R_Vin1, 255);
    analogWrite(L_Vin2, 0);
    analogWrite(R_Vin2, 0);
  }

  // LEFT TURNING MOVEMENT
  if (control == 'Q') { // slow
    reset_control_loop();
    analogWrite(L_Vin1, 0);
    analogWrite(R_Vin1, 0);
    analogWrite(L_Vin2, 100);
    analogWrite(R_Vin2, 110);
  }
  if (control == 'W') { // medium
    reset_control_loop();
    analogWrite(L_Vin1, 0);
    analogWrite(R_Vin1, 0);
    analogWrite(L_Vin2, 150);
    analogWrite(R_Vin2, 150);
  }
  if (control == 'E') { // fast
    reset_control_loop();
    analogWrite(L_Vin1, 0);
    analogWrite(R_Vin1, 0);
    analogWrite(L_Vin2, 200);
    analogWrite(R_Vin2, 200);
  }
  if (control == 'R') { // ultra-fast
    reset_control_loop();
    analogWrite(L_Vin1, 0);
    analogWrite(R_Vin1, 0);
    analogWrite(L_Vin2, 255);
    analogWrite(R_Vin2, 255);
  }

  // BRAKE
  if (control == 'S') {
    is_moving = false;
    apply_brakes();
  }

  // read current state of clk 
  read_L_encoder();
  delay(1);
  read_R_encoder();
  delay(1);

  if (millis() > (last_write_time + write_time_incr)) {
    last_write_time = millis();
    write_encoder_data();
    reset_encoders();
  }

  sonar_control_loop();
}


// read the left wheel rotary encoder 
void read_L_encoder() {
  // read current state of clk 
  currentState_L_clk = digitalRead(L_clk);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentState_L_clk != lastState_L_clk  && currentState_L_clk == 1){
    
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(L_dt) != currentState_L_clk) {
      L_counter --;
      L_dir ="CCW";
    } else {
      // Encoder is rotating CW so increment
      L_counter ++;
      L_dir ="CW";
    }
  }

  // remember the last CLK state
  lastState_L_clk = currentState_L_clk;
}


// read the right wheel rotary encoder
void read_R_encoder() {
  // read current state of clk 
  currentState_R_clk = digitalRead(R_clk);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentState_R_clk != lastState_R_clk  && currentState_R_clk == 1){
    
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(R_dt) != currentState_R_clk) {
      R_counter --;
      R_dir ="CCW";
    } else {
      // Encoder is rotating CW so increment
      R_counter ++;
      R_dir ="CW";
    }
  }
  // remember the last CLK state
  lastState_R_clk = currentState_R_clk;
}


// write rotary encoder data 
void write_encoder_data() {
  Serial3.print("LDir: ");
  Serial3.print(L_dir);
  Serial3.print(" | LCount: ");
  Serial3.print(L_counter);
  Serial3.print(" | RDir: ");
  Serial3.print(R_dir);
  Serial3.print(" | RCount: ");
  Serial3.println(R_counter);
}


// reset rotary encoder counters 
void reset_encoders() {
  R_dir = "";
  L_dir = "";
  R_counter = 0;
  L_counter = 0;
}

// apply brakes
void apply_brakes() {
  digitalWrite(L_Vin1, LOW);
  digitalWrite(L_Vin2, LOW);
  digitalWrite(R_Vin1, LOW);
  digitalWrite(R_Vin2, LOW);
}


// reset control loop timing
void reset_control_loop() {
  is_moving = true;
  last_move_time = millis();
}

// check control loop to see if agent should stop moving
void check_control_loop() {
  if (is_moving) {
    if (millis() > (last_move_time + move_loop_time)){
      is_moving = false;
      apply_brakes();
    }
  }
}

// control the sonar reading
void sonar_control_loop() {
  if (millis() > (last_ping_time + between_time_sonar)) {
    if (!pinged_1) {
      last_ping_time = millis();
      pinged_1 = true;
      distance_1 = sonar_1.ping_cm();
    } else if (!pinged_2) {
      last_ping_time = millis();
      pinged_2 = true;
      distance_2 = sonar_2.ping_cm();
    } else if (!pinged_3) {
      last_ping_time = millis();
      pinged_3 = true;
      distance_3 = sonar_3.ping_cm();
    } else if (!pinged_4) {
      last_ping_time = millis();
      pinged_4 = true;
      distance_4 = sonar_4.ping_cm();
    } else {
      pinged_1 = false;
      pinged_2 = false;
      pinged_3 = false;
      pinged_4 = false;
      write_sonar();
    }
  }
}

// write sonar readings 
void write_sonar() {
  Serial3.print("S1: ");
  Serial3.print(distance_1);
  Serial3.print(" cm");
  Serial3.print(" | S2: ");
  Serial3.print(distance_2);
  Serial3.print(" cm");
  Serial3.print(" | S3: ");
  Serial3.print(distance_3);
  Serial3.print(" cm");
  Serial3.print(" | S4: ");
  Serial3.print(distance_4);
  Serial3.println(" cm");
  distance_1 = -99.0;
  distance_2 = -99.0;
  distance_3 = -99.0;
  distance_4 = -99.0;
}
