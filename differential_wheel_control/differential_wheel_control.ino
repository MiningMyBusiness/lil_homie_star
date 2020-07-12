#include <NewPing.h>

// sudo chmod 666 /dev/ttyACM0
// sudo rfcomm bind 0 98:D3:31:FB:60:59 1
// to connect to my_BT003
// start cutecom
// pair with device and send commands 
// disconnect from device when done
// sudo rfcomm release rfcomm0
// to release my_BT003 from pair

////////////////////////////////////////////
///// variables for range sensing //////////
////////////////////////////////////////////

const int Trig_1 = 22;
const int Echo_1 = 23;
bool pinged_1 = false;
float distance_1 = 75.0;
const int Trig_2 = 24;
const int Echo_2 = 25;
bool pinged_2 = false;
float distance_2 = 75.0;
const int Trig_3 = 26;
const int Echo_3 = 27;
bool pinged_3 = false;
float distance_3 = 75.0;
const int Trig_4 = 28;
const int Echo_4 = 29;
bool pinged_4 = false;
float distance_4 = 75.0;
const int max_dist = 100; // maximum distance in cm
int between_time_sonar = 41; //ms to wait between sonar readings (there will be 4)
unsigned long last_ping_time = 0; // time the last sonar was pinged

NewPing sonar_1(Trig_1, Echo_1, max_dist);
NewPing sonar_2(Trig_2, Echo_2, max_dist);
NewPing sonar_3(Trig_3, Echo_3, max_dist);
NewPing sonar_4(Trig_4, Echo_4, max_dist);


/////////////////////////////////////////////////////
////// variables for pins for motor control /////////
/////////////////////////////////////////////////////

const int L_Vin1 = 2;      
const int L_Vin2 = 3;   
const int R_Vin1 = 4;      
const int R_Vin2 = 5; 
bool is_moving = false; // boolean to control movement command loop
int move_loop_time = 240; // time in ms to run last control signal
unsigned long last_move_time = 0;


/////////////////////////////////////////////
////// variables for rotary encoder /////////
/////////////////////////////////////////////

const int L_clk = 11;
const int L_dt = 10; 
const int R_clk = 9;
const int R_dt = 8;

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
int write_time_incr = 180; // num of ms to wait to write rotary encoder counter


///////////////////////////////////////////////////
///// variables for pwm differential control //////
///////////////////////////////////////////////////

int pwm_star_total = 80;
int pwm_star_diff = 0;
int pwm_star_right = 0;
int pwm_star_right_react = 0;
int pwm_star_left = 0;
int pwm_star_left_react = 0;
float d_thresh_rl = 50; // cm 
float d_thresh_f = 50; // cm
float k_l = 1.0;
float k_r = 1.0;
float k_f = 1.0;
float tau = 1.3; // for calculating k 


///////////////////////////////////////////////////
///// variables for control over serial comm //////
///////////////////////////////////////////////////

bool just_recieved_command = false;
int control = 0;


//////////////////////////////////////
///// variables for bump sensing /////
//////////////////////////////////////

const int bump_fr = 50; // front right bump sensor
const int bump_fl = 51; // front left bump sensor
const int bump_br = 52; // back right bump sensor
const int bump_bl = 53; // back left bump sensor 

bool bumped = false; // if any bump sensors have been activated while moving
unsigned long last_bump_time = 0;
int backup_dur = 500; // number of milliseconds to back away for
int spin_dur = 250; // number of milliseconds to spin for 
bool bumpState_f = false; // front left and right bump sensors activated
bool bumpState_b = false; // back left and right bump sensors activated
bool bumpState_fr = false; // only front right bump sensor activated 
bool bumpState_fl = false; // only front left bump sensor activated
bool bumpState_br = false; // only back right
bool bumpState_bl = false; // only back left

// variables for being bothered 
bool bothered = false; // if any bump sensors are activate while not moving 
unsigned long last_bother_time = 0;



/////////////////////////////////////////////////////////
//////////////////// SETUP //////////////////////////////
/////////////////////////////////////////////////////////

void setup() {
  // initialize the serial communication:
  Serial3.begin(9600);
  Serial3.setTimeout(25);
  // initialize pins 
  pinMode(L_Vin1, OUTPUT); // MOTOR CONTROL PINS
  pinMode(L_Vin2, OUTPUT); // MOTOR CONTROL PINS
  pinMode(R_Vin1, OUTPUT); // MOTOR CONTROL PINS
  pinMode(R_Vin2, OUTPUT); // MOTOR CONTROL PINS
  pinMode(L_clk, INPUT); // ROTARY ENCODER PINS
  pinMode(L_dt, INPUT); // ROTARY ENCODER PINS
  pinMode(R_clk, INPUT); // ROTARY ENCODER PINS
  pinMode(R_dt, INPUT); // ROTARY ENCODER PINS
  pinMode(bump_fr, INPUT); // BUMP SENSOR PINS
  pinMode(bump_fl, INPUT); // BUMP SENSOR PINS
  pinMode(bump_br, INPUT); // BUMP SENSOR PINS
  pinMode(bump_bl, INPUT); // BUMP SENSOR PINS
  // set all to low to start with
  digitalWrite(L_Vin1, LOW);
  digitalWrite(L_Vin2, LOW);
  digitalWrite(R_Vin1, LOW);
  digitalWrite(R_Vin1, LOW);
  // read the initial state of CLK
  lastState_L_clk = digitalRead(L_clk);
  lastState_R_clk = digitalRead(R_clk);
}


/////////////////////////////////////////////////////////
///////////////// MAIN CONTROL LOOP /////////////////////
/////////////////////////////////////////////////////////

void loop() {
  check_control_loop(); // start by checking to see if motion should be stopped
  sonar_control_loop();

   // check if data has been sent from the computer:
  while (Serial3.available() > 0) {
    // read the most recent byte (which will be from 0 to 255):
    control = Serial3.parseInt();
    just_recieved_command = true;
    Serial3.print("Recieved: ");
    Serial3.println(control);
  }

  if (just_recieved_command) {
    just_recieved_command = false;
    // BRAKE
    if (control == 0) {
      is_moving = false;
      apply_brakes();
    } else {
      // DELIBERATIVE CONTROL COMMAND
      reset_control_loop();
    }
  }

  // REACTIVE CONTROL COMMAND
  if (is_moving) {
    pwm_star_diff = map(control, 1, 201, -1*pwm_star_total, pwm_star_total);
    pwm_star_diff = constrain(pwm_star_diff, -1*pwm_star_total, pwm_star_total);
    pwm_star_right = (pwm_star_total + pwm_star_diff)/2;
    pwm_star_left = (pwm_star_total - pwm_star_diff)/2;
    update_pwm_star_right_left();
  }


  // CHECK FOR BUMPS AND BOTHERS
  check_bump_sensors();
  check_bump_control();
  check_bother_control();
  

  // EXECUTE MOTOR COMMAND
  if (is_moving && !bumped) {
    int pwm_right = 90 + pwm_star_right_react;
    int pwm_left = 90 + pwm_star_left_react;
    pwm_right = constrain(pwm_right, 0, 255);
    pwm_left = constrain(pwm_left, 0, 255);
    analogWrite(R_Vin1, pwm_right);
    analogWrite(L_Vin2, pwm_left);
  }

  // read current state of clk 
  read_L_encoder();
  delay(2);
  read_R_encoder();
  delay(2);

  if (millis() > (last_write_time + write_time_incr)) {
    last_write_time = millis();
    update_pwm_star_total(); // closed loop speed control, if moving
    write_encoder_data();
    reset_encoders();
  }
}

///////////////////////////////////////////////////////////
/////////////// ROTARY ENCODER ROUTINES ///////////////////
///////////////////////////////////////////////////////////

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


/////////////////////////////////////////////////////////////////
//////////////// MOTOR CONTROL LOOP ROUTINES ////////////////////
/////////////////////////////////////////////////////////////////

// apply brakes
void apply_brakes() {
  digitalWrite(L_Vin1, LOW);
  digitalWrite(L_Vin2, LOW);
  digitalWrite(R_Vin1, LOW);
  digitalWrite(R_Vin2, LOW);
}

// right wheel forward 
void right_forward() {
  int pwm_right = 90 + pwm_star_total/2;
  pwm_right = constrain(pwm_right, 0, 255);
  analogWrite(R_Vin1, pwm_right);
}

// right wheel back
void right_backward() {
  int pwm_right = 90 + pwm_star_total/2;
  pwm_right = constrain(pwm_right, 0, 255);
  analogWrite(R_Vin2, pwm_right);
}

// left wheel forward 
void left_forward() {
  int pwm_left = 90 + pwm_star_total/2;
  pwm_left = constrain(pwm_left, 0, 255);
  analogWrite(L_Vin2, pwm_left);
}

// left wheel back
void left_backward() {
  int pwm_left = 90 + pwm_star_total/2;
  pwm_left = constrain(pwm_left, 0, 255);
  analogWrite(L_Vin1, pwm_left);
}

// move forward
void move_forward() {
  right_forward();
  left_forward();
}

// move backward 
void move_backward() {
  right_backward();
  left_backward();
}

// spin right 
void spin_right() {
  right_backward();
  left_forward();
}

// spin left
void spin_left() {
  right_forward();
  left_backward();
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



/////////////////////////////////////////////////////////////////////
/////////////////// SONAR SENSING LOOP ROUTINES /////////////////////
/////////////////////////////////////////////////////////////////////

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
}


//////////////////////////////////////////////////////////////////
////////////////// REACTIVE CONTROL ROUTINES /////////////////////
//////////////////////////////////////////////////////////////////

// left wheel multiplier
void update_k_l() {
  k_l = 1.0;
  if (distance_4 > 0.0) {
    if (distance_4 < d_thresh_rl) {
      k_l = pow(distance_4/d_thresh_rl, tau);
    }
  }
}

// right wheel multiplier
void update_k_r() {
  k_r = 1.0;
  if (distance_2 > 0.0) {
    if (distance_2 < d_thresh_rl) {
      k_r = pow(distance_2/d_thresh_rl, tau);
    }
  }
}

// front direction multiplier
void update_k_f() {
  k_f = 1.0;
  if (distance_1 > 0.0) {
    if (distance_1 < d_thresh_f) {
      k_f = pow(distance_1/d_thresh_f, tau);
    }
    if (distance_1 < 10.0) { // don't crash into stuff
      k_f = 0;
      Serial3.println("WALL");
    }
  }
}


// enact reactive control on both wheels
void update_pwm_star_right_left() {
  update_k_l();
  update_k_r();
  update_k_f();
  pwm_star_right_react = int(k_r*(2.0 - k_l)*k_f*float(pwm_star_right));
  pwm_star_left_react = int(k_l*(2.0 - k_r)*k_f*float(pwm_star_left));
}


// closed loop speed control 
void update_pwm_star_total() {
  float avg_cnt = (abs(float(R_counter)) + abs(float(L_counter)))/2.0;
  if (is_moving && k_f != 0) { // if you're moving and the front reactive multiplier is not zero
    if (avg_cnt < 3) {
      pwm_star_total += 10;
    }
    if (avg_cnt > 4) {
      pwm_star_total -= 10;
    }
  }
}


//////////////////////////////////////////////////////////////
//////////////// BUMP SENSOR ROUTINES ////////////////////////
//////////////////////////////////////////////////////////////

void check_bump_sensors() {
  bumpState_fr = digitalRead(bump_fr); 
  bumpState_fl = digitalRead(bump_fl);
  bumpState_br = digitalRead(bump_br);
  bumpState_bl = digitalRead(bump_bl); 
  if (bumpState_fr || bumpState_fl || bumpState_br || bumpState_bl) {
    if (is_moving) {
      bumped = true;
      last_bump_time = millis();
    } else {
      bothered = true;
      last_bother_time = millis();
    }
  }
  if (bumpState_fr && bumpState_fl) {
    bumpState_f = true;
    bumpState_fr = false;
    bumpState_fl = false;
  }
  if (bumpState_br && bumpState_bl) {
    bumpState_b = true;
    bumpState_br = false;
    bumpState_bl = false;
  }
}


void check_bump_control() {
  if (bumped) {
    if (millis() - last_bump_time > (backup_dur + spin_dur)) {
      bumped = false;
    }
  }
  if (bumped) {
    if (millis() - last_bump_time < backup_dur) { // backup phase
      if (bumpState_f || bumpState_fr || bumpState_fl) {
        move_backward();
      }
      if (bumpState_b || bumpState_br || bumpState_bl) {
        move_forward();
      }
    } else if (millis() - last_bump_time <= (backup_dur + spin_dur)) { // spin phase
      if (bumpState_fr || bumpState_br) {
        spin_left();
      } else if (bumpState_fl || bumpState_bl) {
        spin_right();
      } else {
        if (distance_2 == distance_4) {
          spin_right();
        } else if (distance_4 == 0) {
          spin_right();
        } else if (distance_2 == 0) {
          spin_left();
        } else if (distance_2 >= distance_4) {
          spin_left();
        } else if (distance_4 >= distance_2) {
          spin_right();
        }
      }
    }
  }
}

void check_bother_control() {
  if (bothered) {
    if (millis() - last_bother_time > backup_dur) {
      bothered = false;
    }
  }
  if (bothered) {
    if (bumpState_fr) {
      right_backward();
    } 
    if (bumpState_fl) {
      left_backward();
    }
    if (bumpState_f) {
      right_backward();
      left_backward();
    }
    if (bumpState_br) {
      right_forward();
    }
    if (bumpState_bl) {
      left_forward();
    }
    if (bumpState_b) {
      right_forward();
      left_forward();
    }
  }
}
