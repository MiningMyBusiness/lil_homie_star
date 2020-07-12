import processing.serial.*;

Serial myport; // Create object from Serial class
String myString = null;
String control = null;

// robot starting position and orientation
float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;

/// target locations
/// (140, 140) between computer desk and dining chair
/// (230, 0) at the left corner of the couch and tv table 
/// (-100, 130) by the leg of the bed and the piano (bedroom)
/// (210, 260) by the work bench and the dinig table (3D printer)
/// (100, 380) at the edge of the carpet by the kitchen
float target_x = 230;
float target_y = 0;
float r_thresh = 15; // threshold error for reaching target

/// distance sensor readings
float my_s1 = 75;
float my_s2 = 75;
float my_s3 = 75;
float my_s4 = 75;


/// wheel counts 
float lcnt = 0.0;
float rcnt = 0.0;
float del_t = 0.183; // time between wheel counts 
long time_wheel_read = 0;
int num_wheel_reads = 0;
float del_theta_max = radians(15); // the most that can be turned at once by the robot in 183 ms


/// robot parameters 
float wheel_circ = 21.2; //cm 
float clics_per_rev = 20.0; // number of clicks per revolution
float dist_bet_wheels = 15.8; //cm


/// commands
int diff_drive_command = 0;

/// program state
boolean start_control = false;
long last_write_time = 0;
int write_time_diff = 60; // time in ms to wait before writing commands



void setup() {
  size(440,440);
  myport = new Serial(this, "/dev/rfcomm0", 9600);
  myport.bufferUntil('\n');
}

void draw() {
  
  // read serial port 
  while (myport.available() > 0) {
    myString = myport.readStringUntil('\n');
    parse_serial_data();
    if (abs(lcnt) > 0.0 || abs(rcnt) > 0.0) {
      update_robot_position();
    }
    compute_differential_drive();
  }
  
  visualize_walls();
  
  if (start_control) {
    if ((millis() - last_write_time) > write_time_diff) {
      myport.write(str(diff_drive_command));
      last_write_time = millis();
    }
  }
  
}


void keyPressed() {
  if (key == 'z' || key == 'Z') {
    diff_drive_command = 0;
    myport.write(diff_drive_command);
    exit();
  }
  if (key == 's' || key == 'S') {
    if (start_control) {
      start_control = false;
    } else {
      start_control = true;
    }
  }
}


void parse_serial_data() {
  lcnt = 0.0;
  rcnt = 0.0;
  if (myString != null) {
    String[] list = split(myString, ":");
    //if (list[0].equals("Recieved")) {
    //  println(myString);
    //}
    if (list[0].equals("S1") == true) {
      String[] list_s1 = split(list[1], " cm");
      my_s1 = float(list_s1[0]);
      String[] list_s2 = split(list[2], " cm");
      my_s2 = float(list_s2[0]);
      String[] list_s3 = split(list[3], " cm");
      my_s3 = float(list_s3[0]);
      String[] list_s4 = split(list[4], " cm");
      my_s4 = float(list_s4[0]);
    }
    String[] list2 = split(myString, ": ");
    if (list2[0].equals("LDir") == true) {
      if (num_wheel_reads < 2) {
        num_wheel_reads += 1;
        time_wheel_read = millis();
      } else {
        int ms_diff = int(millis() - time_wheel_read);
        del_t = float(ms_diff)/1000.0;
        time_wheel_read = millis();
      }
      String[] list_lcnt = split(list2[2], " |");
      lcnt = float(list_lcnt[0]);
      if (lcnt > 0.0) {
        lcnt += 0.5;
      }
      if (lcnt < 0.0) {
        lcnt -= 0.5;
      }
      rcnt = float(list2[4]);
      if (rcnt > 0.0) {
        rcnt += 0.5;
      }
      if (rcnt < 0.0) {
        rcnt -= 0.5;
      }
      if (abs(rcnt) > 0.0) {
        rcnt = -1*rcnt; // correct for flipped direction
      }
    }
  }
}


void visualize_walls() {
  background(125);
  rect(width/2 - 20, height/2 - 20, 40, 40); // this is the car 
  ellipse(width/2 - 10, height/2 - 10, 10, 10);
  ellipse(width/2 + 10, height/2 - 10, 10, 10);
  if (my_s1 > 0) {
    rect(width/4, 0, width/2, map(my_s1, 0, 100, height/2 - 20, 0));
  }
  if (my_s2 > 0) {
    rect(0, height/4, map(my_s2, 0, 100, width/2 - 20, 0), height/2);
  }
  if (my_s3 > 0) {
    rect(width/4, map(my_s3, 0, 100, height/2 + 20, height), width/2, height - map(my_s3, 0, 100, height/2 + 20, height));
  }
  if (my_s4 > 0) {
    rect(map(my_s4, 0, 100, width/2 + 20, width), height/4, width - map(my_s4, 0, 100, width/2 + 20, width), height/2);
  }
}


void update_robot_position() {
  float l_dist = wheel_circ*lcnt/clics_per_rev;
  float r_dist = wheel_circ*rcnt/clics_per_rev;
  print(r_dist);
  print(", ");
  println(l_dist);
  float omega_delt = (r_dist - l_dist)/dist_bet_wheels;
  float v_r = r_dist/del_t;
  float v_l = l_dist/del_t;
  if (abs(v_l) > 0.00 || abs(v_r) > 0.00) {
    //float R = (dist_bet_wheels/2)*((v_l + v_r)/(v_r - v_l));
    //float term1 = R*(sin(robot_theta)*cos(omega_delt) + cos(robot_theta)*sin(omega_delt));
    //float term2 = R*(sin(robot_theta)*sin(omega_delt) - cos(robot_theta)*cos(omega_delt));
    //float ICC_x = robot_x - R*sin(robot_theta);
    //float ICC_y = robot_y + R*cos(robot_theta);
    ///// update values
    //robot_x = ICC_x + term1;
    //robot_y = ICC_y + term2;
    float term1 = (r_dist + l_dist)/2.0;
    float term2 = (r_dist - l_dist)/(2.0*dist_bet_wheels);
    float term3 = robot_theta + term2;
    robot_x += term1*cos(term3);
    robot_y +=  term1*sin(term3);
    robot_theta += omega_delt;
  }
  print(robot_x);
  print(", ");
  print(robot_y);
  print(", ");
  println(degrees(robot_theta));
}


void compute_differential_drive() { 
  PVector to_target = new PVector(target_x - robot_x, target_y - robot_y);
  float r = to_target.mag();
  if (r <= r_thresh) {
    diff_drive_command = 0;
  } else {
    PVector orient = new PVector(cos(robot_theta), sin(robot_theta));
    float target_theta = PVector.angleBetween(orient, to_target);
    // cross product to get direction 
    PVector orient_3 = new PVector(orient.x, orient.y, 0);
    PVector target_3 = new PVector(to_target.x, to_target.y, 0);
    target_3 = PVector.div(target_3, target_3.mag());
    PVector cross_prod = orient_3.cross(target_3);
    if (cross_prod.z < 0) {
      target_theta = -1*target_theta;
    }
    float phi = 1.0;
    if (r > r_thresh) {
      phi = 1.0 - ((0.9*(r - r_thresh))/(50.0 - 10.0));
    }
    if (r >= 50) {
      phi = 0.1;
    }
    float del_theta = target_theta*phi;
    del_theta = max(del_theta, -1*del_theta_max);
    del_theta = min(del_theta, del_theta_max);
    diff_drive_command = int(map(del_theta, -1*del_theta_max, del_theta_max, 1, 201));
    //print(degrees(target_theta));
    //print(", ");
    //println(degrees(del_theta));
    //println(diff_drive_command);
  }
}
