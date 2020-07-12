import processing.serial.*;
PrintWriter output;

Serial myport; // Create object from Serial class
String myString = null;
String control = null;

void setup() {
  size(440,440);
  // Create a new file in the sketch directory
  output = createWriter("comm_trial_53_w_vis.txt"); 
  myport = new Serial(this, "/dev/rfcomm0", 9600);
  myport.bufferUntil('\n');
}

void draw() {
  while (myport.available() > 0) {
    myString = myport.readStringUntil('\n');
    if (myString != null) {
      output.print(str(millis()) + " : "); 
      output.println(myString);
    }
  }
  
  if (myString != null) {
    String[] list = split(myString, ":");
    if (list[0].equals("S1") == true) {
      background(125);
      rect(width/2 - 20, height/2 - 20, 40, 40); // this is the car 
      ellipse(width/2 - 10, height/2 - 10, 10, 10);
      ellipse(width/2 + 10, height/2 - 10, 10, 10);
      String[] list_s1 = split(list[1], " cm");
      float my_s1 = float(list_s1[0]);
      String[] list_s2 = split(list[2], " cm");
      float my_s2 = float(list_s2[0]);
      String[] list_s3 = split(list[3], " cm");
      float my_s3 = float(list_s3[0]);
      String[] list_s4 = split(list[4], " cm");
      float my_s4 = float(list_s4[0]);
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
  }
}


void keyPressed() {
  if (key == 'z' || key == 'Z') {
    output.flush(); // Writes the remaining data to the file
    output.close(); // Finishes the file
    exit(); // Stops the program
  }
  if (key == CODED) {
    if (keyCode == UP) {
      myport.write('G');
      control = "F";
    } else if (keyCode == DOWN) {
      myport.write('V');
      control = "B";
    } else if (keyCode == RIGHT) {
      myport.write('W');
      control = "R";
    } else if (keyCode == LEFT) {
      myport.write('O');
      control = "L";
    }
    output.print(str(millis()) + " : ");
    output.println("Control " + control);
  }
}
