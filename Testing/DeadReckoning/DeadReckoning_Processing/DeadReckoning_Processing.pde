import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

//Serial myPort;
String data = "";

void setup() {
  size (500, 500, P3D);
    background(255);
  //myPort = new Serial(this, "COM4", 9600); // starts the serial communication
  //myPort.bufferUntil('\n');
  //drawAxes(300);
  drawAxes(400);
  camera(30.0, mouseY, 220.0, // eyeX, eyeY, eyeZ
         0.0, 0.0, 0.0, // centerX, centerY, centerZ
         0.0, 1.0, 0.0); // upX, upY, upZ
}

void draw() {
  stroke(0);
  translate(200, 200, 0);
  noSmooth();
  strokeCap(ROUND);
  strokeWeight(3);
  point(10, 10, 10);
  
  ////strokeWeight(4);
  //stroke(192);
  //line(10,10,90,90,10,90);
  noStroke();
  box(90);
  stroke(255);
  line(-100, 0, 0, 100, 0, 0);
  line(0, -100, 0, 0, 100, 0);
  line(0, 0, -100, 0, 0, 100);
}

void drawAxes(float size){
  //X  - red
  strokeWeight(4);
  stroke(100,100,100);
  line(10,10,50,50,10,50);
  
  stroke(192,0,0);
  line(0,0,0,size,0,0);
  //Y - green
  stroke(0,192,0);
  line(0,0,0,0,size,0);
  //Z - blue
  stroke(0,0,192);
  line(0,0,0,0,0,size);
}


// Read data from the Serial Port
//void serialEvent (Serial myPort) { 
//  // reads the data from the Serial Port up to the character '\n' and puts it into the String variable "data".
//  data = myPort.readStringUntil('\n');
//  // if you got any bytes other than the linefeed:
//  if (data != null) {
//    data = trim(data);
    
//    // split the string at "/"
//    println(data);
//    String items[] = split(data, "/");
//    if (items.length > 1) {
//      //--- Quaternion values
//      //r = float(items[0]);
//      //i = float(items[1]);
//      //j = float(items[2]);
//      //k = float(items[3]);

//    }
//  }
//}



//void setup() {
//  size(640, 360, P3D);
//  fill(204);
//}

//void draw() {
//  lights();
//  background(0);
  
//  // Change height of the camera with mouseY
//  camera(30.0, mouseY, 220.0, // eyeX, eyeY, eyeZ
//         0.0, 0.0, 0.0, // centerX, centerY, centerZ
//         0.0, 1.0, 0.0); // upX, upY, upZ
  
//  noStroke();
//  box(90);
//  stroke(255);
//  line(-100, 0, 0, 100, 0, 0);
//  line(0, -100, 0, 0, 100, 0);
//  line(0, 0, -100, 0, 0, 100);
//}
