import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
import oscP5.*;
import netP5.*;


OscP5 oscP5;
NetAddress myRemoteLocation;

Serial myPort;
String data = "";
float r, i, j, k;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float roll, pitch, yaw;

void setup() {
  size (1200, 800, P3D);
  myPort = new Serial(this, "COM6", 115200); // starts the serial communication
  myPort.bufferUntil('\n');
  
  //oscP5 = new OscP5(this,12000);
  //myRemoteLocation = new NetAddress("127.0.0.1",12000);
  
  r = 1.0;
  i = 0;
  j = 0;
  k = 0;
  
  roll = 0;
  pitch = 0;
  yaw = 0;
}

void draw() {
  quaternion_to_euler(i, j, k, r);
  print(int(roll));
  print(", ");
  print(int(pitch));
  print(", ");
  print(int(yaw));
  print(", ");
  println();
  translate(width/2, height/2, 0);
  background(233);
  textSize(22);
  text("Roll: " + int(roll) + "     Pitch: " + int(pitch), -100, 265);
  
  // Rotate the object
  rotateX(radians(pitch));
  rotateZ(radians(roll));
  rotateY(radians(yaw));
  
  // 3D 0bject
  box (400, 40, 20); // Draw box
  translate(50, 0, 0);
  textSize(30);  
  fill(0, 76, 153);
  box (386, 40, 200); // Draw box
  textSize(25);
  fill(255, 255, 255);
  text("Test", -183, 10, 101);
}


// Read data from the Serial Port
void serialEvent (Serial myPort) { 
  // reads the data from the Serial Port up to the character '\n' and puts it into the String variable "data".
  data = myPort.readStringUntil('\n');
  // if you got any bytes other than the linefeed:
  if (data != null) {
    data = trim(data);
    
    // split the string at "/"
    //println(data);
    String items[] = split(data, "/");
    if (items.length > 1) {
      //--- Quaternion values
      r = float(items[0]);
      i = float(items[1]);
      j = float(items[2]);
      k = float(items[3]);
      //roll  = float(items[0]);
      //pitch = float(items[1]);
      //yaw   = float(items[2]);

    }
  }
}

void oscEvent(OscMessage theOscMessage) {
  /* print the address pattern and the typetag of the received OscMessage */
  print("### received an osc message.");
  
  
  int firstValue = theOscMessage.get(0).intValue();

  println(firstValue);
}

void quaternion_to_euler(float x, float y, float z, float w)
{
    float t0 = 2.0 * (w * x + y * z);
    float t1 = 1.0 - 2.0 * (x * x + y * y);
    roll = degrees(atan2(t0, t1));

    float t2 = 2.0 * (w * y - z * x);
    
    t2 = t2 > 1 ? 1.0 : t2;
    t2 = t2 < -1 ? -1.0 : t2;
    
    pitch = degrees(asin(t2));

    float t3 = 2.0 * (w * z + x * y);
    float t4 = 1.0 - 2.0 * (y * y + z * z);
    yaw = degrees(atan2(t3, t4));
    
    
    //yaw   = degrees(atan2(2.0f * (x * y + w * z), w * w + x * x - y * y - z * z));
    //pitch = degrees(-asin(2.0f * (x * z - w * y)));
    //roll = degrees(atan2(2.0f * (w * x + y * z), w * w - x * x - y * y + z * z));
}
