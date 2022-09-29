import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;
String data = "";

char x_lb, x_ub, y_lb, y_ub, z_lb, z_ub;

int x_raw, y_raw, z_raw;
float x, y, z;

void setup() {
  size (1200, 800, P3D);
  myPort = new Serial(this, "COM7", 115200); // starts the serial communication
  myPort.bufferUntil('\n');
}

void draw() {
  
}


// Read data from the Serial Port
void serialEvent (Serial myPort) { 
  // reads the data from the Serial Port up to the character '\n' and puts it into the String variable "data".
  data = myPort.readStringUntil('\n');
  // if you got any bytes other than the linefeed:
  if (data != null) {
    data = trim(data);
    
    //println(data);
    
    String subData = data.substring(24, 42);
    
    println(subData);
    // erase until match i2c-example: 
    // split by ' '
    String items[] = split(subData, " ");
    if (items.length > 1) {
      //--- Quaternion values
      x_ub = char(parseInt(items[0], 16));
      x_lb = char(parseInt(items[1], 16));
      y_ub = char(parseInt(items[2], 16));
      y_ub = char(parseInt(items[3], 16));
      z_ub = char(parseInt(items[4], 16));
      z_ub = char(parseInt(items[5], 16));
      
      x_raw = (x_ub << 8) | x_lb;
      y_raw = (y_ub << 8) | y_lb;
      z_raw = (z_ub << 8) | z_lb;
      
      x = x_raw;
      x *= 0.00390625;
      y = y_raw;
      y *= 0.00390625;
      z = z_raw;
      z *= 0.00390625;

      //println("%f, %f, %f", x, y, z);
    }
  }
}
