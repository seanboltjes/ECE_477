import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
import java.util.*;


Serial myPort;
String data = "";

int zoom = 10;
final static byte inc = 2;

float x, y, z;
float lastX, lastY, lastZ;


List<Point> points = new ArrayList<Point>();

void setup() {
  myPort = new Serial(this, "COM4", 115200); // starts the serial communication
  myPort.bufferUntil('\n');

  size(1200, 800, P3D);
  fill(204);
  
  textMode(SHAPE);
  textSize(30);
}

void draw()
{
  lights();
  background(0);
  
  HandleZoomAndPerspective();
  DrawAxes();
  
  
  
  if (x != lastX || y != lastY || z != lastZ)
  {
    lastX = x;
    lastY = y;
    lastZ = z;
  
    Point pointy = new Point(x, y, z);
    points.add(pointy);
  }
  
  //DrawAllPoints();
}

void DrawAllPoints()
{
    stroke(255);
    noSmooth();
    strokeCap(ROUND);
    strokeWeight(5);
    
    Point pointy;
    float zoomer = (float)zoom;
    
    for (int i = 0; i < points.size(); i++)
    {
      pointy = points.get(i); 
      point(pointy.x * zoomer, pointy.y * zoomer, pointy.z * zoomer);
    }
}

void HandleZoomAndPerspective()
{
  if (mousePressed)
    if      (mouseButton == LEFT)   zoom += inc;
    else if (mouseButton == RIGHT)  zoom -= inc;
  
  // Change height of the camera with mouseY
  camera(30.0 + mouseX - (zoom / 10), mouseY - (zoom / 10), 220.0 - (zoom / 8), // eyeX, eyeY, eyeZ
         0.0, 0.0, 0.0, // centerX, centerY, centerZ
         0.0, 0.0, -1.0); // upX, upY, upZ
}

void DrawAxes()
{
  text("+X", 20, -10);
  text("-X", -70, -10);
  text("+Y", -40, 50);
  text("-Y", -35, -50);
  
  
  strokeWeight(4);
  
  //X - red (+x-axis)
  stroke(192,0,0);
  line(0,0,0,1000,0,0);
  line(-1000,0,0,0,0,0);
  
  //Y - green (+y-axis)
  stroke(0,192,0);
  line(0,0,0,0,1000,0);
  line(0,-1000,0,0,0,0);
  
  //Z  - blue (+z-axis)
  stroke(0,0,192);
  line(0,0,1000,0,0,0);
  line(0,0,0,0,0,-1000);
  
  strokeWeight(2);
  
  int tickSpacing = zoom;
  //int tickSpacing = 2;
  int tickLength = 12;
  
  int numTicks = zoom != 0 ? 1000 / zoom : 999;
  
  // make ticks on the X axis
  for (int i = 1; i < numTicks; i++)
  {  
    stroke(192,0,0);
    line(i * tickSpacing, 0, 0, i * tickSpacing, tickLength, 0);
    line(-i * tickSpacing, 0, 0, -i * tickSpacing, tickLength, 0);
  }
  
  // make ticks on the Y axis
  for (int i = 1; i < numTicks; i++)
  {  
    stroke(0,192,0);
    line(0, i * tickSpacing, 0, tickLength, i * tickSpacing, 0);
    line(0, -i * tickSpacing, 0, tickLength, -i * tickSpacing, 0);
  }
  
  // make ticks on the Z axis
  for (int i = 1; i < numTicks; i++)
  {  
    stroke(0,0,192);
    line(0, 0, i * tickSpacing, tickLength, 0, i * tickSpacing);
    line(0, 0, -i * tickSpacing, tickLength, 0, -i * tickSpacing);
  }
}




// Read data from the Serial Port
void serialEvent (Serial myPort) { 
  // reads the data from the Serial Port up to the character '\n' and puts it into the String variable "data".
  data = myPort.readStringUntil('\n');
  // if you got any bytes other than the linefeed:
  if (data != null) {
    data = trim(data);
    
    // split the string at "/"
    println(data);
    String items[] = split(data, "/");
    if (items.length > 1) {
      //--- Position Values
      x = float(items[0]);
      y = float(items[1]);
      z = float(items[2]);
    }
    else if (items.length == 1){
      float scrubber = float(items[0]);
       
      if (scrubber == 777.77) {
         points.clear(); 
      }
    }
  }
}













class Point
{
  public float x;
  public float y;
  public float z;
  
  public Point(float x2, float y2, float z2) {
    x = x2;
    y = y2;
    z = z2;
  };
};
