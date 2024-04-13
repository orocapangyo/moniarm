import processing.serial.*;
PrintWriter output;
Serial COMPort;  // Create object from Serial class

void setup() 
{
  String portName;
  
  if (args != null) {
    portName = args[0];
  } else {
    portName = "/dev/ttyUSB0";
  }
  
  size(400,400);
  noStroke();
  background(0);
  frameRate(5);

  COMPort = new Serial(this, portName, 115200);
  output = createWriter("/home/jetson/ros2_ws/src/moniarm/moniarm_control/moniarm_control/automove.txt");  
  
  textSize(26); 
  textAlign(CENTER);
  text("press p or space to capture",width/2,60);
  text("Ctrl-C to quit",width/2,100);
}

void keyPressed() {
  COMPort.write(key);
  if (key == 0x03) {
    text("Bye Bye...",width/2,200);
    output.flush(); // Writes the remaining data to the file
    output.close(); // Finishes the file
    exit(); // Stops the program
  }
}

void draw() 
{
  if (COMPort.available() > 0) {  // If data is available,
    String read = COMPort.readString();  // read and store it to string read
    output.print(read);
    print(read);
  }
}
