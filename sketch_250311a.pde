
import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;
String data="";
float roll, pitch, yaw;

void setup() {
  fullScreen(P3D); // Set up the window size
  myPort = new Serial(this, "COM18", 19200); // Open the serial port, make sure it's correct
  myPort.bufferUntil('\n'); // Buffer data until a newline is encountered
}

void draw() {
  translate(width/2, height/2, 0); // Center the origin in the middle of the window
  background(0); // Set background to black (0 is black, 255 is white)

  // Display Roll and Pitch values
  textSize(22);
  text("Roll: " + int(roll) + "     Pitch: " + int(pitch) + "     Yaw: " + int(yaw), -100, 265);

  // Rotate the 3D object based on Roll, Pitch, and Yaw
  rotateX(radians(-pitch));  // Rotate along X-axis (Pitch)
  rotateZ(radians(-roll));    // Rotate along Z-axis (Roll)
  rotateY(radians(yaw));     // Rotate along Y-axis (Yaw)

  // Draw a 3D box to represent the orientation
  textSize(30);
  fill(0, 76, 153); // Set fill color for the box (blue-ish)
  box(386, 40, 200); // Draw a 3D box (dimensions of the box)
  textSize(25);
  fill(255, 255, 255); // Set fill color for the text (white)
  text("Innovation Lab Group-10", -125, 10, 101);
}

// Read data from the Serial Port
void serialEvent(Serial myPort) { 
  data = myPort.readStringUntil('\n'); // Read the incoming data until newline character
  if (data != null) {
    data = trim(data); // Trim the incoming data to remove any extra whitespace
    // Split the string by the '|' character
    String items[] = split(data, '|');
    if (items.length > 1) {
      // Parse the Roll, Pitch, and Yaw values
      roll = float(items[0].substring(6));  // Extract the roll value (skip "Roll: ")
      pitch = float(items[1].substring(8)); // Extract the pitch value (skip "Pitch: ")
      yaw = float(items[2].substring(6));   // Extract the yaw value (skip "Yaw: ")
    }
  }
}
