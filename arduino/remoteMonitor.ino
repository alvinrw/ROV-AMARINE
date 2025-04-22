
// Copy this code and use it on the arduino IDE
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Joystick pins (adjust as needed)
const int joyX = A0;  
const int joyY = A1;
const int joyButton = 2;

// Bluetooth setup (replace pin numbers if necessary)
const int BT_RX = 10;  
const int BT_TX = 11; 
SoftwareSerial BTSerial(BT_RX, BT_TX); 

// I2C LCD object (Replace 0x27 if your address differs)
LiquidCrystal_I2C lcd(0x27, 16, 2);  

// States to track menu selection
int selectedOption = 0; 
bool optionChosen = false;

void setup() {
  Serial.begin(9600); 
  BTSerial.begin(9600);  // Initialize Bluetooth 

  // Initialize I2C LCD 
  lcd.init();
  lcd.backlight();

  // Center Joystick 
  pinMode(joyButton, INPUT_PULLUP);
  int centerX = analogRead(joyX);
  int centerY = analogRead(joyY);

  drawUI(); 
}

void loop() {
  if (isConnected){
    handleJoystickInput(centerX, centerY); 

    if (optionChosen) {
      sendOptionToJetson();
      optionChosen = false;
    }
  } else {
    lcd.clear();
    lcd.setCursor(0.0)
    lcd.print(\"ROV-DISCONNECTED")
    isConnected = true;
    drawUI();
  } else {
    isConnected = false;
  }
  
  if (optionChosen) {
    sendOptionToJetson(); 
    optionChosen = false; 
  }
}

void handleJoystickInput(int centerX, int centerY) {
  int xVal = analogRead(joyX) - centerX;
  int yVal = analogRead(joyY) - centerY;

  if (abs(yVal) > 30) { 
    if (yVal > 0) selectedOption = max(0, selectedOption - 1); 
    else selectedOption = min(4, selectedOption + 1); 
    drawUI();
  }

  if (digitalRead(joyButton) == LOW) { // Button pressed
    optionChosen = true; 
  }
}

void sendOptionToJetson() {
  switch (selectedOption) {
    case 0: BTSerial.println("MAIN-A"); break;
    case 1: BTSerial.println("MAIN-B"); break;
    case 2: BTSerial.println("THRUSTER-CHECK"); break;
    case 3: BTSerial.println("GRIPPER-CHECK"); break;
    case 4: BTSerial.println("FORCE-STOP"); break; // Assuming you have 5 options
  }
}

void drawUI() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AMARINE-ROV");
  lcd.setCursor(0, 1);
  lcd.print("Status: Connected"); 

  // Options (adjust spacing as needed)
  lcd.setCursor(0, 3);
  if (selectedOption == 0) lcd.print("> MAIN-A");
  else lcd.print("  MAIN-A");

  lcd.setCursor(0, 4);
  if (selectedOption == 1) lcd.print("> MAIN-B");
  else lcd.print("  MAIN-B");

  lcd.setCursor(0, 5);
  if (selectedOption == 2) lcd.print("> THRUSTER-CHECK");
  else lcd.print("  THRUSTER-CHECK");

  lcd.setCursor(0, 6);
  if (selectedOption == 3) lcd.print("> GRIPPER-CHECK");
  else lcd.print("  GRIPPER-CHECK");

  lcd.setCursor(0, 7);
  if (selectedOption == 4) lcd.print("> FORCE-STOP");
  else lcd.print("  FORCE-STOP");
}