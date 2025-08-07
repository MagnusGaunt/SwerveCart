#include "swerveClass.h"
#include <ps5Controller.h>
#include <string.h>
#include <CAN.h>

#define RX_GPIO_NUM 21
#define TX_GPIO_NUM 22

//Converted controller outputs
// ranges from a value of -1 : 1
#define JOYSTICKMAX 128
#define STICKDRIFT 8

float convertValue(int8_t input, int8_t max, int8_t drift){
  int8_t posOrNeg = -1;
  if (input >= 0){
    input++;
    posOrNeg = 1;
  }
  if(-drift <= input && input <= drift){
    return 0;
  }
  
  max -= drift;
  input -= drift * posOrNeg;
  return float(input) / float(max);
}

float getX(){
  return convertValue(ps5.LStickX(), JOYSTICKMAX, STICKDRIFT);
}

float getY(){
  return convertValue(ps5.LStickY(), JOYSTICKMAX, STICKDRIFT);
}

float getR(){
  return convertValue(ps5.RStickX(), JOYSTICKMAX, STICKDRIFT);
}

template <typename T>
void sendNumber(T input, int PacketId) {
    char data[sizeof(T)];
    memcpy(data,&input,sizeof(T));
    CAN.beginPacket(PacketId);
    for (int i = 0; i < sizeof(T); ++i){
      CAN.write(data[i]);
    }
    CAN.endPacket();
}

//RunnerCode

void setup() {
  Serial.begin(115200);
  while (!Serial){
  } // Optional: Wait for the serial port to be ready

  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // Start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1); // Halt the program if CAN initialization fails
  }

  ps5.begin("AC:36:1B:D1:64:0D"); // Replace with the MAC address of your controller
  Serial.println("Ready.");

  analogReadResolution(12);
}

void loop() {
  while (!ps5.isConnected()) { // commented out as ps5 controller seems to connect quicker when microcontroller is doing nothing
    Serial.println("PS5 controller not found");
    CAN.beginPacket(0x00);
    CAN.endPacket();
    delay(300);
  }

  Swerve swerve{};
  swerve.kinematics(getX(), getY(), getR());
  
  //all sent values are {x, y}
  sendNumber<>(swerve.getModule(0), 0x01);
  sendNumber<>(swerve.getModule(1), 0x02);
  sendNumber<>(swerve.getModule(2), 0x03);
  sendNumber<>(swerve.getModule(3), 0x04);

  delay(200);
}
