#include <ps5Controller.h>
#include <string.h>
#include <CAN.h>

#define RX_GPIO_NUM 21
#define TX_GPIO_NUM 22

#define PI 3.1415926535897932384626433832795
#define ROWS 4
#define COLS 2

//Converted controller outputs
// ranges from a value of -1 : 1
#define JOYSTICKMAX 128
#define STICKDRIFT 8

double convertValue(int8_t input, int8_t max, int8_t drift){
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
  return double(input) / double(max);
}

double getX(){
  return convertValue(ps5.LStickX(), JOYSTICKMAX, STICKDRIFT);
}

double getY(){
  return convertValue(ps5.LStickY(), JOYSTICKMAX, STICKDRIFT);
}

double getR(){
  return convertValue(ps5.RStickX(), JOYSTICKMAX, STICKDRIFT);
}

class Matrix {

private:
  double mat[ROWS][COLS];

public:

  Matrix() {
    for (uint8_t i = 0; i < ROWS; i++) {
      for (uint8_t j = 0; j < COLS; j++) {
        mat[i][j] = 0;
      }
    }
  }

  //allows for easy asigning of values
  Matrix(std::initializer_list<std::initializer_list<double>> list) {
  uint8_t i = 0;
  for (const auto& row : list) {
    uint8_t j = 0;
    for (const auto& val : row) {
      mat[i][j] = val;
      j++;
    }
    i++;
  }
}

  //scales a matrix by a value
  Matrix operator*(const double scale) const{
    Matrix result;
    for (uint8_t i = 0; i < ROWS; i++) {
      for (uint8_t j = 0; j < COLS; j++) {
        result.mat[i][j] = mat[i][j] * scale;
      }
    }
    return result;
  }

  //adds two matricies together
  Matrix operator+(Matrix& other) {
    for (uint8_t i = 0; i < ROWS; i++) {
      for (uint8_t j = 0; j < COLS; j++) {
        mat[i][j] += other.mat[i][j];
      }
    }
    return *this;
  }

  //adds 2 matricies without remaking a class
  Matrix& operator+=(const Matrix& other) {
    for (uint8_t i = 0; i < ROWS; i++) {
      for (uint8_t j = 0; j < COLS; j++) {
        mat[i][j] = mat[i][j] + other.mat[i][j];
      }
    }
    return *this;
  }

  //finds the max vector within a matrix
  double matrixMaxVector() const {
    double maxVector = 0;
    for (uint8_t i = 0; i < ROWS; i++) {
      maxVector = max(maxVector, modulePower(i));
    }
    return maxVector;
  }

  //to square things
  double square(double val) const {
    return val * val;
  }

  //gives the theta of a module given an x-y Vector
  double moduleHeading(uint8_t row) const{
    double x = mat[row][0];
    double y = mat[row][1];
    int8_t quad = getQuadrant(x, y);
    double theta = atan(y/x) * 180 / PI;
    switch (quad) {
      case 1:
        return theta;
        break;
      case 2:
        return 180 + theta;
        break;
      case 3:
        return 180 + theta;
        break;
      case 4:
        return 360 + theta;
        break;
      default:
        return -1;
        break;
    }
  }

  //gets the vector of the module.
  double modulePower(uint8_t row) const{
    double x = mat[row][0];
    double y = mat[row][1];
    return sqrt(square(x) + square(y));
  }

  //gets the quadrant which the vector is directed
  int getQuadrant(double x, double y) const{
    if (x == 0 && y == 0) {
      return -1;
    }
    if (x >= 0) {
      if (y >= 0) {
        return 1;
      }
      return 4;
    }
    if (y < 0) {
      return 3;
    }
    return 2;
  }
};

//swerve constants
//double directionMax[row][col] = {
//  {x, y}, module top left
//  {x, y}, module top right
//  {x, y}, module back right
//  {x, y}  module back left
//}

const Matrix fullY = {
  { 0, 1 },
  { 0, 1 },
  { 0, 1 },
  { 0, 1 }
};

const Matrix fullX = {
  { 1, 0 },
  { 1, 0 },
  { 1, 0 },
  { 1, 0 }
};

const Matrix fullR = {
  { 1, 1 },
  { 1, -1 },
  { -1, -1 },
  { -1, 1 }
};

//outputs all of the module vectors in a matrix format
Matrix kinematics(double x, double y, double r) {
  Matrix result;

  result += fullX * x;
  result += fullY * y;
  result += fullR * r;

  double matrixMaxVector = result.matrixMaxVector();
  if (matrixMaxVector > 1) {
    return result*(1.0 / matrixMaxVector);
  }

  return result;
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
  static Matrix swerveMatrix;
  swerveMatrix = kinematics(getX(), getY(), getR());

  while (!ps5.isConnected()) { // commented out as ps5 controller seems to connect quicker when microcontroller is doing nothing
    Serial.println("PS5 controller not found");
    CAN.beginPacket(0x00);
    CAN.endPacket();
    delay(300);
  }
  
  //module 1
  sendNumber<>(swerveMatrix.moduleHeading(0), 0x01);
  sendNumber<>(swerveMatrix.modulePower(0), 0x02);
  //module 2
  sendNumber<>(swerveMatrix.moduleHeading(1), 0x03);
  sendNumber<>(swerveMatrix.modulePower(1), 0x04);
  //module 3
  sendNumber<>(swerveMatrix.moduleHeading(2), 0x05);
  sendNumber<>(swerveMatrix.modulePower(2), 0x06);
  //module 4
  sendNumber<>(swerveMatrix.moduleHeading(3), 0x07);
  sendNumber<>(swerveMatrix.modulePower(3), 0x08);

  delay(200);
}
