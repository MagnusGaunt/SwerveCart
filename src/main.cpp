#include <ps5Controller.h>
#include <CAN.h>

#define TX_GPIO_NUM   5
#define RX_GPIO_NUM   4

#define PI 3.1415926535897932384626433832795
#define ROWS 4
#define COLS 2

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

class Matrix {

private:
  float mat[ROWS][COLS];

public:

  Matrix() {
    for (uint8_t i = 0; i < ROWS; i++) {
      for (uint8_t j = 0; j < COLS; j++) {
        mat[i][j] = 0;
      }
    }
  }

  //allows for easy asigning of values
  Matrix(std::initializer_list<std::initializer_list<float>> list) {
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
  Matrix operator*(const float scale) const{
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
  float matrixMaxVector() const {
    float maxVector = 0;
    for (uint8_t i = 0; i < ROWS; i++) {
      maxVector = max(maxVector, modulePower(i));
    }
    return maxVector;
  }

  //to square things
  float square(float val) const {
    return val * val;
  }

  //gives the theta of a module given an x-y Vector
  float moduleHeading(uint8_t row) const{
    float x = mat[row][0];
    float y = mat[row][1];
    int8_t quad = getQuadrant(x, y);
    float theta = atan(y/x) * 180 / PI;
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
  float modulePower(uint8_t row) const{
    float x = mat[row][0];
    float y = mat[row][1];
    return sqrt(square(x) + square(y));
  }

  //gets the quadrant which the vector is directed
  int getQuadrant(float x, float y) const{
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
//float directionMax[row][col] = {
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
Matrix kinematics(float x, float y, float r) {
  Matrix result;

  result += fullX * x;
  result += fullY * y;
  result += fullR * r;

  float matrixMaxVector = result.matrixMaxVector();
  if (matrixMaxVector > 1) {
    return result*(1.0 / matrixMaxVector);
  }

  return result;
}

void sendCan(float input, uint8_t id) {
  uint8_t floatByte[sizeof(float)];

  memcpy(floatByte, &input, sizeof(float));

  //Serial.print("sending float value ... ");
  //Serial.print(input);
 // Serial.print(" with id ");
  //Serial.println(id);

  CAN.beginPacket(0x01);
  //CAN.write(id);
  for (size_t j = 0; j < sizeof(float); j++) {
    CAN.write(floatByte[j]);
  }
  CAN.endPacket();
}



//RunnerCode

Matrix swerveMatrix;

void setup() {
  Serial.begin(921600);
  while (!Serial); // Optional: Wait for the serial port to be ready

  Serial.println("CAN Sender");

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

  swerveMatrix = kinematics(getX(), getY(), getR());
  for (uint8_t i = 0; i < 4; i++) {

    float theta = swerveMatrix.moduleHeading(i);
    float power = swerveMatrix.modulePower(i);

    //Serial.print("pwr, heading = ");
    //Serial.print(power);
    //Serial.print(", ");
    //Serial.println(theta);

    sendCan(power, i);
    sendCan(theta, i);
  }
  Serial.println("-----------------");
  delay(200);
}