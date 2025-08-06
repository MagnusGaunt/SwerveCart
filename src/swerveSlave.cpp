#include <string.h>
#include <CAN.h>
#include <AS5600.h>
#include <arduino.h>

// setup for the motors

constexpr int driveMotorPin {19};
constexpr int driveMotorChannel {0};
constexpr int azimuthMotorPin {18};
constexpr int azimuthMotorChannel {1};
constexpr int frequency {50};
constexpr int resolution{16};
constexpr int minPulse {1000};
constexpr int maxPulse {2000};

// setup for the canbus

constexpr int rx {21};
constexpr int tx {22};

// setup for the AS5600

AS5600 as5600;
constexpr int sda {4};
constexpr int scl {5};

//pid

constexpr double Kp {.01};
constexpr double Ki {1};
double Kd {10000};

template <typename T>
T getNum(){
    char data[sizeof(T)];
    int index{0};
    while (CAN.available()) {
        data[index] = CAN.read();
        ++index;
    }

    T value{};
    memcpy(&value,data,sizeof(T));
    return value;
}

double getDifference(double goal, double actual){
  double difference {goal - actual};
  if (std::abs(difference) <=180){
    return -difference;
  }
  else if (difference > 0){
    return (360.0 - goal) + (actual);
  }
  return -(360 - actual) - goal;
}

double pidPower(double theta, double pos, int millis){
  if(theta == -1){
    return 0;
  }
  static int lastTime {};
  static double lastPos {};
  const double p {Kp * getDifference(theta, pos)};
  const double i {Ki * 0};
  const double d {Kd * -(pos - lastPos) / 1000.0 * (millis - lastTime)};
  lastTime = millis;
  lastPos = pos;
  return std::max(std::min(p+i+d, 1.0), -1.0);
}

void stopModule(double& theta, double& power){
  theta = -1.0;
  power = 0.0;
}

int powerToDuty(double power){
  return power * 1627 + 4904;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  while (!Serial);

  //motors

  ledcSetup(driveMotorChannel, frequency, resolution);
  ledcAttachPin(driveMotorPin, driveMotorChannel);
  ledcWrite(driveMotorChannel, powerToDuty(0));
  ledcSetup(azimuthMotorChannel, frequency, resolution);
  ledcAttachPin(azimuthMotorPin, azimuthMotorChannel);
  ledcWrite(azimuthMotorChannel, powerToDuty(0));

  //encoder stuff
  Wire.begin(sda, scl);

  as5600.begin(2);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  //canbus stuff
  CAN.setPins(rx, tx);

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  //encoder initialization
  delay(1000);
  as5600.setOffset(2056);
  as5600.setRatio(4);
  as5600.zeroCumulativeToOffset();
}

void loop() {
  static double theta {0};
  static double power {0};
  static double pos {as5600.getScaledAngle()};
  static uint32_t lastTime {0}; //used as a delay while displaying data
  static uint32_t time {0};

  //updates encoder value
  // try to parse packet
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    if (CAN.packetRtr()){
      stopModule(theta, power);
    }
    else{
      switch(CAN.packetId()){
      case 0x01:
        theta = getNum<double>();
        break;
      case 0x02:
        power = getNum<double>();
        break;
      case 0x00: //should never happen, rtr should prock first
        stopModule(theta, power);
        break;
      }
    }
  }
  /*
  if (millis() - lastTime >= 1000){
    pos = as5600.getScaledAngle();
    lastTime = millis();
    Serial.print("The wanted theta is ");
    Serial.print(theta);
    Serial.println(" degrees");
    Serial.print("The power is ");
    Serial.print(power);
    Serial.println(" m/s");
    Serial.println();
    Serial.print("the real encoder pos is ");
    Serial.print(as5600.realAngle());
    Serial.println(" degrees");
    Serial.print("the module pos is ");
    Serial.print(as5600.getScaledAngle());
    Serial.println(" degrees");
    Serial.print("writing ");
    Serial.print(pidPower(theta, as5600.getScaledAngle(), millis()));
    Serial.println(" power");
    //Serial.print("The Difference between wanted and actual is ");
    //Serial.print(getDifference(theta, pos));
    //Serial.println(" degrees");
  }*/
  if (Serial.available()){
    Kd = Serial.parseFloat();
    Serial.print("the new Kd value is ");
    Serial.println(Kd);
  }
  ledcWrite(driveMotorChannel, powerToDuty(power));
  ledcWrite(azimuthMotorChannel, powerToDuty(pidPower(theta, as5600.getScaledAngle(), millis())));
}
