#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Wire.h>
#include <SPI.h>
#include <SimpleFOC.h>

//电机实例
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 12);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

//编码器实例
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);

// 在线电流检测实例
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01f, 50.0f, 39, 36);
InlineCurrentSense current_sense2 = InlineCurrentSense(0.01f, 50.0f, 35, 34);

// commander通信实例
Commander command = Commander(Serial);
void doMotor1(char* cmd) {
  command.motor(&motor1, cmd);
}
void doMotor2(char* cmd) {
  command.motor(&motor2, cmd);
}

//设置报警电压
#define UNDERVOLTAGE_THRES 11.1
void board_init();
float get_vin_Volt();

void setup() {
  Serial.begin(115200);
  board_init();

  // 编码器设置
  I2Cone.begin(19, 18, 400000UL); // AS5600_M0
  I2Ctwo.begin(23, 5, 400000UL); // AS5600_M1

  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);

  //连接motor对象与传感器对象
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // 驱动器设置
  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = get_vin_Volt();
  driver2.init();
  motor2.linkDriver(&driver2);

  // 电流限制
  motor1.current_limit = 1;
  motor2.current_limit = 1;

  // 电压限制
  motor1.voltage_limit = get_vin_Volt();
  motor2.voltage_limit = get_vin_Volt();


  // 电流检测
  current_sense1.init();
  current_sense1.linkDriver(&driver1);     //no link bug修复
  // current_sense1.gain_b *= 1;
  // current_sense1.gain_a *= 1;
  //  current_sense1.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);

  // current sense init and linking
  current_sense2.init();
  current_sense2.linkDriver(&driver2);     //no link bug修复
  // current_sense2.gain_b *= 1;
  // current_sense2.gain_a *= 1;
  //  current_sense2.skip_align = true;
  motor2.linkCurrentSense(&current_sense2);

  // 控制环
  // 其他模式 TorqueControlType::voltage TorqueControlType::dc_current
  motor1.torque_controller = TorqueControlType::foc_current;
  motor1.controller = MotionControlType::angle;
  motor2.torque_controller = TorqueControlType::foc_current;
  motor2.controller = MotionControlType::angle;

  motor1.voltage_sensor_align = 5;
  motor2.voltage_sensor_align = 5;

  // FOC电流控制PID参数
  motor1.PID_current_q.P = 5;
  motor1.PID_current_q.I = 1000;
  motor1.PID_current_d.P = 5;
  motor1.PID_current_d.I = 1000;
  motor1.LPF_current_q.Tf = 0.002;  // 1ms default
  motor1.LPF_current_d.Tf = 0.002;  // 1ms default

  motor2.PID_current_q.P = 5;
  motor2.PID_current_q.I = 1000;
  motor2.PID_current_d.P = 5;
  motor2.PID_current_d.I = 1000;
  motor2.LPF_current_q.Tf = 0.002;  // 1ms default
  motor2.LPF_current_d.Tf = 0.002;  // 1ms default

  // 速度环PID参数
  motor1.PID_velocity.P = 0.021;
  motor1.PID_velocity.I = 0.12;
  motor1.PID_velocity.D = 0;

  motor2.PID_velocity.P = 0.021;
  motor2.PID_velocity.I = 0.12;
  motor2.PID_velocity.D = 0;
  // default voltage_power_supply
  motor1.P_angle.P = 20;
  motor2.P_angle.P = 20;
  // 速度限制
  motor1.velocity_limit = 20;
  motor2.velocity_limit = 20;


  // monitor接口设置
  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // monitor相关设置
  motor1.monitor_downsample = 0;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  motor2.monitor_downsample = 0;
  motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

  //电机初始化
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC();

  motor2.init();
  // align encoder and start FOC
  motor2.initFOC();

  // 初始目标值
  motor1.target = 0.0;
  motor2.target = 0.0;

  // 映射电机到commander
  command.add('A', doMotor1, "motor 1");
  command.add('B', doMotor2, "motor 2");

  Serial.println(F("Double motor sketch ready."));

  _delay(1000);
}


void loop() {
  motor1.loopFOC();
  motor2.loopFOC();

  motor1.move();
  motor2.move();

  command.run();
}

void board_init() {
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  analogReadResolution(12);  //12bit

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES) {
    VIN_Volt = get_vin_Volt();
    delay(100);
    Serial.printf("等待上电,当前电压%.2f\n", VIN_Volt);
  }
  Serial.printf("正在校准电机...当前电压%.2f\n", VIN_Volt);
}

float get_vin_Volt() {
  return analogReadMilliVolts(13) * 8.5 / 1000;
}














#endif