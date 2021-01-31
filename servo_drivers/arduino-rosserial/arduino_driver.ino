#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

#define SERVOMIN 100  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 565  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

ros::NodeHandle nh;

void servo_cb(const std_msgs::Int32MultiArray &cmd_msg) {
  for (int n = 0; n < 16; n++) {
    if (cmd_msg.data[n] != -1) {
      if (cmd_msg.data[n] > SERVOMAX) {
        cmd_msg.data[n] = SERVOMAX;
      }
      if (cmd_msg.data[n] < SERVOMIN) {
        cmd_msg.data[n] = SERVOMIN;
      }
      servos.setPWM(n, 0, cmd_msg.data[n]);
    }
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("/command", &servo_cb);

void setup() {
  nh.getHardware()->setBaud(57600);
  servos.begin();
  servos.setOscillatorFrequency(27000000);
  servos.setPWMFreq(60); // Frecuecia PWM de 60Hz o T=16,66ms
  delay(10);
  nh.initNode();
  nh.subscribe(sub);
  delay(100);
}

void loop() {
  nh.spinOnce();
  delay(20);
}