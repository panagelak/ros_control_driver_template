//#define USE_USBCON // Include this for Due before ROS
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// adafruit driver
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

int pos0 = 150;   // position 0
int pos180 = 600; // position 180

ros::NodeHandle nh;
// Receive the servo command from ros topic
void servo_cb(const std_msgs::Int32MultiArray &cmd_msg) {
    for (uint8_t pos = 0; pos < 16; pos++) {
      if (!pwm_cmd.data[pos] == -1) {
        uint16_t cmd = (uint16_t)pwm_cmd.data[pos];
        servos.setPWM(pos, 0, cmd);
      }
    }
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("/command", &servo_cb);

void setup() {
  nh.getHardware()->setBaud(115200);
  servos.begin();
  servos.setOscillatorFrequency(27000000);
  servos.setPWMFreq(SERVO_FREQ);
  delay(10);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(20);
}