#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Definition
class mapJointStateHandler {
public:
  // constructor
  mapJointStateHandler(ros::NodeHandle &nh);
  // map
  float map(float inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange,
            bool reverse_direction);
  // hard limit
  void limit_to_range(float &inValue, float minRange, float maxRange);
  // callback
  void jsCB(const sensor_msgs::JointState::ConstPtr &joint_state);

private:
  // parameters
  ros::NodeHandle nh_;
  ros::Subscriber js_sub_;
  ros::Publisher command_pub_;
  // configuration!

  // ros lower joint limits
  std::vector<float> joint_lower_ = {-1.57, -1.57, -0.7535, -1.57, -1.57, -1.57, 0.0};
  // ros upper joint limits
  std::vector<float> joint_upper_ = {1.57, 1.57, 0.7535, 1.57, 1.57, 1.57, 0.02};
  // servo command lower limits
  std::vector<float> servo_lower_ = {0, 0, -45, 0, 0, 0, 0};
  // servo command upper limits
  std::vector<float> servo_upper_ = {180, 180, 45, 180, 180, 180, 180};
  // servo command offsets
  std::vector<float> offsets_ = {0, 0, 0, 0, 0, 0, 0};
  // reverse direction
  std::vector<bool> reverse_direction_ = {false, false, false, false, false, false, false};
  // location on pwm driver board (16 slots)
  std::vector<int> servo_pins_ = {0, 1, 2, 3, 4, 5, 6};
  // joint names in the same order as servo pins
  // joint state topic has names in alphabetical order
  std::vector<std::string> j_names_order_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                             "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
                                             "finger1_joint"};
  // servos 1 and 2 depedented
  bool has_close_loop_ = true;

  // command message
  std_msgs::Int32MultiArray command_msg_;
};

// function code
mapJointStateHandler::mapJointStateHandler(ros::NodeHandle &nh) : nh_(nh) {
  // joint states sub
  js_sub_ = nh_.subscribe("/joint_states", 10, &mapJointStateHandler::jsCB, this);
  command_pub_ = nh.advertise<std_msgs::Int32MultiArray>("/command", 10);
  // command msg init
  command_msg_.data.resize(16, -1);
}

void mapJointStateHandler::limit_to_range(float &inValue, float minRange, float maxRange) {
  if (inValue > maxRange) {
    inValue = maxRange;
  }
  if (inValue < minRange) {
    inValue = minRange;
  }
}

float mapJointStateHandler::map(float inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange,
                                bool reverse_direction) {
  float x = (inValue - minInRange) / (maxInRange - minInRange);
  float result = minOutRange + (maxOutRange - minOutRange) * x;
  if (reverse_direction)
    return maxOutRange - result;
  return result;
}

void mapJointStateHandler::jsCB(const sensor_msgs::JointState::ConstPtr &joint_state) {

  // for every pin
  for (size_t i = 0; i < servo_pins_.size(); i++) {
    // find joint name pos in joint state topic
    int pos;
    for (size_t j = 0; j < joint_state->name.size(); j++) {
      if (j_names_order_[i] == joint_state->name[j]) {
        pos = j;
        break;
      }
    }
    // get ros pos
    float j_pos = joint_state->position[pos];
    // map it
    float output =
        this->map(j_pos, joint_lower_[i], joint_upper_[i], servo_lower_[i], servo_upper_[i], reverse_direction_[i]);
    command_msg_.data[servo_pins_[i]] = (int)output;
  }

  // TRICK FOR Close loop
  
  // if Depedented link
  // many diy robots have a close kinematic loop between the shoulder lift and elbow lift joints
  // in the urdf we assume that they are indepededent so here we need to do a trick
  // if we move servo 1 and servo 2 together the robot will move as if the servos are indepedent
  // servo 1 and servo2 probably are oriented in opposite manners so they will probably need to have
  // opposite directions
  // so we need to locate a helper variable that will move the servo 2 together with the 1
  // e.g servo 1 = 90 and servo 2 = 90 form orthogonal angle -> offset_for_depedented 0
  // then with e.g servo 1 = 110 and servo 2 = 70 will again form orthogonal angle
  // then servo 2 can move if servo 1 = 90 in  the range (50- 130)
  // so the lower and upper servo commands should be -40 , +40
  if (has_close_loop_) {
    // helper : reverse of servo 1 + offset_for_depedented to get a position of servo 2 relative to servo 1
    int helper = int(servo_upper_[1] - command_msg_.data[servo_pins_[1]]) + offsets_[2];
    // if we substitute this helper to the servo 2 the elbow must remain in a constant
    // angle with shoulder while the shoulder moves e.g we made them indepedent

    // servo_command.data[servo_pins[2]] = helper;

    // Notice how on the servo limits we have limits of the servo 2 relative to the position of servo 1

    // Now add to the servo 2 command the helper
    command_msg_.data[servo_pins_[2]] += helper;
  }

  // Publish the message
  command_pub_.publish(command_msg_);


}

// Main Ros node

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_joint_states_to_command");
  ros::NodeHandle nh;
  // instanciate object
  mapJointStateHandler handler(nh);
  ros::spin();
  return 0;
}
