#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class SeparateJointStates {

public:
  explicit SeparateJointStates(ros::NodeHandle nh) : nh_(nh) {
    // pub subs
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &SeparateJointStates::joint_statesCB, this);
    arm_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states/arm", 10);
    gripper_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states/gripper", 10);

    arm_js_.name.resize(6);
    arm_js_.position.resize(6);
    arm_js_.velocity.resize(6);
    arm_js_.effort.resize(6);

    gripper_js_.name.resize(2);
    gripper_js_.position.resize(2);
    gripper_js_.velocity.resize(2);
    gripper_js_.effort.resize(2);

    for (std::size_t i = 0; i < arm_joint_names_.size(); i++)
      arm_js_.name[i] = arm_joint_names_[i];
    for (std::size_t i = 0; i < gripper_joint_names_.size(); i++)
      gripper_js_.name[i] = gripper_joint_names_[i];
  }

  void joint_statesCB(const sensor_msgs::JointState::ConstPtr &joint_state) {

    arm_js_.header = joint_state->header;
    gripper_js_.header = joint_state->header;

    for(std::size_t i=0; i<joint_state->name.size(); i++){
      
      int pos_in_arm = -1;
      int pos_in_gripper = -1;
      for(std::size_t j=0; j<arm_js_.name.size(); j++){
        if(joint_state->name[i] == arm_js_.name[j]){
          pos_in_arm = j;
          break;
        }
      }
      for(std::size_t j=0; j<gripper_js_.name.size(); j++){
        if(joint_state->name[i] == gripper_js_.name[j]){
          pos_in_gripper = j;
          break;
        }
      }

      if(pos_in_arm != -1){
        arm_js_.position[pos_in_arm] = joint_state->position[i];
        arm_js_.velocity[pos_in_arm] = joint_state->velocity[i];
        arm_js_.effort[pos_in_arm] = joint_state->effort[i];
      }
      if(pos_in_gripper != -1){
        gripper_js_.position[pos_in_gripper] = joint_state->position[i];
        gripper_js_.velocity[pos_in_gripper] = joint_state->velocity[i];
        gripper_js_.effort[pos_in_gripper] = joint_state->effort[i];
      }
    }

    // publish
    arm_js_pub_.publish(arm_js_);
    gripper_js_pub_.publish(gripper_js_);

  }
  ~SeparateJointStates() {}

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher arm_js_pub_, gripper_js_pub_;
  // define joint states messages
  sensor_msgs::JointState arm_js_, gripper_js_;
  std::vector<std::string> arm_joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                               "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  std::vector<std::string> gripper_joint_names_ = {"finger1_joint", "finger2_joint"};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "separate_joint_states_pub_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  SeparateJointStates handler(nh);
  ros::waitForShutdown();
  return 0;
}