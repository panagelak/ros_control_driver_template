#include <ros/ros.h>
#include <csignal>
#include <generic_hardware_interface/generic_hw_control_loop.h>
#include <generic_hardware_interface/generic_hw_interface.h>

// hardware interface pointer
boost::shared_ptr<generic_hardware_interface::GenericHardwareInterface> hw_interface_ptr;
// control loop pointer
boost::shared_ptr<generic_hardware_control_loop::GenericHWControlLoop> hw_control_loop_ptr;

// Interrupt signal
void signalHandler(int signum) {

  ROS_WARN_STREAM("[generic_hw_interface] Interrupt signal (" << signum << ") received.\n");

  hw_interface_ptr.reset();
  hw_control_loop_ptr.reset();

  exit(signum);
}

int main(int argc, char **argv) {
  // Initialize node
  ros::init(argc, argv, "generic_hardware_interface_node");
  // Async spinner
  // So that callbacks, services dont affect the control thread
  ros::AsyncSpinner spinner(2);
  // start spinner
  spinner.start();
  // node handle
  ros::NodeHandle nh("");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  // Create the hardware interface
  hw_interface_ptr.reset(new generic_hardware_interface::GenericHardwareInterface(nh));
  if (!hw_interface_ptr->init()) {
    ROS_ERROR_STREAM("[generic_hw_interface_node] Could not correctly initialize robot. Exiting");
    exit(1);
  }

  ROS_INFO_STREAM("[generic_hw_interface_node] HW interface initialized");
  // Pass the base hardware interface into the control loop object
  // Creates the controller manager
  // Calls read() -> update() -> write() with monotonic time
  hw_control_loop_ptr.reset(new generic_hardware_control_loop::GenericHWControlLoop(nh, hw_interface_ptr));
  // Start the control loop
  hw_control_loop_ptr->run(); // Blocks until shutdown signal received

  return 0;
}
