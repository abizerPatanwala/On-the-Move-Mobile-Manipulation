#include <memory>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Gripper
{
  public:
    double gripTime; // in sec
    double pubFeq; //in hz
    double closeAngle;  
    double stepSize;
    Gripper(){};
    Gripper(double time, double feq, double angle)
    {
      gripTime = time;
      pubFeq = feq;
      closeAngle = angle;
      stepSize = closeAngle / (gripTime*pubFeq);
    }
    double close(double currAngle)
    {
      return currAngle + stepSize;  
    }
    double open(double currAngle)
    {
      return currAngle - stepSize;  
    }  
};

class gripperController : public rclcpp::Node
{
  public:
    gripperController()
    : Node("gripper_controller")
    {
 
    // Publisher and Subscriber
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10 , std::bind(&gripperController::topic_callback, this, _1));
    }

  private:
   
    void topic_callback(const sensor_msgs::msg::JointState & msg)
    {
      //auto start = std::chrono::high_resolution_clock::now();
      // Initizing Gripper Class
      if (init == true)
      {
        gripper.gripTime = 3;
        gripper.pubFeq = 550;
        gripper.closeAngle = 0.7929;
        gripper.stepSize = gripper.closeAngle / (gripper.gripTime*gripper.pubFeq);
        init = false;
      }
      // Publish the velocity to the gripper
      double currAngle = msg.position[6];
      double nextAngle = gripper.open(currAngle);
      auto commandMsg = std_msgs::msg::Float64MultiArray();
      commandMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      commandMsg.layout.dim[0].size = 1;
      commandMsg.layout.dim[0].stride = 1;
      std::vector<double> tempVec(1, 0);  
      if (currAngle <= 0.0)
      {
        tempVec[0] = currAngle;  
      } 
      else
      {
        tempVec[0] = nextAngle;
      }
      commandMsg.data = tempVec;
      publisher_->publish(commandMsg);
      //auto stop = std::chrono::high_resolution_clock::now();
      //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      //std::cout << duration.count() << std::endl;
    }
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    bool init = true;
    Gripper gripper;
};
















int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gripperController>());
  rclcpp::shutdown();
  return 0;
}
