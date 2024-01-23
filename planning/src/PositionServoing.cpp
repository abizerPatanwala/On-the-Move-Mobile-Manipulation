#include <memory>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "planning/action/servoing.hpp"
#include "Ur5e.cpp"
#include <Eigen/Dense>
using std::placeholders::_1;


class Quintic
{
  public:
    double coeffic[3][6];
    double goal[3];
    double init[3];
    double finalTime;
    Eigen::Matrix3d rotation;

    // Constructor for Initialization
    Quintic(){};
    Quintic(double start[], double end[], double time)
    {
      for (int i = 0; i < 3; i++)
      {
        init[i] = start[i];
        goal[i] = end[i];
      }
      finalTime = time;
      for (int i = 0; i < 3; i++)
      {
        coeffic[i][0] = init[i] ;
        coeffic[i][1] = 0;
        coeffic[i][2] = 0;
        coeffic[i][3] = 10*(goal[i] - init[i]) / std::pow(finalTime,3);
        coeffic[i][4] = -15*(goal[i] - init[i]) / std::pow(finalTime,4);
        coeffic[i][5] = 6*(goal[i] - init[i]) / std::pow(finalTime,5);
      }
    }
    
    // Returns current desired states for the manipulator from quintic interpolation
    Eigen::Matrix4d getCurrDesFrame(double t)
    {
      Eigen::Matrix4d currDesFrame = Eigen::Matrix4d::Zero();
      for (int j = 0; j < 3; j++)
      {
        currDesFrame(j,3) = coeffic[j][0] + coeffic[j][1]*t + coeffic[j][2]*std::pow(t,2) + coeffic[j][3]*std::pow(t,3) + coeffic[j][4]*std::pow(t,4) + coeffic[j][5]*std::pow(t,5);
      }
      currDesFrame(3,3) = 1;
      currDesFrame.block(0,0,3,3) = rotation;
      return currDesFrame;
    }
};
    
class positionServoing : public rclcpp::Node
{
  public:
    using Servoing = planning::action::Servoing;
    positionServoing()
    : Node("position_servoing")
    {
    cb_grp_sub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_grp_sub2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
   
    // Joint state Subscriber
    auto subscription_options1 = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
    subscription_options1.callback_group = cb_grp_sub1;
    jointSubscription = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10 , std::bind(&positionServoing::joint_topic_callback, this, _1), subscription_options1);
    
    //base pose subscriber
    auto subscription_options2 = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
    subscription_options2.callback_group = cb_grp_sub2;
    baseSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odometry", 10, std::bind(&positionServoing::base_topic_callback, this, _1), subscription_options2);
    
    // Publisher
    jointPublisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);
    
    // Server
    using namespace std::placeholders;
    servoingServer = rclcpp_action::create_server<Servoing>(this, "reach_object", std::bind(&positionServoing::handle_goal, this, _1, _2), std::bind(&positionServoing::handle_cancel, this, _1), std::bind(&positionServoing::handle_accepted, this, _1));
    
    //On shutdown
    rclcpp::on_shutdown(std::bind(&positionServoing::onShutdown, this));
    }
    
    private:
    void base_topic_callback(const nav_msgs::msg::Odometry & msg)
    {
      //Get current pose of the robot
      float x = msg.pose.pose.position.x;
      float y = msg.pose.pose.position.y;
      tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      basePose[0]= x;
      basePose[1]= y;
      basePose[2]= yaw;
      baseSpeed = msg.twist.twist.linear.x;   
      baseRotSpeed = msg.twist.twist.angular.z;   
    }
    
    void joint_topic_callback(const sensor_msgs::msg::JointState & msg)
    {
       // Get elapsed time since start and current state, and currrent joint pos and vel.
      for (int i = 0; i < 6; i++)
        currArmPose[i] = msg.position[i];
      currTime = msg.header.stamp.sec + (msg.header.stamp.nanosec / std::pow(10, 9)); 
    }
    
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Servoing::Goal> goal)
    {
      if (goal->mode == "approach" && goal->hold == false)
       RCLCPP_INFO(this->get_logger(), "Received goal request to reach_object");
      if (goal->mode == "lift" && goal->hold == false)
        RCLCPP_INFO(this->get_logger(), "Received goal request to lift_object");
      if (goal->hold == true)
        RCLCPP_INFO(this->get_logger(), "Received goal request to hold arm at the grasp pose");
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Servoing>> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Servoing>> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&positionServoing::server_callback, this, _1), goal_handle}.detach();
    }
    
    void server_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Servoing>> goal_handle)
    {
      // Initialization Ur5e object and joint velocity command message
      const auto goal = goal_handle->get_goal();
      auto result = std::make_shared<Servoing::Result>();
      auto commandMsg = std_msgs::msg::Float64MultiArray();
      commandMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      commandMsg.layout.dim[0].size = 6;
      commandMsg.layout.dim[0].stride = 6;
      std::vector<double> jointVelVec(6, 0);
      
      // Store goal time and pre grasp frame
      if (goal->hold == false)
      {
        Eigen::Matrix4d eeGoalFrame = Eigen::Matrix4d::Zero();
        if (goal->mode == "approach")
        {
          
          for (int i = 0; i < 16; i++)
          {
            eeGoalFrame(i/4, i%4) = goal->obj_frame[i];
          }
          eeGoalFrame(2,3)+= goal->ee_to_wrist + goal->ee_offset;
        }
        if (goal->mode == "lift")
        {
          for (int i = 0; i < 16; i++)
          {
            eeGoalFrame(i/4, i%4) = goal->obj_frame[i];
          }
          eeGoalFrame(2,3)+= goal->ee_to_wrist + goal->ee_offset + 0.05;
        }
        trajGen.finalTime = goal->time; 
        
        // Initialize trajGen 
        Eigen::Matrix4d ur5eToOdom {{std::cos(basePose[2]), -std::sin(basePose[2]) , 0, basePose[0]}, {std::sin(basePose[2]), std::cos(basePose[2]), 0, basePose[1]}, {0, 0, 1, 0.485}, {0, 0, 0, 1}}; 
        Eigen::Matrix4d eeToUr5e = ur.getTransformation(8, currArmPose);
        Eigen::Matrix4d eeToOdom = ur5eToOdom*eeToUr5e;
        std::vector<double> init = {eeToOdom(0,3), eeToOdom(1,3), eeToOdom(2,3)};
        std::vector<double> armGoal = {eeGoalFrame(0,3), eeGoalFrame(1,3), eeGoalFrame(2,3)};
        trajGen.rotation = eeGoalFrame.block(0,0,3,3);
        for (int i = 0; i < 3; i++)
        {
          trajGen.init[i] = init[i];
          trajGen.goal[i] = armGoal[i];
        }
        for (int i = 0; i < 3; i++)
        {
          trajGen.coeffic[i][0] = trajGen.init[i] ;
          trajGen.coeffic[i][1] = 0;
          trajGen.coeffic[i][2] = 0;
          trajGen.coeffic[i][3] = 10*(trajGen.goal[i] - trajGen.init[i]) / std::pow(trajGen.finalTime,3);
          trajGen.coeffic[i][4] = -15*(trajGen.goal[i] - trajGen.init[i]) / std::pow(trajGen.finalTime,4);
          trajGen.coeffic[i][5] = 6*(trajGen.goal[i] - trajGen.init[i]) / std::pow(trajGen.finalTime,5);
        }
        
        // Run the controller to approach the object
        initTime = currTime;
        //const char* filePath = "/home/abizer10/MobileUR5e/maxAngVel.csv";
        //std::ofstream outputFile(filePath,  std::ios::out | std::ios::trunc);
        prevArmPose = currArmPose;
        while((currTime - initTime) <= trajGen.finalTime)
        {
          Eigen::Matrix4d ur5eToOdom {{std::cos(basePose[2]), -std::sin(basePose[2]) , 0, basePose[0]}, {std::sin(basePose[2]), std::cos(basePose[2]), 0, basePose[1]}, {0, 0, 1, 0.485}, {0, 0, 0, 1}}; 
          Eigen::Matrix4d desEEFrame = trajGen.getCurrDesFrame(currTime - initTime);
          Eigen::Matrix4d EEUr5e = ur5eToOdom.inverse()*desEEFrame;
          std::vector<std::vector<double>> res = ur.IK(EEUr5e, prevArmPose);
          std::vector<double> desAngles = res[0];
          for (int i = 0; i < 6; i++)
          {
            jointVelVec[i] = -posGains[i]*(currArmPose[i]- desAngles[i]);
          }
          //outputFile << "Commanded joint velocities " << jointVelVec[0] << "," << jointVelVec[1] << "," << jointVelVec[2] << "," << jointVelVec[3] << "," << jointVelVec[4] << "," << jointVelVec[5] << "," << "\n";
          //outputFile << "Desired joint angles " << desAngles[0] << "," << desAngles[1] << "," << desAngles[2] << "," << desAngles[3] << "," << desAngles[4] << "," << desAngles[5] << "," << "\n";
          ///outputFile << "EE pose " << EEUr5e(0,3) << "," << EEUr5e(1,3) << "," << EEUr5e(2,3) << "\n";
          //outputFile << "IK result is: " << res[1][0] << "\n";
          commandMsg.data = jointVelVec;
          jointPublisher->publish(commandMsg);
          holdFrame = desEEFrame;
          prevArmPose = desAngles;
        }
        
        //outputFile.close();
      }
      
      if (goal->hold == true)
      {
        while(true)
        {
          Eigen::Matrix4d desEEFrame = holdFrame;
          if (goal_handle->is_canceling())
          {
            RCLCPP_INFO(this->get_logger(), "Not holding arm to current position");
            return;
          } 
          Eigen::Matrix4d ur5eToOdom {{std::cos(basePose[2]), -std::sin(basePose[2]) , 0, basePose[0]}, {std::sin(basePose[2]), std::cos(basePose[2]), 0, basePose[1]}, {0, 0, 1, 0.485}, {0, 0, 0, 1}};
          Eigen::Matrix4d EEUr5e = ur5eToOdom.inverse()*desEEFrame;
          std::vector<double> desAngles = ur.IK(EEUr5e, prevArmPose)[0];
          for (int i = 0; i < 6; i++)
          {
            jointVelVec[i] = -posGains[i]*(currArmPose[i]- desAngles[i]);
          }
          commandMsg.data = jointVelVec;
          jointPublisher->publish(commandMsg);
        }
      }
      if (rclcpp::ok())
      {
        
        result->reached = true;
        goal_handle->succeed(result);
      }
    }
    
    
    void onShutdown() 
    {
      RCLCPP_INFO(get_logger(), "Ctrl+C pressed. Shutting down the node");
      rclcpp::shutdown();
    }
    rclcpp::CallbackGroup::SharedPtr cb_grp_sub1;
    rclcpp::CallbackGroup::SharedPtr cb_grp_sub2;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr baseSubscription;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jointPublisher;
    rclcpp_action::Server<Servoing>::SharedPtr servoingServer;
    std::vector<double> currArmPose{0,0,0,0,0,0};
    std::vector<double> prevArmPose;
    bool startPrev = true;
    double basePose[3];
    double baseSpeed;
    double baseRotSpeed;
    double initTime;
    double currTime;
    double posGains[6] = {15, 15, 15, 15, 15, 15};
    Quintic trajGen;
    Ur5e ur;
    Eigen::Matrix4d holdFrame;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<positionServoing>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  return 0;
}
