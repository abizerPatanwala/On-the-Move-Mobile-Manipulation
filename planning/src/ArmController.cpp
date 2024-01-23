#include <memory>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp" 
#include "planning/action/arm_control.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Quintic
{
  public:
    double coeffic[6][6];
    double goal[6];
    double init[6];
    double finalTime;

    // Constructor for Initialization
    Quintic(){};
    Quintic(double start[], double end[], double time)
    {
      for (int i = 0; i < 6; i++)
      {
        init[i] = start[i];
        goal[i] = end[i];
      }
      finalTime = time;
      for (int i = 0; i < 6; i++)
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
    std::vector<std::vector<double>> getCurrDesStates(double t)
    {
      std::vector<std::vector<double>> currDesState;
      
      std::vector<double> temp;
      for (int j = 0; j < 6; j++)
      {
        temp.push_back(coeffic[j][0] + coeffic[j][1]*t + coeffic[j][2]*std::pow(t,2) + coeffic[j][3]*std::pow(t,3) + coeffic[j][4]*std::pow(t,4) + coeffic[j][5]*std::pow(t,5));
      }
      currDesState.push_back(temp);
      temp.clear();
      for (int j = 0; j < 6; j++)
      {
        temp.push_back(coeffic[j][1] + 2*coeffic[j][2]*t + 3*coeffic[j][3]*std::pow(t,2) + 4*coeffic[j][4]*std::pow(t,3) + 5*coeffic[j][5]*std::pow(t,4));
      }
      currDesState.push_back(temp);
      return currDesState;
    }
 
};

class armController : public rclcpp::Node
{
  public:
    using ArmControl = planning::action::ArmControl;
    armController()
    : Node("arm_controller")
    {
    
    // DClient
    cb_grp_clt1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client =  this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller", rmw_qos_profile_services_default, cb_grp_clt1);

    // Publisher and Subscriber
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);
    cb_grp_sub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subscription_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
    subscription_options.callback_group = cb_grp_sub1;
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10 , std::bind(&armController::topic_callback, this, _1), subscription_options);
    
    // Server
    using namespace std::placeholders;
    server = rclcpp_action::create_server<ArmControl>(this, "move_arm", std::bind(&armController::handle_goal, this, _1, _2), std::bind(&armController::handle_cancel, this, _1), std::bind(&armController::handle_accepted, this, _1));
    
    //On shutdown
    rclcpp::on_shutdown(std::bind(&armController::onShutdown, this));
    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg)
    {
       // Get elapsed time since start and current state, and currrent joint pos and vel.
      currPos = msg.position;
      currVel = msg.velocity;
      currTime = msg.header.stamp.sec + (msg.header.stamp.nanosec / std::pow(10, 9)); 
    }
   
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ArmControl::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      if (goal->hold == true)
        RCLCPP_INFO(this->get_logger(), "Holding arm to postion: %f, %f, %f, %f, %f, %f", goal->goal[0], goal->goal[1], goal->goal[2], goal->goal[3], goal->goal[4], goal->goal[5]);
      if (goal->hold == false)
        RCLCPP_INFO(this->get_logger(), "Moving arm to postion: %f, %f, %f, %f, %f, %f", goal->goal[0], goal->goal[1], goal->goal[2], goal->goal[3], goal->goal[4], goal->goal[5]);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmControl>> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmControl>> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&armController::server_callback, this, _1), goal_handle}.detach();
    }
    
    void server_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmControl>> goal_handle)
    {
      const auto goal = goal_handle->get_goal();
      auto result = std::make_shared<ArmControl::Result>();
      
      // Switch to velocity controller
      if (switchController == true)
      {
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = {"arm_velocity_controller"};
        request->deactivate_controllers = {"arm_position_controller"};  
        request->strictness = 2;
        while (!client->wait_for_service(1s))
        {
          if (!rclcpp::ok()) 
          {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto future = client->async_send_request(request);
        future.wait();
        switchController = false;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switch successfull");
      }
      
      
      auto commandMsg = std_msgs::msg::Float64MultiArray();
      commandMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      commandMsg.layout.dim[0].size = 6;
      commandMsg.layout.dim[0].stride = 6;
      std::vector<double> tempVec(6, 0);
      if (goal->hold == false)
      {
        // Initializing quintic class object with start pos, goal pos, and final time
        initTime = currTime;
        for (int i = 0; i < 6; i++)
        {
          trajGen.init[i] = currPos[i];
          trajGen.goal[i] = goal->goal[i];
        }

        trajGen.finalTime = goal->time;
        for (int i = 0; i < 6; i++)
        {
          trajGen.coeffic[i][0] = trajGen.init[i] ;
          trajGen.coeffic[i][1] = 0;
          trajGen.coeffic[i][2] = 0;
          trajGen.coeffic[i][3] = 10*(trajGen.goal[i] - trajGen.init[i]) / std::pow(trajGen.finalTime,3);
          trajGen.coeffic[i][4] = -15*(trajGen.goal[i] - trajGen.init[i]) / std::pow(trajGen.finalTime,4);
          trajGen.coeffic[i][5] = 6*(trajGen.goal[i] - trajGen.init[i]) / std::pow(trajGen.finalTime,5);
        }
          

        // Publish the desired velocity to the arm
        double elapsedTime = currTime - initTime;    
        
        // When elapsed time < final time publish desired velocity from trajectory generated
        while (elapsedTime < trajGen.finalTime)
        { 
          if (goal_handle->is_canceling())
          {
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
          }
          elapsedTime = currTime - initTime;  
          std::vector<std::vector<double>> currDesState = trajGen.getCurrDesStates(elapsedTime);
          for (int i = 0; i < 6; i++)
          {
            tempVec[i] = -posGains[i]*(currPos[i]- currDesState[0][i]);
          }
          commandMsg.data = tempVec;
          publisher_->publish(commandMsg);
        }
      }
      
      // When elapsed time >= final time publish desired velocity to be zero
      if (goal->hold == true)
      {
        while(true)
        {
          if (goal_handle->is_canceling())
          {
            RCLCPP_INFO(this->get_logger(), "Not holding arm to current position");
            return;
          } 
          for (int i = 0; i < 6; i++)
          {
            tempVec[i] = -posGains[i]*(currPos[i]- goal->goal[i]);
          }
          commandMsg.data = tempVec;
          publisher_->publish(commandMsg);
        }
      }
      
      if (rclcpp::ok())
      {
        result->status = true;
        result->last_position = currPos;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Arm moved to position: %f, %f, %f, %f, %f, %f", currPos[0], currPos[1], currPos[2], currPos[3], currPos[4], currPos[5]);
      }

            
    }

    void onShutdown() 
    {
      RCLCPP_INFO(get_logger(), "Ctrl+C pressed. Shutting down the node");
      rclcpp::shutdown();
    }

    rclcpp::CallbackGroup::SharedPtr cb_grp_clt1;
    rclcpp::CallbackGroup::SharedPtr cb_grp_sub1;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;    
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client;
    rclcpp_action::Server<ArmControl>::SharedPtr server;
    Quintic trajGen;
    bool switchController = true; 
    double initTime;
    double posGains[6] = {10, 10, 10, 10, 10, 10};
    std::vector<double> currPos;
    std::vector<double> currVel;
    double currTime;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<armController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  return 0;
}
