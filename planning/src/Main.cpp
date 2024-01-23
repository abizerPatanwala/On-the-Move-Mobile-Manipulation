#include <memory>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "planning/srv/map_path.hpp"
#include "planning/srv/pick_point.hpp"
#include "planning/action/base_control.hpp"
#include "planning/action/arm_control.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_grasping/action/attacher.hpp"
#include "planning/action/servoing.hpp"
#include "Ur5e.cpp"
#include <mutex>
#include <condition_variable>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class pickPlace : public rclcpp::Node
{
  public:
    using ArmControl =  planning::action::ArmControl; 
    using BaseControl =  planning::action::BaseControl;
    using Attacher = ros2_grasping::action::Attacher; 
    using Servoing = planning::action::Servoing;
    pickPlace()
    : Node("pick_place")
    {
      rclcpp::QoS qos_profile(10);
      qos_profile.transient_local();
      cb_grp_clt1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      cb_grp_clt2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      cb_grp_clt3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      
      mapSubscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos_profile, std::bind(&pickPlace::run, this, std::placeholders::_1));
      
      gPlannerClient = this->create_client<planning::srv::MapPath>("/find_global_path", rmw_qos_profile_services_default, cb_grp_clt1);
      optimizerClient = this->create_client<planning::srv::PickPoint>("/perform_optimization", rmw_qos_profile_services_default, cb_grp_clt1);
      armClient = rclcpp_action::create_client<ArmControl>(this, "/move_arm", cb_grp_clt1);
      baseClient = rclcpp_action::create_client<BaseControl>(this, "/move_base", cb_grp_clt2);
      attachClient = rclcpp_action::create_client<Attacher>(this, "/Attacher", cb_grp_clt3);
      servoingClient = rclcpp_action::create_client<Servoing>(this, "/reach_object", cb_grp_clt1);
      
      gripperPublisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);
      rclcpp::on_shutdown(std::bind(&pickPlace::onShutdown, this));
    }
  private:
    void run(const nav_msgs::msg::OccupancyGrid mapMsg)
    { 
      
      // Define start and goal pose of the base of the robot
      std::vector<double> start = {-3.0, -3.0, 0.0};
      std::vector<double> goal = {2.8, 1.5, 0.0};
      
      // Run optimization to find the pick up point and arm Pose
      auto request1 = std::make_shared<planning::srv::PickPoint::Request>(); 
      request1->map = mapMsg;
      request1->start = start;
      request1->goal = goal;
      auto future1 = optimizerClient->async_send_request(request1);
      std::cout << "Calculating pick up location " << std::endl;
      future1.wait();
      auto result1 = future1.get();
      if (result1->status == true)
        std::cout << "Pick up location calculated " << std::endl;
      else
        std::cout << "Calculation failed" << std::endl;
      std::vector<double> pickPose = {result1->base_pose[0], result1->base_pose[1], result1->base_pose[2]};
      std::vector<double> pickArmPose = {result1->arm_pose[0], result1->arm_pose[1], result1->arm_pose[2], result1->arm_pose[3], result1->arm_pose[4], result1->arm_pose[5]};
      std::cout << "Pick up base pose is: " << pickPose[0] << " " << pickPose[1] << " " << pickPose[2] << std::endl;
      std::cout << "Pick up arm pose is: " << pickArmPose[0] << " " << pickArmPose[1] << " " << pickArmPose[2] << " " << pickArmPose[3] << " " << pickArmPose[4] << " " << pickArmPose[5] << std::endl; 
      std::cout << "Cost is: " <<result1->cost_value<< std::endl;
      
      // Calculate global Path from start to goal passing through pick up pose
      std::vector<std::vector<double>> pathPoints;
      pathPoints.push_back(start);
      pathPoints.push_back(pickPose);
      pathPoints.push_back(goal);
      auto request2 = std::make_shared<planning::srv::MapPath::Request>();
      request2->map = mapMsg;
      std::vector<std::vector<double>> startToGoal;
      std::vector<std::vector<double>> startToPick;
      std::vector<std::vector<double>> pickToGoal;
      for (int i = 0; i < 2; i++)
      {
        request2->start = pathPoints[i];
        request2->goal = pathPoints[i+1];
        auto future2 = gPlannerClient->async_send_request(request2);
        std::cout << "Calculating global Path: " << std::endl;
        future2.wait();
        auto result2 = future2.get();
        if (result2->status == true)
          std::cout << "Global Path calculated: " << std::endl;
        else
          std::cout << "Calculation failed: " << std::endl;
        int numPoints = result2->path.layout.dim[0].stride;
        for (int j = 0; j < numPoints; j+= 3)
        {
          std::vector<double> temp;
          temp.push_back(result2->path.data[j]);
          temp.push_back(result2->path.data[j+1]);
          temp.push_back(result2->path.data[j+2]);
          startToGoal.push_back(temp);
          if (i == 0)
            startToPick.push_back(temp);
          if (i == 1)
            pickToGoal.push_back(temp);
        }
      }
 
      //Save the global Path
      savePath(startToGoal, mapMsg, "path");
      
      double startToGoalDist = getIntegratedDist(startToGoal);
      std::cout << "Integrated Distance of the path is: " << startToGoalDist << std::endl;
      
      // Get the base location at which the arm should move
      armMoveLoc = getArmMoveLoc(startToPick);
      std::cout << " Base location at which arm starts moving: " << armMoveLoc[0] << " " << armMoveLoc[1] << std::endl; 
      
     
      //Command the gripper to open
      commandGripper(openAngle);
       
      //Command the arm to go to home position
      auto armMsg = ArmControl::Goal();
      armMsg.time = armMoveTime;
      armMsg.goal = armHome;
      armMsg.hold = false;
      auto armSendGoalOpt = rclcpp_action::Client<ArmControl>::SendGoalOptions();
      armSendGoalOpt.result_callback = std::bind(&pickPlace::armResultCallback, this, std::placeholders::_1);
      armClient->async_send_goal(armMsg, armSendGoalOpt);
      

      //Store the startToPick path into the message to be used by Local Planner
      auto pathMsg = std_msgs::msg::Float64MultiArray();
      pathMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      pathMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      pathMsg.layout.dim[0].label = "height";
      pathMsg.layout.dim[1].label = "width";
      pathMsg.layout.dim[0].size = startToPick.size();
      pathMsg.layout.dim[1].size = startToPick[0].size();
      pathMsg.layout.dim[0].stride = startToPick.size() * startToPick[0].size();
      pathMsg.layout.dim[1].stride = startToPick[0].size();
      pathMsg.layout.data_offset = 0;
      std::vector<double> tempVec(startToPick.size()*startToPick[0].size(), 0);
      for (int i=0; i<int(startToPick.size()); i++)
      {
        for (int j=0; j<int(startToPick[0].size()); j++)
        { 
          tempVec[i*startToPick[0].size() + j] = startToPick[i][j];
        }
      }
      pathMsg.data = tempVec;
      
      //Store the pickToGoal path into the message to be used by Local Planner 
      auto pathMsg2 = std_msgs::msg::Float64MultiArray();
      pathMsg2.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      pathMsg2.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      pathMsg2.layout.dim[0].label = "height";
      pathMsg2.layout.dim[1].label = "width";
      pathMsg2.layout.dim[0].size = pickToGoal.size();
      pathMsg2.layout.dim[1].size = pickToGoal[0].size();
      pathMsg2.layout.dim[0].stride = pickToGoal.size() * pickToGoal[0].size();
      pathMsg2.layout.dim[1].stride = pickToGoal[0].size();
      pathMsg2.layout.data_offset = 0;
      std::vector<double> tempVec2(pickToGoal.size()*pickToGoal[0].size(), 0);
      for (int i=0; i<int(pickToGoal.size()); i++)
      {
        for (int j=0; j<int(pickToGoal[0].size()); j++)
        { 
          tempVec2[i*pickToGoal[0].size() + j] = pickToGoal[i][j];
        }
      }
      pathMsg2.data = tempVec2;

      
      
      //Wait for arm to move to home position
      while(!baseMove){};
      
      // Run Pure Pursuit Controller to go to pick pose
      std::vector<std::vector<double>> null = {{0}, {0,0}};    
      auto baseMsg = BaseControl::Goal();
      baseMsg.path = pathMsg;
      baseMsg.in_workspace = false;
      baseMsg.workspace_speed = null[0][0];
      baseMsg.workspace_end_pose = null[1];
      baseMsg.goal_threshold = 0.05;
      auto baseSendGoalOpt = rclcpp_action::Client<BaseControl>::SendGoalOptions();
      baseSendGoalOpt.result_callback = std::bind(&pickPlace::baseResultCallback, this, std::placeholders::_1);
      baseSendGoalOpt.feedback_callback = std::bind(&pickPlace::baseFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
      baseClient->async_send_goal(baseMsg, baseSendGoalOpt); 
      std::cout << "Running Pure Pursuit Controller " << std::endl; 
      
      // Command the arm to move at the arm move location
      bool armMoved = false;
      baseMove = false;
      while(!goalReached)
      { 
        if (armMove && !armMoved)
        {
          armClient->async_cancel_all_goals();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          armMsg.time = armMoveTime;
          armMsg.goal = pickArmPose;
          armMsg.hold = false;
          armClient->async_send_goal(armMsg, armSendGoalOpt);
          armMoved = true;
         }
         
      }
      
       // Get the workpace base speed and workspace end pose
      std::vector<std::vector<double>> res =  getCoveragePoseSpeed(pickToGoal);
      std::cout << "workspace speed is: " << res[0][0] << std::endl;
      
      
   
      
      
      // Run Pure Pursuit Controller to go to goalPose
      auto baseMsg2 = BaseControl::Goal();
      baseMsg2.path = pathMsg2;
      baseMsg2.in_workspace = true;
      baseMsg2.goal_threshold = 0.1;
      baseMsg2.workspace_speed = res[0][0];
      baseMsg2.workspace_end_pose = res[1];
      baseClient->async_send_goal(baseMsg2, baseSendGoalOpt); 
      std::cout << "Running Pure Pursuit Controller " << std::endl;   
      
       // Move the arm to grasp pose
      armClient->async_cancel_all_goals();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      auto servoingMsg = Servoing::Goal();
      std::vector<double> objFrameVec(16,0);
      for (int i = 0; i < 16; i++)
        objFrameVec[i] = targetFrame(i/4, i%4);
      servoingMsg.obj_frame = objFrameVec;
      servoingMsg.ee_to_wrist = eeToWrist;
      servoingMsg.ee_offset = eeOffset;
      servoingMsg.time = objectReachTime;
      servoingMsg.mode = "approach";
      servoingMsg.hold = false;
      auto servoingSendGoalOpt = rclcpp_action::Client<Servoing>::SendGoalOptions();
      servoingSendGoalOpt.result_callback = std::bind(&pickPlace::servoingResultCallback, this, std::placeholders::_1);
      servoingClient->async_send_goal(servoingMsg, servoingSendGoalOpt);
      
      
      //Wait until the object is grasped
      while(!reachedGraspPose){};
      commandGripper(closeAngle);
       
       // Attach the object 
      auto attachMsg = Attacher::Goal();
      attachMsg.object = "box";
      attachMsg.endeffector = "ur5e_EEcenter_link";
      auto attachSendGoalOpt = rclcpp_action::Client<Attacher>::SendGoalOptions();
      attachClient->async_send_goal(attachMsg, attachSendGoalOpt); 
      
      
     
      
      
      // Lift the object
      servoingClient->async_cancel_all_goals();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      servoingMsg.obj_frame = objFrameVec;
      servoingMsg.ee_to_wrist = eeToWrist;
      servoingMsg.ee_offset = eeOffset;
      servoingMsg.time = objectReachTime;
      servoingMsg.mode = "lift";
      servoingMsg.hold = false;
      servoingClient->async_send_goal(servoingMsg, servoingSendGoalOpt);
      reachedGraspPose = false;
      while(!reachedGraspPose){};
      
       //Move the arm to home position
      servoingClient->async_cancel_all_goals();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      armMsg.time = armMoveTime;
      armMsg.goal = armHome;
      armMsg.hold = false;
      armClient->async_send_goal(armMsg, armSendGoalOpt);


      // Wait to reach to goal
      goalReached = false;
      while(!goalReached){};
      
      
      
    }  
    
    void armResultCallback(const rclcpp_action::ClientGoalHandle<ArmControl>::WrappedResult & result)  
    {
      switch (result.code) 
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          std::cout << "Arm moved to desired position" << std::endl;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          std::cout << "Stopping arm hold " << std::endl;
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
        
      }
      auto armMsg = ArmControl::Goal();
      armMsg.time = armMoveTime;
      armMsg.goal = result.result->last_position;
      armMsg.hold = true;
      auto armSendGoalOpt = rclcpp_action::Client<ArmControl>::SendGoalOptions();
      armSendGoalOpt.result_callback = std::bind(&pickPlace::armResultCallback, this, std::placeholders::_1);
      armClient->async_send_goal(armMsg, armSendGoalOpt);
      std::cout << "Holding arm to current position" << std::endl;
      baseMove = true;
    }
    
    void servoingResultCallback(const rclcpp_action::ClientGoalHandle<Servoing>::WrappedResult & result) 
    {
      switch (result.code) 
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          std::cout << "Arm moved to grasp pose" << std::endl;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          std::cout << "Goal was cancelled" << std::endl;
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      
      auto servoingMsg = Servoing::Goal();
      servoingMsg.mode = "";
      servoingMsg.hold = true;
      auto servoingSendGoalOpt = rclcpp_action::Client<Servoing>::SendGoalOptions();
      servoingSendGoalOpt.result_callback = std::bind(&pickPlace::servoingResultCallback, this, std::placeholders::_1);
      servoingClient->async_send_goal(servoingMsg, servoingSendGoalOpt);
      
      reachedGraspPose = true;
    }
    void baseFeedbackCallback(rclcpp_action::ClientGoalHandle<BaseControl>::SharedPtr, const std::shared_ptr<const BaseControl::Feedback> feedback)
    { 
      currBasePos.clear();
      currBasePos = feedback->curr_position;
      if(std::sqrt(std::pow(currBasePos[0] - armMoveLoc[0], 2) + std::pow(currBasePos[1] - armMoveLoc[1], 2)) <= 0.05)
      {
        armMove = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(17));
      }
    }
  
    void baseResultCallback(const rclcpp_action::ClientGoalHandle<BaseControl>::WrappedResult & result)  
    {
      switch (result.code) 
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          std::cout << "Base reached successfully to destination" << std::endl;
          goalReached = true;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
    }
    
    std::vector<double> getArmMoveLoc(std::vector<std::vector<double>> path)
    {
     std::vector<double> pickPose = path[path.size() - 1];
     std::vector<double> beforePickPose = path[path.size() - 2];
     std::vector<double> desLoc;
     double angleDiff;
     if (std::abs(pickPose[2] - beforePickPose[2]) <= M_PI)
      angleDiff =  std::abs(pickPose[2] - beforePickPose[2]);
     else
      angleDiff =  2*M_PI - (std::abs(pickPose[2] - beforePickPose[2]));
      
     double rotateTime = angleDiff / goalRotateSpeed;
     double remTimeSlow = armMoveTime - rotateTime;
     if (remTimeSlow <= 0)
     {
      desLoc = pickPose;
      return desLoc;
     }
     double remTimeFast = std::min(0.0, remTimeSlow - slowDist / slowSpeed); 
     double remDistSlow = remTimeSlow*slowSpeed;
     double remDist;
     if (remTimeFast!= 0.0)
      remDist = remTimeFast*desLinVel + remDistSlow;
     else
      remDist = remDistSlow;
     
     double integDist = 0.0;
     for (int i = int(path.size() - 2); i >= 0; i--)
     {
      integDist+= std::sqrt(std::pow(path[i][0] - path[i+1][0] ,2) + std::pow(path[i][1] - path[i+1][1] ,2));
      if (integDist >= remDist)
      {  
        desLoc = path[i];
        break;
      }
     }
     return desLoc;
    }
    
    //Function to open or close the gripper
    void commandGripper(double angle)
    {
      auto commandMsg = std_msgs::msg::Float64MultiArray();
      commandMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      commandMsg.layout.dim[0].size = 1;
      commandMsg.layout.dim[0].stride = 1;
      std::vector<double> tempVec;
      tempVec.push_back(angle);  
      commandMsg.data = tempVec;
      gripperPublisher->publish(commandMsg);
    }
    
    // Function to calculate the postion on the path and the speed base moves at until the object is picked up
    std::vector<std::vector<double>> getCoveragePoseSpeed(std::vector<std::vector<double>> pickToGoal)
    {
      double time = 2*objectReachTime;
      double dist = 0;
      std::vector<double> pose(2,0);
      std::vector<std::vector<double>> res;
      for (int i = 1; i < int(pickToGoal.size()); i++)
      {
        dist+= std::sqrt(std::pow(pickToGoal[i][0] - pickToGoal[i-1][0], 2) + std::pow(pickToGoal[i][1] - pickToGoal[i-1][1], 2));
        if(dist >= manipRadius)
        {
          pose[0] = pickToGoal[i][0];
          pose[1] = pickToGoal[i][1];
          break;
        }
      }
      std::vector<double> speed = {dist / time};
      res.push_back(speed);
      res.push_back(pose);
      return res; 
    }
    
    double getIntegratedDist(std::vector<std::vector<double>> path)
    {
      double dist = 0;
      for (int i = 1; i < int(path.size()); i++)
      {
        dist+= std::sqrt(std::pow(path[i][0] - path[i-1][0], 2) + std::pow(path[i][1] - path[i-1][1], 2));
      }
      return dist;
    }
    // Plot the path
    void savePath(std::vector<std::vector<double>> path, const nav_msgs::msg::OccupancyGrid & mapMsg, std::string s)
    {
      double resolution = mapMsg.info.resolution;
      std::vector<double> mapOrigin = {mapMsg.info.origin.position.x, mapMsg.info.origin.position.y}; 
      
      std::vector<int> tempRow(mapMsg.info.width,0);
      std::vector<std::vector<int>> tempGrid(mapMsg.info.height, tempRow);
      std::vector<std::vector<int>>grid = tempGrid;
      int CurrRow;
      int CurrCol;
      for (int i = 0; i < mapMsg.info.width*mapMsg.info.height; i++)
      {
        CurrRow = i / mapMsg.info.width;
        CurrCol = i % mapMsg.info.width;
        grid[CurrRow][CurrCol] = mapMsg.data[i];
      }
      int row = grid.size();
      int col = grid[0].size();
      
      std::vector<double> x;
      std::vector<double> y;
      for(int i = 0; i < int(path.size()); i++)
      {
        x.push_back(path[i][0]);
        y.push_back(path[i][1]);
      }
      plt::plot(x, y); 
      plt::xlim(-int(col*resolution) / 2, int(col*resolution) / 2);
      plt::ylim(-int(row*resolution) / 2, int(row*resolution) / 2);
      x.clear();
      y.clear();
      for (int i = 0; i < row; i++)
      {
        for(int j = 0; j < col; j++)
        {
          if (grid[i][j] == 100)
          {
            double xcorner = j*resolution + mapOrigin[0];
            double ycorner = i*resolution + mapOrigin[1]; 
            std::vector<double> x_coords = {xcorner, xcorner + resolution, xcorner + resolution, xcorner, xcorner};
            std::vector<double> y_coords = {ycorner, ycorner, ycorner + resolution, ycorner + resolution, ycorner};
            plt::plot(x_coords, y_coords, "k-");
          }
        }
      }
      plt::save(s + ".png");
      plt::close();
    }
  
    void onShutdown() 
    {
      RCLCPP_INFO(get_logger(), "Ctrl+C pressed. Shutting down the node");
      rclcpp::shutdown();
    }
    
    bool baseMove = false;
    bool armMove = false;
    bool goalReached = false;
    bool reachedGraspPose = false;
    std::vector<double> currBasePos;
    std::vector<double> armMoveLoc;
    std::vector<double> armHome = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix4d targetFrame {{-1, 0, 0, 0.74}, {0, 1, 0, -0.15}, {0, 0, -1, 0.558}, {0, 0, 0, 1}};
    double armMoveTime = 5.0;
    double goalRotateSpeed = (15.0/180.0) * M_PI;
    double desLinVel = 0.4;
    double slowSpeed = 0.2;
    double slowDist = 0.6;
    double openAngle = 0.0;
    double closeAngle = 0.4;
    double eeOffset = 0.02;
    double eeToWrist = 0.1603;
    double objectReachTime = 2.0;
    double manipRadius = 0.6;
    
    rclcpp::CallbackGroup::SharedPtr cb_grp_clt1;
    rclcpp::CallbackGroup::SharedPtr cb_grp_clt2;
    rclcpp::CallbackGroup::SharedPtr cb_grp_clt3;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription_;   
    rclcpp::Client<planning::srv::MapPath>::SharedPtr gPlannerClient;
    rclcpp::Client<planning::srv::PickPoint>::SharedPtr optimizerClient;
    rclcpp_action::Client<BaseControl>::SharedPtr baseClient;
    rclcpp_action::Client<ArmControl>::SharedPtr armClient;
    rclcpp_action::Client<Attacher>::SharedPtr attachClient;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripperPublisher;
    rclcpp_action::Client<Servoing>::SharedPtr servoingClient;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<pickPlace>();
  executor.add_node(node);
  executor.spin();
  //rclcpp::spin(node);  
  return 0;
}
