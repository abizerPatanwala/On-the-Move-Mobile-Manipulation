#include <memory>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "planning/action/base_control.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
using std::placeholders::_1;


class RegulatedPP
{
  public:
    double minLookAhead;
    double maxLookAhead;
    double desiredLinVel;
    double curvRadThresh;
    double lookAheadGain;
    double minSpeedReg;
    double goalThresh;
    double maxPoseSearch;
    double slowSpeed;
    double slowDist;
    double goalAngleThresh;
    double goalRotateSpeed;
    double workspaceSpeed;
    std::vector<double> workspaceEndPose;
    bool inWorkspace;
    std::vector<std::vector<double>> globalPath;   
    double goal[3]; 


    //Constructor for initialization of pure pursuit parameters
    RegulatedPP(){};
    RegulatedPP(double minLook, double maxLook, double desVel, double radius, double gain, double regSpeed, double thresh, double maxSearch, double slowSpeed, double slowDist, double goalAngleThresh, double goalRotateSpeed, double workspaceSpeed, std::vector<double> workspaceEndPose, bool inWorkspace)
    {
      minLookAhead = minLook;
      maxLookAhead = maxLook;
      desiredLinVel = desVel;
      curvRadThresh = radius;
      lookAheadGain = gain;
      minSpeedReg = regSpeed; 
      goalThresh = thresh;
      maxPoseSearch = maxSearch;
      this->slowSpeed = slowSpeed;
      this->slowDist = slowDist;
      this->goalAngleThresh = goalAngleThresh;
      this->goalRotateSpeed = goalRotateSpeed;
      this->workspaceSpeed = workspaceSpeed;
      this->workspaceEndPose = workspaceEndPose;
      this->inWorkspace = inWorkspace;
    }

    // Initiates global path
    void initiateGlobalPath(const std_msgs::msg::Float64MultiArray & msg)
    {
      globalPath.clear();
      int numPoints = msg.layout.dim[0].stride;
      for (int i = 0; i < numPoints; i+= 3)
      {
        globalPath.push_back({msg.data[i], msg.data[i+1], msg.data[i+2]});
      }
      goal[0] = globalPath.back()[0];
      goal[1] = globalPath.back()[1];
      goal[2] = globalPath.back()[2];
    }

    // Calculate lookahead distance based on current speed
    double getLookaheadDist(double speed)
    {
      double dist;
      dist = std::abs(speed) * lookAheadGain;
      if (dist < minLookAhead)
        dist = minLookAhead;
      if (dist > maxLookAhead)
        dist = maxLookAhead;
      return dist;
    }

    // Prune the Global Path, transform it to local corrdinate system and return it.
    std::vector<std::vector<double>> pathTransform(double pose[3])
    {
      double baseX = pose[0];
      double baseY = pose[1];
      double baseYaw = pose[2];
      int shortIndex;
      double currDist;
      double minDist = FLT_MAX;  
      std::vector<std::vector<double>> localPath;

      // Prune the global path to remove points already crossed
      for (int i = 0; i < int(globalPath.size()) - 1; i++)
      {
        currDist = std::sqrt(std::pow(globalPath[i][0] - baseX, 2) + std::pow(globalPath[i][1] - baseY, 2));
        if (currDist < minDist)
        { 
          shortIndex = i;
          minDist = currDist;
        }
        if (currDist > maxPoseSearch)
          break;
      }
     
     if (int(globalPath.size())!= 1)
     {
      auto start = globalPath.begin();
      std::vector<std::vector<double>>::iterator end = globalPath.begin() + shortIndex;
      globalPath.erase(start, end);
     }
      
     // Transform the global path into local corrdinate system
     double currX;
     double currY;
     double tempVec[2]; 
     for(int i = 0; i < int(globalPath.size()); i++)
     {
      tempVec[0] = globalPath[i][0] - baseX;
      tempVec[1] = globalPath[i][1] - baseY;      
      currX = tempVec[0]*std::cos(baseYaw) + tempVec[1]*std::sin(baseYaw);
      currY = -tempVec[0]*std::sin(baseYaw) + tempVec[1]*std::cos(baseYaw);   
      localPath.push_back({currX, currY});
     }
     return localPath;
    }

    // Get the lookahead point at current lookahead distance by interpolation
    std::vector<double> getLookAheadPt(double lookaheadDist, std::vector<std::vector<double>> localPath)
    {
      std::vector<double> lookAheadPt;      
      int outsideIndex = -1;
      int insideIndex = -1;
      float insideDist = 0;
      float outsideDist = FLT_MAX;
      float currDist;
      for (int i = 0; i < int(localPath.size()); i++)
      {
        currDist = std::sqrt(std::pow(localPath[i][0], 2) + std::pow(localPath[i][1], 2));
        if ((currDist >= lookaheadDist) && (currDist < outsideDist))
        {
          outsideDist = currDist;
          outsideIndex = i;
        }
        if ((currDist < lookaheadDist) && (currDist > insideDist))
        {
          insideDist = currDist;
          insideIndex = i;
        }
      }
      if ((outsideIndex == -1) || (insideIndex == -1))
      {
        lookAheadPt = {localPath.back()[0], localPath.back()[1]};
        return lookAheadPt;
      }
      lookAheadPt = interpolate(insideIndex, outsideIndex, localPath, lookaheadDist);      
      return lookAheadPt;
    }    

    // Returns the the point lying on a circle at lookahead distance by interpolating just outside and just inside point
    std::vector<double> interpolate(int insideIndex, int outsideIndex, std::vector<std::vector<double>> localPath, double lookaheadDist)
    {
      // Calculate the equation of line
      double x1 = localPath[insideIndex][0];
      double y1 = localPath[insideIndex][1];
      double x2 = localPath[outsideIndex][0];
      double y2 = localPath[outsideIndex][1];
      double m;
      double c;
      if (x1 == x2)
      {
        m = FLT_MAX;
        c = x1;
      }
      else
      {
        m = (y1 - y2) / (x1 - x2);
        c = y1 - m*x1;
      }
      
      // Calculate the intersections of line and circle formed by lookahead distance
      double r = lookaheadDist;
      double inter1[2];
      double inter2[2]; 
      if ( m!= FLT_MAX)
      {
        inter1[0] = (-m*c + std::sqrt(std::pow(r,2)*(1 + std::pow(m,2)) - std::pow(c,2))) / (1 + std::pow(m,2));
        inter1[1] = m*inter1[0] + c;
        
        inter2[0] = (-m*c - std::sqrt(std::pow(r,2)*(1 + std::pow(m,2)) - std::pow(c,2))) / (1 + std::pow(m,2));
        inter2[1] = m*inter2[0] + c;        
      }
      else
      {
        inter1[0] = c;
        inter1[1] = std::sqrt(std::pow(r,2) - std::pow(c,2));

        inter2[0] = c;
        inter2[1] = -std::sqrt(std::pow(r,2) - std::pow(c,2));
      }
    

      //Get the intersection point which is between the given line segment
      std::vector<double> lookAheadPt;
      double dist1 = std::sqrt(std::pow(inter1[0] - x1, 2) + std::pow(inter1[1] - y1, 2)) + std::sqrt(std::pow(inter1[0] - x2, 2) + std::pow(inter1[1] - y2, 2));
      double dist2 = std::sqrt(std::pow(inter2[0] - x1, 2) + std::pow(inter2[1] - y1, 2)) + std::sqrt(std::pow(inter2[0] - x2, 2) + std::pow(inter2[1] - y2, 2));
      if (dist1 < dist2)
      {
        lookAheadPt = {inter1[0], inter1[1]};
      }
      else
      {
        lookAheadPt = {inter2[0], inter2[1]};
      }
      return lookAheadPt;
    }

    // Get remaining integrated distance on the path
    double getRemainingDist(std::vector<std::vector<double>> localPath)
    {
      double remainDist = 0;
      for (int i = 1; i < localPath.size(); i++)
      {
        remainDist+= std::sqrt(std::pow(localPath[i][0] - localPath[i-1][0], 2) + std::pow(localPath[i][1] - localPath[i-1][1], 2));
      }
      return remainDist;
    }
    
    // Calculate radius of curvature based on lookahead point
    double getRadiusOfCurv(std::vector<double> lookAheadPt, double lookAheadDist)
    {
        return std::pow(lookAheadDist,2) / (2 * lookAheadPt[1]);
    }

    // Regulate velocity based on radius of curvature and other factors
    double regulateVel(double radiusOfCurv, double pose[3], std::vector<std::vector<double>> localPath)
    {
      double vel;
      // Slow down the robot when in the workspace
      if((inWorkspace == true) && (std::sqrt(std::pow(pose[0] - workspaceEndPose[0], 2) + std::pow(pose[1] - workspaceEndPose[1], 2)) >= 0.01))
      {
        if (workspaceSpeed >= desiredLinVel)
          vel = desiredLinVel;
        else
          vel = workspaceSpeed;
        return vel;
      }  
      else
        inWorkspace = false;
        
        
      // Slow down the robot few distance before we reach goal region
      double distRemain = getRemainingDist(localPath); 
      if ( distRemain <= slowDist)
      {
        vel = slowSpeed;
        return vel;
      }
     
      // Regulate velocity by curvature
      if (std::abs(radiusOfCurv) > curvRadThresh)
        vel = desiredLinVel;
      else
        vel = desiredLinVel * (std::abs(radiusOfCurv) / curvRadThresh);
      if (vel < minSpeedReg)
        vel = minSpeedReg;
        
      return vel;
    }

    // Main function which runs Regulated Pure Pursuit
    std::vector<double> run(double pose[3], double currSpeed)
    {
      std::vector<std::vector<double>> localPath= pathTransform(pose);      
      double lookAheadDist = getLookaheadDist(currSpeed);
      std::vector<double>lookAheadPt = getLookAheadPt(lookAheadDist, localPath);
      double radiusOfCurv = getRadiusOfCurv(lookAheadPt, lookAheadDist);
      double nextSpeed = regulateVel(radiusOfCurv, pose, localPath);
      double nextOmega = nextSpeed / radiusOfCurv;
      std::vector<double> twist = {nextSpeed, nextOmega};
      return twist;
    }
    
    // Returns true if the robot is within goal region
    bool isNearGoal(double pose[3])
    {
      
      if (std::sqrt((std::pow(pose[0] - goal[0],2) + std::pow(pose[1] - goal[1],2))) <= goalThresh)
      {
        return true;
      }
      else
        return false;
    }

};


class localPlanner : public rclcpp::Node
{
  public:
    using BaseControl = planning::action::BaseControl;
     
    localPlanner()
    : Node("local_planner")
    {
      cb_grp_sub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto subscription_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
      subscription_options.callback_group = cb_grp_sub1;
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odometry", 10, std::bind(&localPlanner::topic_callback, this, _1), subscription_options);
      
      using namespace std::placeholders;
    server = rclcpp_action::create_server<BaseControl>(this, "move_base", std::bind(&localPlanner::handle_goal, this, _1, _2), std::bind(&localPlanner::handle_cancel, this, _1), std::bind(&localPlanner::handle_accepted, this, _1));
  
      twistPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    }

  private:
    void topic_callback(const nav_msgs::msg::Odometry & msg)
    {
      //Get current pose of the robot
      float x = msg.pose.pose.position.x;
      float y = msg.pose.pose.position.y;
      tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      pose[0] = x;
      pose[1] = y;
      pose[2] = yaw;
      currSpeed = msg.twist.twist.linear.x;   
      currOmega = msg.twist.twist.angular.z; 
    }
    
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BaseControl::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<BaseControl>> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<BaseControl>> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&localPlanner::server_callback, this, _1), goal_handle}.detach();
    }
    void server_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<BaseControl>> goal_handle)
    {
      const auto goal = goal_handle->get_goal();
      
      // Initialise Pure Pursuit Controller
       RegulatedPP planner;
      double minLookAhead = 0.1;
      double maxLookAhead = 0.1;
      double desLinVel = 0.4;
      double curvRadThresh = 0.9;
      double lookAheadGain = 1.5;
      double minSpeedReg = 0.1;
      double goalAngleThresh = (1.0/180.0) * M_PI;
      double goalRotateSpeed = (15.0/180.0) * M_PI;
      double maxPoseSearch = 3.0;
      double slowSpeed = 0.2;
      double slowDist = 0.6;
      planner.minLookAhead = minLookAhead;
      planner.maxLookAhead = maxLookAhead;
      planner.desiredLinVel = desLinVel;
      planner.curvRadThresh = curvRadThresh;
      planner.lookAheadGain = lookAheadGain;
      planner.minSpeedReg = minSpeedReg;
      planner.goalThresh = goal->goal_threshold;
      planner.initiateGlobalPath(goal->path);
      planner.maxPoseSearch = maxPoseSearch;
      planner.slowSpeed = slowSpeed;
      planner.slowDist = slowDist;
      planner.goalAngleThresh = goalAngleThresh;
      planner.goalRotateSpeed = goalRotateSpeed;
      planner.workspaceSpeed = goal->workspace_speed;
      planner.workspaceEndPose = goal->workspace_end_pose;
      planner.inWorkspace = goal->in_workspace;
      std::cout << "Pure Pursuit Initialized " << std::endl;
      
      auto twistMsg = geometry_msgs::msg::Twist();
      auto feedback = std::make_shared<BaseControl::Feedback>();
      auto result = std::make_shared<BaseControl::Result>();
      std::vector<double> twist;
      std::vector<double> poseVec;
      std::cout << "Running Pure Pursuit Controller " << std::endl;
      // Check if the robot is within the goal threshold. If within the goal then set linear and anguar velocity to zero else run Pure Pursuit Controller       
      while(true)
      {
        // Run Regulated Pure Pursuit Controller
        if (planner.isNearGoal(pose))
        {
          break;
        }
        twist = planner.run(pose, currSpeed);
        twistMsg.linear.x = twist[0];
        twistMsg.angular.z = twist[1];
        twistPublisher->publish(twistMsg);
        poseVec.push_back(pose[0]);
        poseVec.push_back(pose[1]);
        poseVec.push_back(pose[2]);
        feedback->curr_position = poseVec;
        goal_handle->publish_feedback(feedback);
        poseVec.clear();
      }
      if (rclcpp::ok())
      {
        twistMsg.linear.x = 0.000;
        twistMsg.angular.z = 0.000;
        double goalAngVel;
        double tempPose = pose[2];
        if (tempPose < 0)
        {
          tempPose = tempPose + 2*M_PI;
        }
        if (((planner.goal[2] - tempPose) < 0) && (std::abs(planner.goal[2] - tempPose) <= M_PI))
          goalAngVel = -planner.goalRotateSpeed;
        if(((planner.goal[2] - tempPose) < 0) && (std::abs(planner.goal[2] - tempPose) > M_PI))
          goalAngVel = planner.goalRotateSpeed;
        if(((planner.goal[2] - tempPose) > 0) && (std::abs(planner.goal[2] - tempPose) <= M_PI))
          goalAngVel = planner.goalRotateSpeed;
        if(((planner.goal[2] - tempPose) > 0) && (std::abs(planner.goal[2] - tempPose) > M_PI))
          goalAngVel = -planner.goalRotateSpeed;
          
        while(std::abs(tempPose - planner.goal[2]) >= planner.goalAngleThresh)
        {
          tempPose = pose[2];
          if (tempPose < 0)
          {
            tempPose = tempPose + 2*M_PI;
          }
          twistMsg.linear.x = 0.000;
          twistMsg.angular.z = goalAngVel;
          twistPublisher->publish(twistMsg);
          std::this_thread::sleep_for(std::chrono::milliseconds(17));
        } 
        
        twistMsg.linear.x = 0.000;
        twistMsg.angular.z = 0.000;
        twistPublisher->publish(twistMsg);
        
        result->reached_dest = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }

    }
    
    rclcpp::CallbackGroup::SharedPtr cb_grp_sub1;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp_action::Server<BaseControl>::SharedPtr server;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher;
   
    double pose[3];
    double currSpeed;
    double currOmega;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localPlanner>());
  rclcpp::shutdown();
  return 0;
}


