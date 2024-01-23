#include <memory>
#include <bits/stdc++.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "planning/srv/pick_point.hpp"
#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim.hpp"
#include "gnuplot_i.hpp"
#include "Ur5e.cpp"
#define OPTIM_PI 3.14159265358979
using std::placeholders::_1;

void wait_for_key ()
{
  std::cout << std::endl << "Press ENTER to continue..." << std::endl; 
  std::cin.clear();
  std::cin.ignore(std::cin.rdbuf()->in_avail());
  std::cin.get();
  return;
}

class BaseFunctions
{
  public:
    double baseWidth = 0.36;
    double baseLength = 0.37;
    double halfDiagonal = std::sqrt(std::pow(baseWidth, 2) + std::pow(baseLength, 2)) / 2.0;
    double alpha = std::atan((baseWidth/2.0) / (baseLength / 2.0)); 
    double stepSize = 0.1;
    std::vector<double> angPrimitive;
    double resolution = 0.1;
    
    BaseFunctions()
    {
      angPrimitive.push_back(0.0);
      for (int i = 0; i < 6; i++)
      {
        angPrimitive.push_back(((i+1)*7*M_PI) / 180.0);
      }
      for (int i = 0; i < 6; i++)
      {
        angPrimitive.push_back(-1*((i+1)*7*M_PI) / 180.0);
      }
    }
    
    // Returns true if the mobile base is in collision
    bool collisionCheck(double x, double y, OBB obj, double distClearance, std::vector<double> thetaAndEdge)
    {
      double objCenter[2] = {(obj.currVertices(0,0) + obj.currVertices(2,0)) / 2.0, (obj.currVertices(0,1) + obj.currVertices(2,1)) / 2.0};
      std::vector<double> Edge0 = {obj.currVertices(0,0), obj.currVertices(0,1), obj.currVertices(1,0), obj.currVertices(1,1)}; //Edge0
      std::vector<double> Edge1 = {obj.currVertices(1,0), obj.currVertices(1,1), obj.currVertices(2,0), obj.currVertices(2,1)}; //Edge1 
      std::vector<double> Edge2 = {obj.currVertices(2,0), obj.currVertices(2,1), obj.currVertices(3,0), obj.currVertices(3,1)}; //Edge2
      std::vector<double> Edge3 = {obj.currVertices(3,0), obj.currVertices(3,1), obj.currVertices(0,0), obj.currVertices(0,1)}; //Edge3
      
      double edgeThetas[4]; // Edge0, Edge 2, Edge 1, Edge 3
      for (int i = 0; i < 2; i++)
      {
        edgeThetas[2*i + 0] = std::atan((obj.currVertices(i,1) - obj.currVertices(i+1,1)) / (obj.currVertices(i,0) - obj.currVertices(i+1,0)));
        edgeThetas[2*i + 1] = edgeThetas[2*i + 0] + M_PI;
        if (edgeThetas[2*i + 0] < 0)
          edgeThetas[2*i + 0] = edgeThetas[2*i + 0] + 2*M_PI;
      }
      
      double currOrient = thetaAndEdge[0];
      std::vector<double> cp1 = {x+ halfDiagonal*cos(currOrient+alpha), y + halfDiagonal*sin(currOrient+alpha)};
      std::vector<double> cp2 = {x+ halfDiagonal*cos(currOrient-alpha), y + halfDiagonal*sin(currOrient-alpha)};
      std::vector<double> cp3 = {x+ halfDiagonal*cos(-M_PI + currOrient - alpha), y + halfDiagonal*sin(-M_PI + currOrient - alpha)};
      std::vector<double> cp4 = {x+ halfDiagonal*cos(-M_PI + currOrient + alpha), y + halfDiagonal*sin(-M_PI + currOrient + alpha)};
      std::vector<std::vector<double>> cornerPts = {cp1, cp2, cp3, cp4};
      
      if ((thetaAndEdge[0] == -10) && (thetaAndEdge[1] == -10))
      {
        return true;
      }
      
      if (thetaAndEdge[1] == 0)
      {
        std::vector<std::vector<double>> edges = {Edge0, Edge2};
        double dist = getClosestEdgedist(cornerPts, edges);
        if (dist < distClearance)
          return true;
        else
          return false;
      } 

      if (thetaAndEdge[1] == 1)
      {
        std::vector<std::vector<double>> edges = {Edge1, Edge3};
        double dist = getClosestEdgedist(cornerPts, edges);
        if (dist < distClearance)
          return true;
        else
          return false;
      } 
      
      if (thetaAndEdge[1] == -1)
      {
        double lineArr0[8] = {obj.currVertices(0,0), obj.currVertices(0,1), obj.currVertices(1,0), obj.currVertices(1,1), obj.currVertices(2,0), obj.currVertices(2,1), obj.currVertices(3,0), obj.currVertices(3,1)}; //Edge0 and Edge2
        double lineArr1[8] = {obj.currVertices(1,0), obj.currVertices(1,1), obj.currVertices(2,0), obj.currVertices(2,1), obj.currVertices(3,0), obj.currVertices(3,1), obj.currVertices(0,0), obj.currVertices(0,1)}; //Edge1 and Edge3
        std::vector<std::vector<double>> edges = {Edge0, Edge1, Edge2, Edge3};
        for (int i = 0; i < 4; i++)
        {
          bool pos1 = ptinBetween(lineArr0, cornerPts[i][0], cornerPts[i][1], objCenter);
          bool pos2 = ptinBetween(lineArr1, cornerPts[i][0], cornerPts[i][1], objCenter);
          bool res = pos1 && pos2;
          if (pos1 == false && pos2 == false)
            continue;
          else
            return true;
        }
        double dist = getClosestEdgedist(cornerPts, edges);
        if (dist < distClearance)
          return true;
        else 
          return false;
      } 
    }
    
    double getClosestEdgedist(std::vector<std::vector<double>> cornerPts, std::vector<std::vector<double>> edges)
    {
      double minDist = FLT_MAX;
      double currDist;
      double x1;
      double y1;
      double x2;
      double y2;
      double m;
      double c;
      for (int i = 0; i < int(edges.size()); i++)
      {
        x1 = edges[i][0];
        y1 = edges[i][1];
        x2 = edges[i][2];
        y2 = edges[i][3];
        if (x1!=x2)
        {
          m = (y2 - y1) / (x2 - x1);
          c = y1 - m*x1;
        }
        else
        {
          m = FLT_MAX;
        }
        for (int j = 0; j < cornerPts.size(); j++)
        {
          if (m!= FLT_MAX)
            currDist = std::abs(m*cornerPts[j][0] - cornerPts[j][1] + c) / (std::sqrt(1 + m*m));
          else 
            currDist = std::abs(x1 - cornerPts[j][0]);
          if (currDist < minDist)
            minDist = currDist;
        }
      }
      return minDist;
    }
    
    
    std::vector<double> getThetaAndEdge(double x, double y, OBB obj, std::vector<double> goal)
    {
      std::vector<double> thetaAndEdge = {0, 0};
      bool pos[2]; // If between line then true else false;
      double objCenter[2] = {(obj.currVertices(0,0) + obj.currVertices(2,0)) / 2.0, (obj.currVertices(0,1) + obj.currVertices(2,1)) / 2.0};
      
      double lineArr0[8] = {obj.currVertices(0,0), obj.currVertices(0,1), obj.currVertices(1,0), obj.currVertices(1,1), obj.currVertices(2,0), obj.currVertices(2,1), obj.currVertices(3,0), obj.currVertices(3,1)}; //Edge0 and Edge2
      
      double lineArr1[8] = {obj.currVertices(1,0), obj.currVertices(1,1), obj.currVertices(2,0), obj.currVertices(2,1), obj.currVertices(3,0), obj.currVertices(3,1), obj.currVertices(0,0), obj.currVertices(0,1)}; //Edge1 and Edge3
      double goalVec[2] = {goal[0] - x, goal[1] - y};

      pos[0] = ptinBetween(lineArr0, x, y, objCenter);
      pos[1] = ptinBetween(lineArr1, x, y, objCenter);
      
      double edgeThetas[4];
      for (int i = 0; i < 2; i++)
      {
        edgeThetas[2*i + 0] = std::atan((obj.currVertices(i,1) - obj.currVertices(i+1,1)) / (obj.currVertices(i,0) - obj.currVertices(i+1,0)));
        edgeThetas[2*i + 1] = edgeThetas[2*i + 0] + M_PI;
        if (edgeThetas[2*i + 0] < 0)
          edgeThetas[2*i + 0] = edgeThetas[2*i + 0] + 2*M_PI;
      }
              
      if ((pos[0] == true) && (pos[1] == false))
      {
        double angle1 = getAngleBetVec(goalVec, edgeThetas[2]);
        double angle2 = getAngleBetVec(goalVec, edgeThetas[3]);
        if (angle1 <= angle2)
          thetaAndEdge[0] = edgeThetas[2];
        else
          thetaAndEdge[0] = edgeThetas[3];
        thetaAndEdge[1] = 1;
      }
      
      if ((pos[1] == true) && (pos[0] == false))
      {
        double angle1 = getAngleBetVec(goalVec, edgeThetas[0]);
        double angle2 = getAngleBetVec(goalVec, edgeThetas[1]);
        if (angle1 <= angle2)
          thetaAndEdge[0] = edgeThetas[0];
        else
          thetaAndEdge[0] = edgeThetas[1];
        thetaAndEdge[1] = 0;
      }
       
      if ((pos[1] == false) && (pos[0] == false))
      {
        double angleMin = 10;
        double angle;
        int edgeIndex = 0; 
        for (int i = 0; i < 4; i++)
        {
          angle = getAngleBetVec(goalVec, edgeThetas[i]);
          if (angle < angleMin)
          {
            angleMin = angle;
            edgeIndex = i;
          }
        }
        thetaAndEdge[0] = edgeThetas[edgeIndex];
        thetaAndEdge[1] = -1;
      }
      
      if ((pos[1] == true) && (pos[0] == true))
      {
        thetaAndEdge[0] = -10;
        thetaAndEdge[1] = -10;
      }
      return thetaAndEdge;
    }
    
    double getAngleBetVec(double vec1[2], double vecTheta)
    {
      double vec2[2] = {std::cos(vecTheta), std::sin(vecTheta)};
      double vec1Mod = std::sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1]);
      double angle = std::acos((vec1[0]*vec2[0] + vec1[1]*vec2[1]) / vec1Mod);
      return angle;
    }
    bool ptinBetween(double lineArr[8], double x, double y, double objCenter[2])
    {
      bool pos[2]; // Same side as object center, then 1 else 0;
      for (int i = 0; i <2; i++)
      {
        double x1 = lineArr[4*i];
        double y1 = lineArr[4*i + 1];
        double x2 = lineArr[4*i + 2];
        double y2 = lineArr[4*i + 3];
        double m;
        double c;
        if (x1!=x2)
        {
          m = (y2 - y1) / (x2 - x1);
          c = y1 - m*x1;
          if (((m*objCenter[0] - objCenter[1] + c) * (m*x - y + c)) >= 0)
            pos[i] = true;
          else
            pos[i] = false;
        }
        else
        {
          if (((objCenter[0] - x1)*(x - x1)) >=0)
            pos[i] = true;
          else
            pos[i] = false;
        }
      }
      
      if (pos[0] && pos[1])
        return true;
      else 
        return false;
    }
    
};

class OptimData
{
  public:
    Ur5e ur;
    OBB obj;
    BaseFunctions baseFunc;
    Eigen::Matrix4d targetFrame;
    std::vector<double> start;
    std::vector<double> goal;
    double manipRadius;
    double distClearance;
    OptimData(OBB obj, Eigen::Matrix4d targetFrame, std::vector<double> start, std::vector<double> goal, double manipRadius, double distClearance)
    {
      this->obj = obj;
      this->start = start;
      this->goal = goal;
      this->manipRadius = manipRadius;
      this->targetFrame = targetFrame;
      this->targetFrame(2,3)+= ur.eeToWrist3 + ur.eeToObject; 
      this->distClearance = distClearance;
    }
    
    std::vector<double> distInReg(const Eigen::Vector2d& mobileBasePose)
    {
      double centerx = targetFrame(0,3);
      double centery = targetFrame(1,3);
      double r = manipRadius;
      double x2 = mobileBasePose[0];
      double y2 = mobileBasePose[1]; 
      double SnG[4] =  {start[0], start[1], goal[0], goal[1]};
      std::vector<double> totalDist;
      for (int i = 0; i < 3; i = i + 2)
      {
        double x1 = SnG[i];
        double y1 = SnG[i+1];
        double m;
        double xinter1;
        double yinter1;
        double xinter2;
        double yinter2;
        if ((x2 - x1) != 0)
        {
          m = (y2 - y1) / (x2 - x1);
          double c = y1 - m*x1;
          double a = m*r;
          double b = r;
          double k = m*centerx - centery + c;
          double ctheta1 = (-a*k + b*std::sqrt(std::pow(a,2) + std::pow(b,2) - std::pow(k,2))) / (std::pow(a,2) + std::pow(b,2));
          double ctheta2 = (-a*k - b*std::sqrt(std::pow(a,2) + std::pow(b,2) - std::pow(k,2))) / (std::pow(a,2) + std::pow(b,2));
          xinter1 = centerx + r*ctheta1;
          yinter1 = m*xinter1 + c;
          xinter2 = centerx + r*ctheta2;
          yinter2 = m*xinter2 + c; 
        }
        else
        {
          double c = x1;
          xinter1 = c;
          yinter1 = centery + std::sqrt(std::pow(r,2) - std::pow(c- centerx, 2)); 
          xinter2 = c;
          yinter2 = centery - std::sqrt(std::pow(r,2) - std::pow(c- centerx, 2)); 
        }
        double dist1 = std::pow(x1 - xinter1,2) + std::pow(y1 - yinter1,2);
        double dist2 = std::pow(x1 - xinter2,2) + std::pow(y1 - yinter2,2);
        if (dist1 < dist2)
          totalDist.push_back(std::sqrt(std::pow(x2 - xinter1,2) + std::pow(y2 - yinter1,2)));
        else 
          totalDist.push_back(std::sqrt(std::pow(x2 - xinter2,2) + std::pow(y2 - yinter2,2))); 
      }
      return totalDist;
    }
    
    void visualize(Gnuplot& g1)
    {
      Eigen::Matrix4d ur5eToOdom {{0, -1 , 0, 0.41 }, {1, 0 , 0, -0.13}, {0, 0, 1, 0.485}, {0, 0, 0, 1}}; 
      Eigen::Matrix4d targetWUr5e = (ur5eToOdom.inverse()) * targetFrame;
      std::vector<double> startPrev = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0};
      std::vector<std::vector<double>> res = ur.IK(targetWUr5e, startPrev);
      ur.transformUr5eBB(res[0],ur5eToOdom);
      ur.visualizeUr5eBB(g1);
      
    } 
};

 double costFunc(const Eigen::VectorXd& mobileBasePose, Eigen::VectorXd* grad_out, void* dataPtr)
  {
    OptimData data = *static_cast<OptimData *>(dataPtr);
    std::vector<double> thetaAndEdge = data.baseFunc.getThetaAndEdge(mobileBasePose(0), mobileBasePose(1), data.obj, data.goal);
    double theta = thetaAndEdge[0];
    Eigen::Matrix4d ur5eToOdom {{std::cos(theta), -std::sin(theta) , 0, mobileBasePose(0)}, {std::sin(theta), std::cos(theta), 0, mobileBasePose(1)}, {0, 0, 1, 0.485}, {0, 0, 0, 1}}; 
    
    //Initialize cost to 0
    double cost = 0;
    
    // Calculte minimum of Eigenvalues of the jacobian at the current configuration and add to the cost.
    Eigen::Matrix4d targetWUr5e = (ur5eToOdom.inverse()) * data.targetFrame;
    std::vector<double> startPrev = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0};
    std::vector<std::vector<double>> ikRes = data.ur.IK(targetWUr5e, startPrev);
    std::vector<double> targetAngles = ikRes[0];
    int ikStatus = std::round(ikRes[1][0]);
    if (ikStatus == 0.0)
    {
      Eigen::Matrix<double, 6, 6> jacobian = data.ur.getJacobian(targetAngles);
      Eigen::Matrix<double, 6, 6> A = jacobian * jacobian.transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(A);
      auto eigenValues = eigensolver.eigenvalues();  
      cost = cost - eigenValues.minCoeff();
    }
    else
      cost = cost + 10;
          
    // Calculate cost incurred due to the arm collision
    data.ur.transformUr5eBB(targetAngles, ur5eToOdom);
    bool collision = data.ur.checkCollision(data.obj);
    if (collision)
      cost = cost + 10;

    // Calculate cost corresponding to spending time in manipulation region
    std::vector<double> totalDist = data.distInReg(mobileBasePose);
    cost = cost + totalDist[0] - totalDist[1];


    //Calculate cost corresponding to distance clearance from the table. The robot should be able to rotate in place. Update the orientation of the robot accordingly
    if(data.baseFunc.collisionCheck(mobileBasePose(0), mobileBasePose(1), data.obj, data.distClearance, thetaAndEdge))
      cost = cost + 10;
    return cost;
  }
  
  //Saves the cost of the workspace in a csv file
  void save_cost(float manipRadius, void* dataPtr)
  {
   double radiusStep = 0.01;
   double thetaStep = (1.0 * M_PI) / 180.0;
   OptimData data = *static_cast<OptimData *>(dataPtr);
   double targetX = data.targetFrame(0,3);
   double targetY = data.targetFrame(1,3);
   std::vector<double> xVec;
   std::vector<double> yVec;
   std::vector<double> costVec;
   
   int radiusCt = std::round(manipRadius / radiusStep);
   int thetaCt = std::round(2*M_PI / thetaStep);
   Eigen::VectorXd basePose = Eigen::VectorXd::Zero(2);
   double cost;
   for (int r = 1; r <= radiusCt; r++)
   {
    double currTheta = 0;
    for (int t = 0; t < thetaCt-1; t++)
    {
      currTheta = t*thetaStep;
      basePose(0) = targetX + r*radiusStep*std::cos(currTheta);
      basePose(1) = targetY + r*radiusStep*std::sin(currTheta);
      cost = costFunc(basePose, nullptr, dataPtr);
      xVec.push_back(basePose(0));
      yVec.push_back(basePose(1));
      costVec.push_back(cost);
    }
   }
   const char* filePath = "/home/abizer10/MobileUR5e/cost.csv";
   std::ofstream outputFile(filePath,  std::ios::out | std::ios::trunc);
   for (size_t i = 0; i < xVec.size(); ++i) 
   {
    outputFile << xVec[i] << "," << yVec[i] << "," << costVec[i] << "\n";
   }
   outputFile.close();
  }
  
  
class optimizer : public rclcpp::Node
{
  public:
    optimizer()
    : Node("optimizer")
    {
      server =  this->create_service<planning::srv::PickPoint>("perform_optimization", std::bind(&optimizer::server_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void server_callback(const std::shared_ptr<planning::srv::PickPoint::Request> request,
          std::shared_ptr<planning::srv::PickPoint::Response> response)
    {
      std::cout << "Received the request for optimization" << std::endl;
      
      Ur5e ur;
      OBB table;
      double l = 0.46; double b = 0.46; double h = 0.46;
      Eigen::Matrix<double, 8, 3> vertices {{-l/2.0, -b/2.0, 0.0}, {l/2.0, -b/2.0, 0.0}, {l/2.0, b/2.0, 0.0}, {-l/2.0, b/2.0, 0.0}, {-l/2.0, -b/2.0, -h}, {l/2.0, -b/2.0, -h}, {l/2.0, b/2.0, -h}, {-l/2.0, b/2.0, -h}};
      table.init(vertices, l, b, h); 
      Eigen::Matrix4d tableFrame {{1, 0, 0, 0.74}, {0, 1, 0, 0.0}, {0, 0, 1, 0.518}, {0, 0, 0, 1}};
      table.transformBox(tableFrame);
      
      Eigen::Matrix4d targetFrame {{-1, 0, 0, 0.74}, {0, 1, 0, -0.15}, {0, 0, -1, 0.558}, {0, 0, 0, 1}};
      std::vector<double> start = request->start;
      std::vector<double> goal = request->goal;
      double manipRadius = 0.6;
      double distClearance = 0.10;
      OptimData data(table, targetFrame, start, goal, manipRadius, distClearance);
      void *dataPtr = &data; 
      
      // Visualzie Bounding Boxes of arm and table
      /*Gnuplot g1("lines");
      g1.set_xlabel("x-axis").set_ylabel("y-axis").set_zlabel("z-axis");
      //g1.set_xrange(-1.5,1.5).set_yrange(-1.5,1.5).set_zrange(-1.5,1.5);
      data.visualize(g1);
      table.visualizeBox(g1);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));   
      wait_for_key();
      */
      
      // Save the cost of the workspace
      /*
      Gnuplot g1("points");
      g1.set_xlabel("x-position").set_ylabel("y-position").set_zlabel("cost");
      g1.set_title("Cost associated with different base position in the workspace");
      save_cost(manipRadius, dataPtr);
      */
      
      Eigen::VectorXd x(2);
      x << targetFrame(0,3) - manipRadius, targetFrame(1,3);
      optim::algo_settings_t settings;
      settings.vals_bound = true;
      settings.print_level= 0;
      settings.rel_objfn_change_tol = 1E-02;
      settings.de_settings.check_freq = 25;
      settings.de_settings.n_pop = 20;
      Eigen::Vector2d upper_bounds (targetFrame(0,3) + manipRadius, targetFrame(1,3) + manipRadius);
      Eigen::Vector2d lower_bounds (targetFrame(0,3) - manipRadius, targetFrame(1,3) - manipRadius);
      settings.lower_bounds = lower_bounds;
      settings.upper_bounds = upper_bounds;
      settings.de_settings.initial_lb = lower_bounds;
      settings.de_settings.initial_ub = upper_bounds;
      //settings.de_settings.mutation_method = best;
      auto timeStart = std::chrono::high_resolution_clock::now();
      bool success = optim::de(x, costFunc, dataPtr, settings);
      auto timeStop = std::chrono::high_resolution_clock::now();
      int duration = std::chrono::duration_cast<std::chrono::milliseconds>(timeStop - timeStart).count();    
      if (success)
      {
        std::cout << "Optimization completed in " << duration <<" milliseconds" << std::endl;
        response->status = true;
        response->cost_value = settings.opt_fn_value;
        std::vector<double> temp;
        temp.push_back(x(0));
        temp.push_back(x(1));
        std::vector<double> thetaAndEdge = data.baseFunc.getThetaAndEdge(x(0), x(1), data.obj, goal);
        if(data.baseFunc.collisionCheck(x(0), x(1), data.obj, data.distClearance, thetaAndEdge))
          std::cout << " Base is Colliding " << std::endl;
     
        temp.push_back(thetaAndEdge[0]);
        response->base_pose = temp;
        Eigen::Matrix4d ur5eToOdom {{std::cos(thetaAndEdge[0]), -std::sin(thetaAndEdge[0]) , 0, x(0)}, {std::sin(thetaAndEdge[0]), std::cos(thetaAndEdge[0]), 0, x(1)}, {0, 0, 1, 0.485}, {0, 0, 0, 1}}; 
        Eigen::Matrix4d targetWUr5e = (ur5eToOdom.inverse()) * data.targetFrame;
        std::vector<double> startPrev = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0};
        std::vector<std::vector<double>> ikResult = data.ur.IK(targetWUr5e, startPrev);
        response->arm_pose = ikResult[0];
        double status = ikResult[1][0];
        std::cout << "Status of IK is " << status << std::endl;
        std::cout << " Sent pick up point pose of the base to the client " << std::endl;
      }
      else
        { 
          response->status = false;
          std::cout << "Optimization Failed" << std::endl;
        }
    }
  private:
    rclcpp::Service<planning::srv::PickPoint>::SharedPtr server;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optimizer>());
  rclcpp::shutdown();
  return 0;
}
