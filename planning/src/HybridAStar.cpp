#include <memory>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "matplotlibcpp.h"
#include "planning/srv/map_path.hpp"
using std::placeholders::_1;
namespace plt = matplotlibcpp;

class Map
{
  public:
    int width;   
    int height;
    double resolution;
    std::vector<double> origin;
    std::vector<std::vector<int>> grid;
    void initGrid()
    {
      std::vector<int> tempRow(width,0);
      std::vector<std::vector<int>> tempGrid(height, tempRow);
      grid = tempGrid;
    }
    void fillGrid(const nav_msgs::msg::OccupancyGrid & msg)
    {
     initGrid();
     int CurrRow;
     int CurrCol;
     for (int i = 0; i < width*height; i++)
      {
       CurrRow = i / width;
       CurrCol = i % width;
       grid[CurrRow][CurrCol] = msg.data[i];
      }
    }
};

class MobileBase
{
  public:
    double baseWidth = 0.46;
    double baseLength = 0.46;
    double halfDiagonal = std::sqrt(std::pow(baseWidth, 2) + std::pow(baseLength, 2)) / 2.0;
    double alpha = std::atan((baseWidth/2.0) / (baseLength / 2.0)); 
    double reverseCost = 5.0;
    double turnCost = 3.0;  
};

class Node
{
  public:
    std::vector<int> discCoord; //discrete coordinates of the robot on the map
    std::vector<std::vector<double>> traj; //continuous trajectory of the robot from the current node to next one
    double g; // cost from start node to current node
    double h; // heuristic cost from current node to goal node
    double f; //Total cost of this node(g+h)
    std::vector<int> parentDiscCoord; //discrete coordinates of the parent  
    bool isCollision; // Node is in collision or not 
};

class hybridAStar
{
  public:
    // Parameters required for Hybrid A star 
    std::vector<double> start; //(x, y, theta)
    std::vector<double> goal;  //(x, y, theta)
    int row;
    int col;
    double resolution;
    double stepSize;
    std::vector<double> mapOrigin;
    std::vector<std::vector<int>> grid;
    std::vector<std::vector<Node>> gridNodes;
    std::vector<std::vector<bool>> closedList;
    std::vector<double> linPrimitive = {1};
    std::vector<double> angPrimitive;
    MobileBase base;
    // Constructor function for initialization of parameters
    hybridAStar(std::vector<double> s, std::vector<double> g, std::vector<std::vector<int>> gr, double res, double step, std::vector<double> angular, std::vector<double> origin)
    {
      start = s;
      goal = g;
      row = gr.size();
      col = gr[0].size();
      grid = gr;
      angPrimitive = angular;
      resolution = res;
      stepSize = step;
      mapOrigin = origin;
    }

    
    //Function for initializing grid nodes
    void initGridNodes()
    {
     
     for (int i = 0; i < row; i++)
     {
      std::vector<Node> tempRow;
      for (int j = 0; j < col; j++)
      { 
        
        Node temp;
        temp.discCoord = {i, j};   
        temp.g = FLT_MAX;
        temp.f = FLT_MAX;
        temp.h = FLT_MAX;  
        if (grid[i][j] == 100)
        {
          temp.isCollision = true;
        }
        else
        {
          temp.isCollision = false;
        }
        tempRow.push_back(temp);
        
      }
      gridNodes.push_back(tempRow);
     }
     
    }

    
    // Function for initializing closed list with False
    void initClosedList()
    {
      std::vector<bool> tempRow(col, false);
      std::vector<std::vector<bool>> tempCLosedList(row, tempRow);
      closedList = tempCLosedList;
    }
    // Function for simulating trajectory from motion primitive
    Node simulate(double linVel, double angVel, double x, double y, double theta)
    {
      double parentX = x;
      double parentY = y; 
      Node temp;
      temp.isCollision = false;
      temp.parentDiscCoord = {std::round(parentY), std::round(parentX)};
      float pathLength = 5.0*sqrt(2.0)/ 4.0;             
      for (int i = 0; i < std::ceil(pathLength/stepSize); i++)
      {
        theta = theta + angVel*stepSize;
        if (theta >= 2*M_PI)
        {
          theta = theta - 2*M_PI;
        }
        if (theta < 0)
        {
          theta = theta + 2*M_PI;
        }        
        x = x + linVel*stepSize*std::cos(theta);
        y = y + linVel*stepSize*std::sin(theta);
        if (!isValid(x, y, theta))
        {
          temp.isCollision = true;
          return temp;
        }
        temp.traj.push_back({x,y,theta});
      }
      temp.discCoord = {std::round(temp.traj.back()[1]), std::round(temp.traj.back()[0])};
      temp.g = pathCost(linVel, angVel, parentX, parentY, pathLength);
      temp.h = std::sqrt(std::pow(temp.traj.back()[0] - goal[0] , 2) + std::pow(temp.traj.back()[1] - goal[1], 2) + 10*std::pow(temp.traj.back()[2] - goal[2], 2));     
      temp.f = temp.g + temp.h; 
      return temp;
    }

    
    // Returns true if the mobile base co-ordinates are within bound and mobile base is not in collision 
    bool isValid(double x, double y, double theta)
    {
      // Checks if the point is within the map      
      if ((x < 0) || (x >= col) || (y < 0) || (y >= row))
      {
        return false;
      }
      // Checks if the mobile base at location (x,y,theta) is in collision
      if (collisionCheck(x, y, theta))
      {
        return false;
      }
      return true;
    }

    
    // Returns true if the mobile base is in collision or out of map
    bool collisionCheck(double x, double y, double theta)
    {
     //Checks if the centre of mobile base is in collision
     
     // Calculate coordinates of 4 corners of the mobile base, based on currrent x, y, theta
     std::vector<double> cp1 = {x+ base.halfDiagonal*cos(theta+base.alpha)/resolution, y + base.halfDiagonal*sin(theta+base.alpha)/resolution};
     std::vector<double> cp2 = {x+ base.halfDiagonal*cos(theta-base.alpha)/resolution, y + base.halfDiagonal*sin(theta-base.alpha)/resolution};
     std::vector<double> cp3 = {x+ base.halfDiagonal*cos(-M_PI + theta - base.alpha)/resolution, y + base.halfDiagonal*sin(-M_PI + theta - base.alpha)/resolution};
     std::vector<double> cp4 = {x+ base.halfDiagonal*cos(-M_PI + theta + base.alpha)/resolution, y + base.halfDiagonal*sin(-M_PI + theta + base.alpha)/resolution};
  
     // Check if the 4 corners are in the map
     if(MapCheck(cp1) || MapCheck(cp2) || MapCheck(cp3) || MapCheck(cp4))
     {
      return true;
     }
     
     //Lines to check for collision (1->2), (2->4), (4->3), (3->1)
     // 1st line check(1->2)
     if (lineCheck(cp1, cp2))
     {
      return true;
     }

     // 2nd line check(2->4)
     if (lineCheck(cp2, cp4))
     {
      return true;
     }
     // 3rd line check(4->3)
     if (lineCheck(cp4, cp3))
     {
      return true;
     }
     
     // 4th line check(3->1)
     if (lineCheck(cp3, cp1))
     {
      return true;
     }  
     
     return false;
    }

    // Returns true if line is in Collision 
    bool lineCheck(std::vector<double> pt1, std::vector<double> pt2)
    {
      double x1 = pt1[0];
      double y1 = pt1[1];
      double x2 = pt2[0];
      double y2 = pt2[1];
      double m;
      if ((x2 - x1) == 0)
      {
        m = FLT_MAX;
      }
      else
      {
        m = (y2 - y1) / (x2 - x1);
      }
      double c = y1 - m*x1;
      float startpt;
      float endpt; 
      int lineRow;
      int lineCol;
      
      // If slope is between 1 and -1, then the line is more horizontal so increment x by 1 and do collision check
      if ((m >= -1) && (m <= 1))
      {
        startpt = std::min(x1,x2);
        endpt = std::max(x1,x2);
        float i = startpt;
        while(int(i) <= int(endpt))
        {
          if (i > endpt)
            i = endpt;
          lineCol = int(i);
          lineRow = int(m*i + c);
          if (gridNodes[lineRow][lineCol].isCollision)
            return true;
          i = i + 1;
        }
        return false;
      }

      // If slope is not between 1 and -1, then the line is more vertical so increment y by 1 and do collision check
      else
      {
        startpt = std::min(y1,y2);
        endpt = std::max(y1,y2);
        float i = startpt;
        while(int(i) <= int(endpt))
        {
          if (i > endpt)
            i = endpt;
          lineRow = int(i);
          if (m == FLT_MAX)
          {  
            lineCol = int(x1);
          }
          else
          {  
            lineCol = int((i - c) / m);
          }
          if (gridNodes[lineRow][lineCol].isCollision)
            return true;
          i = i + 1;
        }
        return false;
      }
    }

    
    // Calculates cost of the path from current node to the neghbour node
    double pathCost(double linVel, double angVel, double parentX, double parentY, double pathLength)
    {
     double cost = 0;
     
     // Cost to come to the current neighbour node via currrent parent node 
     cost = cost + gridNodes[std::round(parentY)][std::round(parentX)].g + pathLength;
     // Aditional cost associated with non holonomic behaviour of the mobile base
     if (linVel < 0)
     {
      cost = cost + base.reverseCost;
     }
     if (angVel != 0)
     {
      cost = cost + base.turnCost*std::abs(angVel);
     }
     return cost;
    }

   
    //Return true if the point is out of the map
    bool MapCheck(std::vector<double> pt)
    {
      if (pt[0] < 0 || pt[1] < 0 || pt[0] >= col || pt[1] >= row)
        return true;
      else
        return false;
    }
    
    
    // Main function which implements Hybrid A star
    bool run()
    {
     //boolean varible for checking if goal has been reached or not
     bool destFound = false;

     //initilaze grid nodes with cost as infinity, discrete coordinates, and check collision 
     initGridNodes();
     
     //Initiliaze open list and grid nodes with source node
     std::set<std::pair<double, std::pair<int, int>>> openList;
     gridNodes[std::round(start[1])][std::round(start[0])].traj.push_back({start[0], start[1], start[2]});
     gridNodes[std::round(start[1])][std::round(start[0])].discCoord = {std::round(start[1]), std::round(start[0])};
     gridNodes[std::round(start[1])][std::round(start[0])].g = 0.0;
     gridNodes[std::round(start[1])][std::round(start[0])].h = std::sqrt(std::pow(start[0] - goal[0] , 2) + std::pow(start[1] - goal[1], 2) + 10*std::pow(start[2] - goal[2], 2));
     gridNodes[std::round(start[1])][std::round(start[0])].f = gridNodes[std::round(start[1])][std::round(start[0])].h; 
     gridNodes[std::round(start[1])][std::round(start[0])].parentDiscCoord = {std::round(start[1]), std::round(start[0])};
     openList.insert(std::make_pair(0.0, std::make_pair(std::round(start[1]), std::round(start[0]))));
     
     //Run initial checks to verify that the start and goal points are valid
     // if start out of range
     if (MapCheck(start))
     {
      std::cout << "Start coordinates are out of bound" << std::endl;
      return destFound;
     }
     // if goal out of range
     if (MapCheck(goal))
     {
      std::cout << "Goal coordinates are out of bound" << std::endl;
      return destFound;
     }
     // if start is in collision 
     if (gridNodes[std::round(start[1])][std::round(start[0])].isCollision == true)
     {
      std::cout << "Start is in collision" << std::endl;
      return destFound;          
     }    
     // if goal is in collision 
     if (gridNodes[std::round(goal[1])][std::round(goal[0])].isCollision == true)
     {
      std::cout << "Goal is in collision" << std::endl;
      return destFound;          
     }
     // if start pose is such that the mobile base is in collision with obstacles.
     if (collisionCheck(start[0], start[1], start[2]))
     {
      std::cout << "Mobile base is colliding with obstacles at the start position" << std::endl;
      return destFound; 
     }
     // if end pose is such that the mobile base is in collision with obstacles.
     if (collisionCheck(goal[0], goal[1], goal[2]))
     {
      std::cout << "Mobile base will collide with obstacles at the goal position" << std::endl;
      return destFound; 
     }
  
     // Initialize boolean closed list with False
     initClosedList();
     
     //Run Hybrid A* until the goal is found or the open list becomes empty
     while(!openList.empty())
     {
     //std::set<std::pair<double, std::pair<int, int>>>::iterator itr;
      //std::cout << "open list size is: " << openList.size() << std::endl;
      //for (itr = openList.begin(); itr != openList.end(); itr++) {
        // std::pair<double, std::pair<int, int>> temp  = *itr;
        //std::cout << temp.second.first << " " << temp.second.second <<std::endl;
   // }
    
      //Pop the node with the least cost from the openList and add it to closed list
      std::pair<double, std::pair<int, int>> currPair  = *openList.begin();
      openList.erase(openList.begin());
      std::vector<int> currNode = {currPair.second.first, currPair.second.second};
      closedList[currNode[0]][currNode[1]] = true;
      
      // Check if the current node is within resolution distance of goal. If goal, then terminate
      if ((currNode[0] == std::round(goal[1])) && (currNode[1] == std::round(goal[0])))
      {
        destFound = true;
        std::cout << "Path to goal Found" << std::endl;  
        return destFound;
      }
     
      // Calculate neighbours of the current node and update their trajectory, cost and parent by iterating through all the velocity primitives    
 
      for (int i = 0; i < int(linPrimitive.size()); i++)
      {
        for(int j = 0; j < int(angPrimitive.size()); j++)
        {
          //Calculate the trajectory and the co-ordinates of neighbour node, by simulating current linear and angular velocity primitive using kinematic model
          double initX = gridNodes[currNode[0]][currNode[1]].traj.back()[0];
          double initY = gridNodes[currNode[0]][currNode[1]].traj.back()[1];
          double initAng = gridNodes[currNode[0]][currNode[1]].traj.back()[2];
          Node currNeigh  = simulate(linPrimitive[i], angPrimitive[j], initX, initY, initAng);
          //std::cout << "Neighbour " << i*angPrimitive.size() + j << " is " << currNeigh.discCoord[0] << " " << currNeigh.discCoord[1] << std::endl;
          if (currNeigh.isCollision == false)
          {
            int currNeighRow = currNeigh.discCoord[0];
            int currNeighCol = currNeigh.discCoord[1]; 
            if (closedList[currNeighRow][currNeighCol] == false)
            {
              if (currNeigh.f < gridNodes[currNeighRow][currNeighCol].f)
              {
                if (gridNodes[currNeighRow][currNeighCol].f != FLT_MAX)
                {
                  openList.erase(openList.find(std::make_pair(gridNodes[currNeighRow][currNeighCol].f, std::make_pair(currNeighRow, currNeighCol))));
                }
                openList.insert(std::make_pair(currNeigh.f, std::make_pair(currNeighRow, currNeighCol)));
                gridNodes[currNeighRow][currNeighCol].discCoord = {currNeighRow, currNeighCol};
                gridNodes[currNeighRow][currNeighCol].traj = currNeigh.traj;
                gridNodes[currNeighRow][currNeighCol].g = currNeigh.g;
                gridNodes[currNeighRow][currNeighCol].h = currNeigh.h;
                gridNodes[currNeighRow][currNeighCol].f = currNeigh.f;
                gridNodes[currNeighRow][currNeighCol].parentDiscCoord = currNeigh.parentDiscCoord;
              }
            }
          }
        }
      }
    }
    return destFound;
  }


  //Returns the path found by Hybrid A Star
  std::vector<std::vector<double>> retrievePath()
  {
    std::vector<std::vector<double>> path;
    Node currNode = gridNodes[std::round(goal[1])][std::round(goal[0])];
    //Store the path from goal to start
    float x;
    float y;
    path.push_back({goal[0]*resolution + mapOrigin[0], goal[1]*resolution + mapOrigin[1], goal[2]});
    while(currNode.discCoord != currNode.parentDiscCoord)
    {
      
      for(int i = int(currNode.traj.size()) - 1; i >= 0; i--)
      {
        x = currNode.traj[i][0]*resolution + mapOrigin[0];
        y = currNode.traj[i][1]*resolution + mapOrigin[1];
        path.push_back({x, y, currNode.traj[i][2]});
      } 
      currNode = gridNodes[currNode.parentDiscCoord[0]][currNode.parentDiscCoord[1]];  
    }
   
    //Reverse the path
    int start = 0;
    int end = int(path.size()) - 1;
    std::vector<double> temp;
    while(start <= end)
    {
      temp = path[end];
      path[end] = path[start];
      path[start] = temp;
      start++;
      end--;
    }
    return path;
  }
 

  // Plot the path
  void plot(std::vector<std::vector<double>> path)
  {
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
    std::vector<double> z;
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
    plt::show(); 
  }
};

class globalPlanner : public rclcpp::Node
{
  public:
    globalPlanner()
    : Node("global_planner")
    {
    server =  this->create_service<planning::srv::MapPath>("find_global_path", std::bind(&globalPlanner::server_callback, this, std::placeholders::_1, std::placeholders::_2));
    rclcpp::on_shutdown(std::bind(&globalPlanner::onShutdown, this));
    }

  private:
    void server_callback(const std::shared_ptr<planning::srv::MapPath::Request> request,
          std::shared_ptr<planning::srv::MapPath::Response> response)
    {
      //Initialize the map
      auto mapMsg = nav_msgs::msg::OccupancyGrid();
      mapMsg = request->map;
      Map map;
      map.width = mapMsg.info.width;
      map.height = mapMsg.info.height;
      map.resolution = mapMsg.info.resolution;
      map.origin = {mapMsg.info.origin.position.x, mapMsg.info.origin.position.y}; 
      map.fillGrid(mapMsg); 
      // Convert start and goal to map coordinate system
      std::vector<double> start = request->start;
      std::vector<double> goal = request->goal;
      start[0] = (start[0] - map.origin[0]) / map.resolution;
      start[1] = (start[1] - map.origin[1]) / map.resolution;
      goal[0] = (goal[0] - map.origin[0]) / map.resolution;
      goal[1] = (goal[1] - map.origin[1]) / map.resolution;
      
      // Initialize Hybrid A* Parameters
      double stepSize = 0.1;
      std::vector<double> angPrimitive;
      angPrimitive.push_back(0.0);
      for (int i = 0; i < 6; i++)
      {
      angPrimitive.push_back(((i+1)*7*M_PI) / 180.0);
      }
      for (int i = 0; i < 6; i++)
      {
      angPrimitive.push_back(-1*((i+1)*7*M_PI) / 180.0);
      }
      hybridAStar planner(start, goal, map.grid, map.resolution, stepSize, angPrimitive, map.origin);
      
      // Run Hybrid A*      
      RCLCPP_INFO(this->get_logger(), "Running hybrid A star");
      auto timeStart = std::chrono::high_resolution_clock::now();
      bool success = planner.run();
      auto timeStop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timeStop - timeStart);
      RCLCPP_INFO(this->get_logger(), "Time taken to calculate the path(in milliseconds): '%ld' ", duration.count());
      
      //If path exists, then retreive the path and publish it to globalPath topic
      if (success)
      {
        std::vector<std::vector<double>> path = planner.retrievePath(); 
        auto message = std_msgs::msg::Float64MultiArray();
        message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message.layout.dim[0].label = "height";
        message.layout.dim[1].label = "width";
        message.layout.dim[0].size = path.size();
        message.layout.dim[1].size = path[0].size();
        message.layout.dim[0].stride = path.size() * path[0].size();
        message.layout.dim[1].stride = path[0].size();
        message.layout.data_offset = 0;
        std::vector<double> tempVec(path.size()*path[0].size(), 0);
        for (int i=0; i<int(path.size()); i++)
        {
          for (int j=0; j<int(path[0].size()); j++)
          { 
            tempVec[i*path[0].size() + j] = path[i][j];
          }
        }
        message.data = tempVec;
        response->path = message;
        response->status = true;
        std::cout << "Path sent to client" << std::endl;
        //planner.plot(path);
      }
      else
      {
        response->status = false;
         auto message = std_msgs::msg::Float64MultiArray();
        response->path = message;
      }
    }
    
    void onShutdown() 
    {
      RCLCPP_INFO(get_logger(), "Ctrl+C pressed. Shutting down the node");
      rclcpp::shutdown();
    }
    rclcpp::Service<planning::srv::MapPath>::SharedPtr server;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<globalPlanner>());
  rclcpp::shutdown();
  return 0;
}
