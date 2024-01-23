#include <memory>
#include <bits/stdc++.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "gnuplot_i.hpp"
#include "OBB.cpp"
#include <Eigen/Dense>
class Ur5e
{
  // Two additional frames are defined for the convenience of creating bounding boxes. So we have now 2 extra joints with their value being zero. 
  // The joint format is {theta1, theta2, 0, 0, theta3, theta4, theta5, theta6} 
  public: 
    double DH[8][4] = {{0, 0.1625, 0, M_PI_2}, {0, 0.1333, 0.0, 0.0}, {0, 0.0, -0.425, 0.0}, {0, -0.1333, 0, 0}, {0, 0, -0.3922, 0}, {0, 0.1333, 0, M_PI_2}, {0, 0.1, 0, -M_PI_2}, {0, 0.1,  0, 0}}; // order is theta,d,r,alpha
    OBB ur5eBB[8];
    KDL::Chain ur5eChain;
    double eeToWrist3 = 0.1603;
    double eeToObject = 0.15;
    Ur5e()
    {
      createBB();
      std::string urdf_path = ament_index_cpp::get_package_share_directory("skywalker");
      urdf_path += "/urdf/ur5e.urdf";
      KDL::Tree ur5_tree;
	    if (!kdl_parser::treeFromFile(urdf_path, ur5_tree))
      {
		    std::cout<<"Failed to generate KDL tree";
	    }
	    ur5_tree.getChain("ur5e_base_link", "ur5e_wrist_3_link", ur5eChain);
    }
  
    std::vector<std::vector<double>> IK(Eigen::Matrix4d targetFrame, std::vector<double> prevAngles)
    {
      std::vector<double> targetAngles;
      Eigen::Matrix<double, 6, 1> L;
      L.setOnes();
      KDL::ChainIkSolverPos_LMA ik_solver(ur5eChain, L, 1e-3, 2000);
     
      KDL::Rotation rotMat(targetFrame(0,0), targetFrame(0,1), targetFrame(0,2), targetFrame(1,0), targetFrame(1,1), targetFrame(1,2), targetFrame(2,0), targetFrame(2,1), targetFrame(2,2));
      KDL::Vector transVec(targetFrame(0,3), targetFrame(1,3), targetFrame(2,3));
      KDL::Frame target_frame(rotMat, transVec);
      
      KDL::JntArray jnt_pos_start(6);
      for (int i = 0; i < int(prevAngles.size()); i++)
      {
        jnt_pos_start(i) = prevAngles[i];
      }
      
      KDL::JntArray jnt_pos_goal(6);
      int status = ik_solver.CartToJnt(jnt_pos_start, target_frame, jnt_pos_goal);
      for (int i = 0; i < 6; i++)
      {
        targetAngles.push_back(jnt_pos_goal(i));
      }
      std::vector<std::vector<double>> res;
      res.push_back(targetAngles);
      std::vector<double> resStatus = {status};
      res.push_back(resStatus);
      return res;
    }
    
    void createBB()
    {
      double BB_dim[8][3] = {{0.0745, 0.0745, 0.1625}, {0.0745, 0.0745, 0.1333}, {0.0745, 0.0745, 0.425}, {0.0745, 0.0745, 0.1333}, {0.0745, 0.0745, 0.3922}, {0.0745, 0.0745, 0.1333}, {0.0745, 0.0745, 0.0997}, {0.0745, 0.0745, 0.0996}};
      
      Eigen::Matrix<double, 8, 3> temp;
      
      double l = BB_dim[0][0]; double b = BB_dim[0][1]; double h = BB_dim[0][2]; 
      temp << -l/2.0, 0.0 , -b/2.0, l/2.0, 0.0 , -b/2.0, l/2.0, 0.0 , b/2.0, -l/2.0, 0.0 , b/2.0, -l/2.0, -h, -b/2.0 , l/2.0, -h , -b/2.0, l/2.0, -h, b/2.0, -l/2.0, -h, b/2.0;
      ur5eBB[0].init(temp, l, b, h);

       l = BB_dim[1][0];  b = BB_dim[1][1];  h = BB_dim[1][2];
       temp << -l/2.0, -b/2.0, 0.0, l/2.0, -b/2.0, 0.0, l/2.0, b/2.0, 0.0, -l/2.0, b/2.0, 0.0, -l/2.0, -b/2.0, -h, l/2.0, -b/2.0, -h, l/2.0, b/2.0, -h, -l/2.0, b/2.0, -h;
       ur5eBB[1].init(temp, l, b, h);
       
       l = BB_dim[2][0];  b = BB_dim[2][1];  h = BB_dim[2][2];
       temp << 0.0, -l/ 2.0, -b/2.0 , 0.0, l/ 2.0, -b/2.0, 0.0, l/ 2.0, b/2.0, 0.0, -l/ 2.0, b/2.0, h, -l/ 2.0, -b/2.0, h, l/ 2.0, -b/2.0, h, l/ 2.0, b/2.0, h, -l/ 2.0, b/2.0;
       ur5eBB[2].init(temp, l, b, h);

       l = BB_dim[3][0];  b = BB_dim[3][1];  h = BB_dim[3][2];
       temp << -l/2.0, -b/2.0, 0.0, l/2.0, -b/2.0, 0.0, l/2.0, b/2.0, 0.0, -l/2.0, b/2.0, 0.0, -l/2.0, -b/2.0, h, l/2.0, -b/2.0, h, l/2.0, b/2.0, h, -l/2.0, b/2.0, h;
       ur5eBB[3].init(temp, l, b, h);
 
       l = BB_dim[4][0];  b = BB_dim[4][1];  h = BB_dim[4][2];
       temp << 0.0, -l/ 2.0, -b/2.0 , 0.0, l/ 2.0, -b/2.0, 0.0, l/ 2.0, b/2.0, 0.0, -l/ 2.0, b/2.0, h, -l/ 2.0, -b/2.0, h, l/ 2.0, -b/2.0, h, l/ 2.0, b/2.0, h, -l/ 2.0, b/2.0;
       ur5eBB[4].init(temp, l, b, h);
      
       l = BB_dim[5][0];  b = BB_dim[5][1];  h = BB_dim[5][2];
       temp << -l/2.0, 0.0 , -b/2.0, l/2.0, 0.0, -b/2.0, l/2.0, 0.0, b/2.0, -l/2.0, 0.0, b/2.0, -l/2.0, -h, -b/2.0, l/2.0, -h, -b/2.0, l/2.0, -h, b/2.0, -l/2.0, -h, b/2.0;
      ur5eBB[5].init(temp, l, b, h);

       l = BB_dim[6][0];  b = BB_dim[6][1];  h = BB_dim[6][2];
      temp << -l/2.0, 0.0 , -b/2.0, l/2.0, 0.0, -b/2.0, l/2.0, 0.0, b/2.0, -l/2.0, 0.0, b/2.0, -l/2.0, h, -b/2.0, l/2.0, h, -b/2.0, l/2.0, h, b/2.0, -l/2.0, h, b/2.0;
      ur5eBB[6].init(temp, l, b, h);

       l = BB_dim[7][0];  b = BB_dim[7][1];  h = BB_dim[7][2];
       temp << -l/2.0, -b/2.0, 0.0, l/2.0, -b/2.0, 0.0, l/2.0, b/2.0, 0.0, -l/2.0, b/2.0, 0.0, -l/2.0, -b/2.0, -h, l/2.0, -b/2.0, -h, l/2.0, b/2.0, -h, -l/2.0, b/2.0, -h;
       ur5eBB[7].init(temp, l, b, h);
    
    } 
    
    void transformUr5eBB(std::vector<double> thetas, Eigen::Matrix4d ur5eToOdom)
    {
      for(int i = 0; i < 8; i++)
      {
        Eigen::Matrix4d H = getTransformation(i+1, thetas);
        H = ur5eToOdom * H;
        ur5eBB[i].transformBox(H);
      }
    }
    
    Eigen::Matrix4d getTransformation(int link, std::vector<double> thetas)
    {
      std::vector<double> augThetas = {thetas[0], thetas[1], 0, 0, thetas[2], thetas[3], thetas[4], thetas[5]};
      Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
      for (int i = link - 1; i >=0; i--)
      {
        double d = DH[i][1];
        double r = DH[i][2];
        double alpha = DH[i][3];
        Eigen::Matrix4d temp {{std::cos(augThetas[i]), -std::sin(augThetas[i])*std::cos(alpha), std::sin(augThetas[i])* std::sin(alpha), r*std::cos(augThetas[i])}, {std::sin(augThetas[i]), std::cos(augThetas[i])*std::cos(alpha), -std::cos(augThetas[i])* std::sin(alpha), r*std::sin(augThetas[i])}, {0, std::sin(alpha), std::cos(alpha), d }, {0, 0, 0, 1}};
        H = temp * H;
      }
      Eigen::Matrix4d base {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
      H = base * H;
      return H;
    }
    
    std::vector<Eigen::Matrix4d> getAllTransformations(std::vector<double> thetas)
    {
      std::vector<Eigen::Matrix4d> allTransforms;
      std::vector<double> augThetas = {thetas[0], thetas[1], 0, 0, thetas[2], thetas[3], thetas[4], thetas[5]};
      Eigen::Matrix4d inertiaToBase {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
      for (int i = 0; i < 8; i++)
      {
        Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
        for (int j = i; j>=0; j--)
        {
          double d = DH[j][1];
          double r = DH[j][2];
          double alpha = DH[j][3];
          Eigen::Matrix4d temp {{std::cos(augThetas[j]), -std::sin(augThetas[j])*std::cos(alpha), std::sin(augThetas[j])* std::sin(alpha), r*std::cos(augThetas[j])}, {std::sin(augThetas[j]), std::cos(augThetas[j])*std::cos(alpha), -std::cos(augThetas[j])* std::sin(alpha), r*std::sin(augThetas[j])}, {0, std::sin(alpha), std::cos(alpha), d }, {0, 0, 0, 1}};
          H = temp * H; 
        }
        H = inertiaToBase * H;
        allTransforms.push_back(H);
      }
      return allTransforms;
    }
    
    Eigen::Matrix<double, 6, 6> getJacobian(std::vector<double> thetas)
    {
      std::vector<Eigen::Matrix4d> allTransforms = getAllTransformations(thetas);
      allTransforms.erase(allTransforms.begin() + 1, allTransforms.begin() + 3);
      Eigen::Matrix<double, 6, 6> jacobian;
      Eigen::Matrix<double, 3, 1> z0 {0, 0, 1};
      Eigen::Matrix<double, 3, 1> t_end = allTransforms.back().block(0, 3, 3, 1);
      for (int i = 0; i < 6; i++)
      {
        if (i == 0)
        {
          jacobian.block(0, i, 3, 1) = z0.cross(t_end);
          jacobian.block(3, i, 3, 1) = z0;
        }  
        else
        {
          Eigen::Matrix<double, 3, 1> vec1 = allTransforms[i-1].block(0, 2, 3, 1);
          Eigen::Matrix<double, 3, 1> vec2 = t_end - allTransforms[i-1].block(0, 3, 3, 1);
          jacobian.block(0, i, 3, 1) = vec1.cross(vec2);
          jacobian.block(3, i, 3, 1) = allTransforms[i-1].block(0, 2, 3, 1);
        }
      }
      return jacobian;
    }
    // returns true if the ur5e arm is colliding with the object
    bool checkCollision(OBB obj)
    {
      for (int i = 0; i < 8; i++)
      {
        if (ur5eBB[i].isCollision(obj))
          return true;
      }
      return false;
    }
    void visualizeUr5eBB(Gnuplot& g1)
    {  
      for (int i = 0; i < 8; i++)
      {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
        
        x = {ur5eBB[i].currVertices(0,0), ur5eBB[i].currVertices(1, 0), ur5eBB[i].currVertices(2, 0), ur5eBB[i].currVertices(3, 0), ur5eBB[i].currVertices(0, 0)}; 
        y = {ur5eBB[i].currVertices(0,1), ur5eBB[i].currVertices(1, 1), ur5eBB[i].currVertices(2, 1), ur5eBB[i].currVertices(3, 1), ur5eBB[i].currVertices(0, 1)}; 
        z = {ur5eBB[i].currVertices(0,2), ur5eBB[i].currVertices(1, 2), ur5eBB[i].currVertices(2, 2), ur5eBB[i].currVertices(3, 2), ur5eBB[i].currVertices(0, 2)}; 
        g1.plot_xyz(x,y,z);
        x.clear();
        y.clear();
        z.clear();


        x = {ur5eBB[i].currVertices(4,0), ur5eBB[i].currVertices(5, 0), ur5eBB[i].currVertices(6, 0), ur5eBB[i].currVertices(7, 0), ur5eBB[i].currVertices(4, 0)}; 
        y = {ur5eBB[i].currVertices(4,1), ur5eBB[i].currVertices(5, 1), ur5eBB[i].currVertices(6, 1), ur5eBB[i].currVertices(7, 1), ur5eBB[i].currVertices(4, 1)}; 
        z = {ur5eBB[i].currVertices(4,2), ur5eBB[i].currVertices(5, 2), ur5eBB[i].currVertices(6, 2), ur5eBB[i].currVertices(7, 2), ur5eBB[i].currVertices(4, 2)};
        g1.plot_xyz(x,y,z);
        x.clear();
        y.clear();
        z.clear();

        x = {ur5eBB[i].currVertices(0,0), ur5eBB[i].currVertices(4,0)};
        y = {ur5eBB[i].currVertices(0,1), ur5eBB[i].currVertices(4,1)};
        z = {ur5eBB[i].currVertices(0,2), ur5eBB[i].currVertices(4,2)};
        g1.plot_xyz(x,y,z);
        x.clear();
        y.clear();
        z.clear();

        x = {ur5eBB[i].currVertices(1,0), ur5eBB[i].currVertices(5,0)};
        y = {ur5eBB[i].currVertices(1,1), ur5eBB[i].currVertices(5,1)};
        z = {ur5eBB[i].currVertices(1,2), ur5eBB[i].currVertices(5,2)};
        g1.plot_xyz(x,y,z);
        x.clear();
        y.clear();
        z.clear();

        x = {ur5eBB[i].currVertices(2,0), ur5eBB[i].currVertices(6,0)};
        y = {ur5eBB[i].currVertices(2,1), ur5eBB[i].currVertices(6,1)};
        z = {ur5eBB[i].currVertices(2,2), ur5eBB[i].currVertices(6,2)};
        g1.plot_xyz(x,y,z);
        x.clear();
        y.clear();
        z.clear();

        x = {ur5eBB[i].currVertices(3,0), ur5eBB[i].currVertices(7,0)};
        y = {ur5eBB[i].currVertices(3,1), ur5eBB[i].currVertices(7,1)};
        z = {ur5eBB[i].currVertices(3,2), ur5eBB[i].currVertices(7,2)};
        g1.plot_xyz(x,y,z);
        x.clear();
        y.clear();
        z.clear();
      }
      //wait_for_key();
    }
    
    void wait_for_key ()
    {
      std::cout << std::endl << "Press ENTER to continue..." << std::endl; 
      std::cin.clear();
      std::cin.ignore(std::cin.rdbuf()->in_avail());
      std::cin.get();
      return;
    }
};
