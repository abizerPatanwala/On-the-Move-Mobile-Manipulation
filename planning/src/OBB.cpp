#include <memory>
#include <bits/stdc++.h>
#include "gnuplot_i.hpp"
#include <Eigen/Dense>

class OBB
{
  public:
    Eigen::Matrix<double, 8, 3> vertices;
    Eigen::Matrix<double, 8, 3> currVertices;
    Eigen::Matrix<double, 3, 3> faceNormals;
    Eigen::Matrix<double, 3, 3> edges;
    double l; double b; double h;
    
    void init(Eigen::Matrix<double, 8, 3> vertices, double l, double b, double h)
    {
      this->vertices = vertices;
      this->currVertices = vertices;
      this->l = l;
      this->b = b;
      this->h = h;
      computeEdges();
      computeFaceNormals();
    }
   
    void computeEdges()
    {
      edges.row(0) = currVertices.row(0) - currVertices.row(1); // Edge in plane
      edges.row(1) = currVertices.row(1) - currVertices.row(2); // Edge in Plane 
      edges.row(2) = currVertices.row(1) - currVertices.row(5); // Edge perpendicular to plane
    } 
    
    void computeFaceNormals()
    {
      for (int i = 0; i <3; i++)
      {
        int next = (i+1) % 3;
        faceNormals.row(i) = ((edges.row(i).transpose()).cross(edges.row(next).transpose())).transpose();
        double norm = std::sqrt(faceNormals.row(i) * faceNormals.row(i).transpose()); 
        faceNormals.row(i) = faceNormals.row(i) / norm;
      }      
      
    }
  
    void transformBox(Eigen::Matrix4d H)
    {
      Eigen::Matrix<double, 8, 4> vertices_homog;
      vertices_homog.block(0,0,8,3) = vertices;
      vertices_homog.block(0,3,8,1) = Eigen::Matrix<double, 8, 1>::Ones();
      currVertices = ((H * vertices_homog.transpose()).transpose()).block(0, 0, 8, 3);
      computeEdges();
      computeFaceNormals();
    }
    
    // Returns true if the two boxes are colliding
    bool isCollision(OBB b)
    {
      Eigen::Matrix<double, 15, 3> projection;
      projection.block(0,0,3,3) = faceNormals;
      projection.block(3,0,3,3) = b.faceNormals;
      int count = 6;
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          projection.block(count,0,1,3) = ((edges.row(i).transpose()).cross(b.edges.row(j).transpose())).transpose();
          count++;
        }
      }
      
      double min1;
      double max1;
      double min2;
      double max2;
      for (int i = 0; i < 15; i++)
      {
        Eigen::Matrix<double, 8, 1> temp = currVertices * (projection.row(i).transpose());
        min1 = temp.minCoeff();
        max1 = temp.maxCoeff();
        temp = b.currVertices * (projection.row(i).transpose());
        min2 = temp.minCoeff();
        max2 = temp.maxCoeff();
        if ((min2 > max1) || (min1 > max2))
          return false;
      }
      return true; 
    }
    
    void visualizeBox(Gnuplot& g1)
    {
      std::vector<double> x;
      std::vector<double> y;
      std::vector<double> z;        
      
      x = {currVertices(0,0), currVertices(1, 0), currVertices(2, 0), currVertices(3, 0), currVertices(0, 0)}; 
      y = {currVertices(0,1), currVertices(1, 1), currVertices(2, 1), currVertices(3, 1), currVertices(0, 1)}; 
      z = {currVertices(0,2), currVertices(1, 2), currVertices(2, 2), currVertices(3, 2), currVertices(0, 2)}; 
      g1.plot_xyz(x,y,z);
      x.clear();
      y.clear();
      z.clear();


      x = {currVertices(4,0), currVertices(5, 0), currVertices(6, 0), currVertices(7, 0), currVertices(4, 0)}; 
      y = {currVertices(4,1), currVertices(5, 1), currVertices(6, 1), currVertices(7, 1), currVertices(4, 1)}; 
      z = {currVertices(4,2), currVertices(5, 2), currVertices(6, 2), currVertices(7, 2), currVertices(4, 2)};
      g1.plot_xyz(x,y,z);
      x.clear();
      y.clear();
      z.clear();

      x = {currVertices(0,0), currVertices(4,0)};
      y = {currVertices(0,1), currVertices(4,1)};
      z = {currVertices(0,2), currVertices(4,2)};
      g1.plot_xyz(x,y,z);
      x.clear();
      y.clear();
      z.clear();

      x = {currVertices(1,0), currVertices(5,0)};
      y = {currVertices(1,1), currVertices(5,1)};
      z = {currVertices(1,2), currVertices(5,2)};
      g1.plot_xyz(x,y,z);
      x.clear();
      y.clear();
      z.clear();

      x = {currVertices(2,0), currVertices(6,0)};
      y = {currVertices(2,1), currVertices(6,1)};
      z = {currVertices(2,2), currVertices(6,2)};
      g1.plot_xyz(x,y,z);
      x.clear();
      y.clear();
      z.clear();

      x = {currVertices(3,0), currVertices(7,0)};
      y = {currVertices(3,1), currVertices(7,1)};
      z = {currVertices(3,2), currVertices(7,2)};
      g1.plot_xyz(x,y,z);
      x.clear();
      y.clear();
      z.clear();
      
      Eigen::Matrix<double, 1, 3> center {(currVertices.row(0) + currVertices.row(2)) / 2.0};
      for (int i = 0; i < 3; i++)
      {
        x = {center(0,0), center(0,0) + faceNormals(i,0)};
        y = {center(0,1), center(0,1) + faceNormals(i,1)};
        z = {center(0,2), center(0,2) + faceNormals(i,2)};
        g1.plot_xyz(x,y,z);
        x.clear();
        y.clear();
        z.clear();
      }
      
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

