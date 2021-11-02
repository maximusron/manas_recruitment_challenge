#include "math.h"
#include <array>
#include <string.h>
#include <queue>
#include <iostream>
#include <stack>
#include <stdio.h>
#include <limits>
#include <cfloat>
#include <chrono>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <planner/my_msg.h> // to publish
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


using namespace std;
using std::string;
#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

struct Node{ // info about a particular cell used for finding the path
    int parent;
    double f,g; // f = g+h

    Node(){ //constructor to assign with default values
        parent = -1;
        f = DBL_MAX;
        g = DBL_MAX;
        //h = DBL_MAX;
    }
};

typedef pair<double, int> Pair;

namespace globalplanner{
   /* class GlobalPlanner : public nav_core::BaseGlobalPlanner{
        public:
            float originX;
            float originY;
            float resolution;
            int width;
            int height;
            bool initialized_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            ros::NodeHandle my_nh;
            ros::Publisher pubb;


            GlobalPlanner();
            GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            GlobalPlanner(ros::NodeHandle& n);
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
            
            
            
            

            
            

            void getCoordinate(float& x, float& y);
            bool isPointInside(float x, float y);
            int convertToCellIndex(float x, float y);

            int getCellIndex(int i, int j){ // pass (i,j)=(y,x)
                return (width*i + j);
            }
    };*/

    class PlannerAlgorithm{
        public:
            int width, height, maplen;
            vector<int> costmap_;

            PlannerAlgorithm(int w,int h,int len,const vector<int>& costmap){
                width = w;
                height = h;
                maplen = len;
                costmap_ = costmap;
            }
            bool isValid(const int& p);
            bool isNotBlocked(const int& p);
            double distance(const int& src, const int& dest);
            double distance_squared(const int& src, const int& dest);
	        double diagonalDist(const int& src, const int& dest);
            double manhattanDist(const int& src, const int& dest);
            double heuristic(const int& src, const int& dest);
            vector<int> findNeighbours(int index);
            vector<int> findpath(const vector<Node>& matrix, const int& dest);
            double kernel_cost(int index, int ksize);
            double avg_kernel_cost(int index, int ksize);
            bool isInside(const int& row, const int& col);
    };
    class Astar : public PlannerAlgorithm{
        public:
            Astar(int w,int h,int len,const vector<int>& costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            
    };
    class RAstar : public PlannerAlgorithm{
        public:
            RAstar(int w,int h,int len,const vector<int>& costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            vector<int> tracePath(const int& src,const int& dest,const vector<double>& g_score);

    };
    class Tstar : public PlannerAlgorithm{
        public:
            Tstar(int w,int h,int len,const vector<int>& costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            double line_of_sight(const int& s, const int& s1);
            
    };
};
#endif
