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
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>


using namespace std;
using std::string;
#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

struct Node{ // info about a particular cell used for finding the path
    int parent;
    double f,g; // f = g+h. removed h from here(not req to store)

    Node(){ //constructor to assign with default values
        parent = -1;
        f = DBL_MAX;
        g = DBL_MAX;
        //h = DBL_MAX;
    }
};

struct Index{
    int pos,orientation;
};

struct Node_big{ // info about a particular cell used for finding the path
    Index parent;
    double f,g,vel_r,vel_l; // f = g+h. removed h from here(not req to store)

    Node_big(){ //constructor to assign with default values
        parent.pos = -1;
        parent.orientation = -1;
        f = DBL_MAX;
        g = DBL_MAX;
        //h = DBL_MAX;
    }
};

struct Neighbour_smooth{
    int pos,orientation;
    double d,c,V_l,V_r;
};

typedef pair<double, int> Pair;
typedef tuple<double, int, int> Tuple;


namespace globalplanner{
    class GlobalPlanner : public nav_core::BaseGlobalPlanner{
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

            int iter=0,prev_goal,checkpt_dist=300,checkpoint;
            float temp_dist=0,full_dist=0;

            float accn,d_robot;
            Index smooth_prev_start;
            int time_frame;

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
    };

    class PlannerAlgorithm{
        public:
            int width, height, maplen;
            costmap_2d::Costmap2D* costmap_;

            PlannerAlgorithm(int w,int h,int len,costmap_2d::Costmap2D* costmap){
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
    class Astar_smooth : public PlannerAlgorithm{
        public:
            Astar_smooth(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<Index> search_path(const Index& start_pt, const int& dest,const double& accn,vector<double> initial_vel,const int& time_frame,const double& d_robot);
            vector<Neighbour_smooth> findNeighbours(const int& index,const int& angle,const double& accn,const double& U_l,const double& U_r,const int& time_frame,const double& d_robot);
            vector<Index> findpath(const vector<vector<Node_big>>& matrix, const int& dest);
            
    };
    class Astar : public PlannerAlgorithm{
        public:
            Astar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            
    };
    class RAstar : public PlannerAlgorithm{
        public:
            RAstar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            vector<int> tracePath(const int& src,const int& dest,const vector<double>& g_score);

    };
    class Tstar : public PlannerAlgorithm{
        public:
            Tstar(int w,int h,int len,costmap_2d::Costmap2D* costmap)
                : PlannerAlgorithm(w,h,len,costmap)
            {}

            vector<int> search_path(const int& src, const int& dest);
            double line_of_sight(const int& s, const int& s1);
            
    };
};
#endif
