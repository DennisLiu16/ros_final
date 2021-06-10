#ifndef ros_private_navigator_H
#define ros_private_navigator_H

#include "map_struct.h" 
#include <nav_msgs/Path.h>
#include <string>

typedef enum PlanType {TP_Voronoi,TP_Middle,MP_Astar};

class Navigator
{
    public:
        /* var start */
        nav_msgs::Path origin_path;

        /* func start*/
        Navigator(NavInfo_t);
        ~Navigator();
        void InitTopic();
        std::string getPathPuber();
        void setPathPuber(std::string);

        nav_msgs::Path plan(NavMode);
        nav_msgs::Path g2t(nav_msgs::Path);
        nav_msgs::Path t2g(nav_msgs::Path);
        geometry_msgs::PoseStamped g2t(geometry_msgs::PoseStamped);
        geometry_msgs::PoseStamped t2g(geometry_msgs::PoseStamped);
        fConePair_t* g2t(iConePair_t*);   // no need t2g

    protected:
        NavInfo_t _info;
        bool g2t_pair = false;
        PlanType _plantype = TP_Middle;

        nav_msgs::Path calc_fpair_path();   //calc every pair middle point in true coordinate, and related angle in q4
        void update_relative_q(geometry_msgs::PoseStamped*,geometry_msgs::PoseStamped*);
        void ToQuaternion(geometry_msgs::PoseStamped* p,double yaw, double pitch, double roll); 
        void make_sure_fpair_length();   
        

        
        
        

        



};

#endif