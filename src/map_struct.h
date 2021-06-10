#ifndef ros_private_map_struct_H
#define ros_private_map_struct_H

/* aka obstacle and goal struct */

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>  
                                                
    typedef std::pair<int,int> pi_t;
    typedef std::pair<double,double> pd_t;
    typedef std::vector<pi_t> vpi_t;  //vector pair int type
    typedef std::vector<pd_t> vpd_t;  //vector pair int type
    typedef enum NavMode {mode_map,mode_racetrack};

    struct iConePair_t
    {
        vpi_t lhs;
        vpi_t rhs;
    };

    struct fConePair_t
    {
        vpd_t lhs;
        vpd_t rhs;
    };

    struct NavInfo_t
    {
        /* Necessary */
        nav_msgs::OccupancyGridConstPtr map = nullptr;
        geometry_msgs::PoseStampedConstPtr goal = nullptr;
        
        
        /* Optional*/
        geometry_msgs::PoseStampedConstPtr start = nullptr;  // will start at your location as default
        NavMode navmd = mode_map;   //if use racetrack in import map(e.g. symmetry cones), change to mode_racetrack
        iConePair_t *ipair = nullptr;   //2D, pair of cones in grid
        fConePair_t *fpair = nullptr;     //2D, pair of cones in true coordinate   
    };

        
#endif