/* navigation for minicar in certain shape */
#include "navigator.h"
#include "map_struct.h"
#include  <ros/ros.h>

Navigator::Navigator(NavInfo_t info)
{
    /* Normal Condition*/
    if(info.start == nullptr)

        throw std::invalid_argument("Start position is nullptr");

    if(info.goal == nullptr)
    {
        if(info.navmd != mode_racetrack)

            throw std::invalid_argument("Goal is nullptr");
        
        else
        {
            info.goal = info.start;
            ROS_INFO("Mode:RaceTrack, circle loop once, goal equal to start");
        }
    }       

    if(info.navmd == mode_map)
    {
        /*check not nullptr*/
        if(info.map == nullptr)

            throw std::invalid_argument("Map is nullptr");

        /* data safe */
        _info = info;
    }

    else if(info.navmd == mode_racetrack)
    {
        /* check, use for final */
        if(info.fpair == nullptr)
            if(info.ipair == nullptr || info.ipair->lhs.empty() || info.ipair->rhs.empty() )

                throw std::invalid_argument("Obstacle pairs grid array is nullptr or empty");

            else

                g2t_pair = true;

        else if(info.fpair->lhs.empty() || info.fpair->rhs.empty())

            throw std::invalid_argument("Obstacle pairs true array is empty");

        _info = info;
    }
    
    ROS_INFO("Navigator Initial Success");

}

Navigator::~Navigator(){}

nav_msgs::Path Navigator::plan(NavMode mode)
{
    /* return a vector of navigation points */
    if(mode == mode_map)
    {
        /* A star planner later TODO */
    }

    else if(mode == mode_racetrack)
    {
        if(g2t_pair)
        {
            /* make sure map data exist */
            if(_info.map == nullptr)
                
                throw std::invalid_argument("No map info e.g. origin and resolution to apply the tf : g2t");

            /* do g2t and assign to _info fpair */

            _info.fpair = g2t(_info.ipair);
        }

        /* make sure length correct for fpair given situation */
        make_sure_fpair_length();

        /* push back start */
        origin_path.poses.push_back(*_info.start);

        /* use fpair calc every pose and assign stamp...*/
        nav_msgs::Path tmp;
        tmp = calc_fpair_path();  //do it when certain mode

        /* path insert */
        origin_path.poses.insert(origin_path.poses.end(),tmp.poses.begin(),tmp.poses.end());
        
        /* push back goal*/
        origin_path.poses.push_back(*_info.goal);
        
    }
    
}

fConePair_t* Navigator::g2t(iConePair_t *icp)
{
    /* create return pointer */
    fConePair_t *fcp = new fConePair_t();

    /* get map info */
    float resolution = _info.map->info.resolution;
    geometry_msgs::Pose origin = _info.map->info.origin;

    /* init iterator, compare length is next step */
    vpi_t::iterator lit = icp->lhs.begin();
    vpi_t::iterator rit = icp->rhs.begin();

    /* tf form : g*resolution + origin, first is x coordinate, second is y coordinate */
    /* seperate for length diff case */
    for(lit; lit != icp->lhs.end(); lit++)
    {
        double lx = (double)lit->first*resolution + origin.position.x;
        double ly = (double)lit->second*resolution + origin.position.y;

        /* push back to fcp*/
        pd_t lhs(lx,ly);
        fcp->lhs.push_back(lhs);
    }

    for(rit;rit != icp->rhs.end();rit++)
    {
        double rx = (double)rit->first*resolution + origin.position.x;
        double ry = (double)rit->second*resolution + origin.position.y;

        /* push back to fcp*/
        pd_t rhs (rx,ry);
        fcp->rhs.push_back(rhs);
    }
    return fcp;
}

nav_msgs::Path Navigator::calc_fpair_path()
{   
    nav_msgs::Path path; 
    vpd_t::iterator lit = _info.fpair->lhs.begin();
    vpd_t::iterator rit = _info.fpair->rhs.begin();

    for( lit;lit != _info.fpair->lhs.end();lit++,rit++)
    {
        /* middle point */
        geometry_msgs::PoseStamped point;
        geometry_msgs::PoseStamped lastPoint;
        point.pose.position.x = (lit->first + rit->first)/2;
        point.pose.position.y = (lit->second + rit->second)/2;
        
        /* get last middle point and calc quaternion */
        if(!path.poses.empty())

            lastPoint = path.poses.back();
        
        /* if you want to change the first quaternion calc method, change here */
        // get from start
        else
        
            lastPoint = *_info.start;

        update_relative_q(&point,&lastPoint);

        path.poses.push_back(point);

    }

    /* return the path */
}

/* calc quaternion of current point related to last point */
void Navigator::update_relative_q(geometry_msgs::PoseStamped* cp,geometry_msgs::PoseStamped* lp)
{
    /* cp:current point ; lp:last point */

    /* pass roll pitch here */

    /* calc yaw angle first - not sure */
    double delta_x = cp->pose.position.x - lp->pose.position.x;
    double delta_y = cp->pose.position.y - lp->pose.position.y;

    double yaw = atan2(delta_y,delta_x);    // Z axis, X-Y plane
    double pitch = 0.0;     // Y axis, Z-X plane
    double roll = 0.0;      // X axis, Y-Z plane

    /* change to quaternion and assign to cp */
    ToQuaternion(cp,yaw,pitch,roll);
}

void Navigator::ToQuaternion(geometry_msgs::PoseStamped* p,double yaw, double pitch, double roll)
{   /* ref : https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles*/      

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    /* assign */
    p->pose.orientation.w = cr * cp * cy + sr * sp * sy;
    p->pose.orientation.x = sr * cp * cy - cr * sp * sy;
    p->pose.orientation.y = cr * sp * cy + sr * cp * sy;
    p->pose.orientation.z = cr * cp * sy - sr * sp * cy;
}

void Navigator::make_sure_fpair_length()
{
    int l_length = _info.fpair->lhs.size();
    int r_length = _info.fpair->rhs.size();
    if (l_length == r_length)
        return;

    else
    {
        /* diff size happen */
        vpd_t longer = (_info.fpair->lhs.size()>_info.fpair->rhs.size()) ? _info.fpair->lhs : _info.fpair->rhs;
        vpd_t shorter = (longer == _info.fpair->lhs) ? _info.fpair->rhs : _info.fpair->lhs;

        /* Dicision: pick the closest to the needed compensation point */
        vpd_t::iterator main = longer.begin();   // longer
        vpd_t::iterator sub = shorter.begin();    // shorter

        throw std::invalid_argument("length dismatch");

        for(main;main!=longer.end();main++)
        {
            /* TODO*/
        }
        
        return;

    }
    

    

}

/* TODO: smooth the path github https://github.com/gkouros/path-smoothing-ros/blob/master/src/cubic_spline_interpolator.cpp */