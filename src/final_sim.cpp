# include <ros/ros.h>
# include <std_msgs/Float64MultiArray.h>
# include <geometry_msgs/Twist.h>
# include <math.h>
# include <stdbool.h>
# define PI 3.141592

float goal_x[]={
    15.555,
    15.95,
    15.95,
    15.555,
    15.3575,
    14.5675,
    14.1725,
    13.3825,
    13.185,
    12.79,
    12.79,
    13.185,
    13.185,
    12.79,
    12.79,
    13.185,
    13.3825,
    14.1725,
    14.5675,
    15.3575,
    15.555,
    15.95,
    15.95,
    15.555,

    15.555,
    15.95,
    15.95,
    15.555,
    15.3575,
    14.5675,
    14.1725,
    13.3825,
    13.185,
    12.79,
    12.79,
    13.185,
    13.185,
    12.79,
    12.79,
    13.185,
    13.3825,
    14.1725,
    14.5675,
    15.3575,
    15.555,
    15.95,
    15.95,
    15.555
    };

float goal_y[]={
    14.9625,
    15.3575,
    15.7525,
    16.5425,
    16.74,
    17.135,
    17.135,
    16.74,
    16.5425,
    15.7525,
    15.3575,
    14.9625,
    14.5675,
    14.1725,
    13.7775,
    12.9875,
    12.79,
    12.395,
    12.395,
    12.79,
    12.9875,
    13.7775,
    14.1725,
    14.5675,

    14.9625,
    15.3575,
    15.7525,
    16.5425,
    16.74,
    17.135,
    17.135,
    16.74,
    16.5425,
    15.7525,
    15.3575,
    14.9625,
    14.5675,
    14.1725,
    13.7775,
    12.9875,
    12.79,
    12.395,
    12.395,
    12.79,
    12.9875,
    13.7775,
    14.1725,
    14.5675
    };

float goal_rad[]={
    1.10714871779409,
    1.10714871779409,
    2.0344439357957,
    2.0344439357957,
    2.67794504458899,
    2.67794504458899,
    -2.67794504458899,
    -2.67794504458899,
    -2.0344439357957,
    -2.0344439357957,
    -1.10714871779409,
    -1.10714871779409,
    -2.0344439357957,
    -2.0344439357957,
    -1.10714871779409,
    -1.10714871779409,
    -0.463647609000805,
    -0.463647609000806,
    0.463647609000808,
    0.463647609000805,
    1.10714871779409,
    1.10714871779409,
    2.0344439357957,
    2.0344439357957,

    1.10714871779409,
    1.10714871779409,
    2.0344439357957,
    2.0344439357957,
    2.67794504458899,
    2.67794504458899,
    -2.67794504458899,
    -2.67794504458899,
    -2.0344439357957,
    -2.0344439357957,
    -1.10714871779409,
    -1.10714871779409,
    -2.0344439357957,
    -2.0344439357957,
    -1.10714871779409,
    -1.10714871779409,
    -0.463647609000805,
    -0.463647609000806,
    0.463647609000808,
    0.463647609000805,
    1.10714871779409,
    1.10714871779409,
    2.0344439357957,
    2.0344439357957,
    };

std_msgs::Float64MultiArrayConstPtr status;

class Controller
{
    public:
    int k=0;
    float v;
    float w;
    float rho;
    float alpha;
    float beta;
    float k_rho = 0.3;  //0.35
    float k_alpha = 2.0;    //2.0
    float k_beta = -1.5;   //-1.5
    float target_x;
    float target_y;
    float target_rad;
    float delta_x;
    float delta_y;
    float delta_rad;
    float range = 0.2;
    geometry_msgs::Twist cmd;
    
    void calc_state();   //including rho,alpha,beta
    void update_cmd();     //v ,w
    bool close(float,float);

    private:
    void check_limit();
};

void Controller::calc_state()
{
    /* use current status */
    auto data = status->data;
    float pos_x = data.at(0);
    float pos_y = data.at(1);
    float pos_rad = data.at(2);

    if(k < sizeof(goal_x)/sizeof(goal_x[0]))
    {
        target_x = goal_x[k];
        target_y = goal_y[k];
        target_rad = goal_rad[k];
        ROS_INFO("======================");
        ROS_INFO("%d point",k);
    }
    else
    {
        alpha = beta = rho = 0;
        return;
    }
    ROS_INFO("pos_x:%.2f,pos_y:%.2f",pos_x,pos_y);
    delta_x = target_x - pos_x;
    delta_y = target_y - pos_y;
    delta_rad = target_rad - pos_rad; 

    rho = sqrt( pow(delta_x,2) + pow(delta_y,2) );
    alpha = -pos_rad + atan2(delta_y,delta_x);
    beta = delta_rad - alpha;

    while(alpha > PI)
    {
        alpha -= 2*PI;
    }
    while(alpha <= -PI)
    {
        alpha += 2*PI;
    }

    while(beta > PI)
    {
        beta -= 2*PI;
    }
    while(beta <= -PI)
    {
        beta += 2*PI;
    }

}

void Controller::update_cmd()
{
    
    v = k_rho * rho;
    w = k_alpha * alpha + k_beta * beta;
    if(v > 0.5) v = 0.5;
    if(w > 1.0) w = 1.0;
    cmd.linear.x = v;
    cmd.angular.z = w;
    ROS_INFO("v,w is %.2f,%.2f",v,w);
    ROS_INFO("d_x:%.2f,d_y:%.2f",delta_x,delta_y);
    ROS_INFO("target_x:%.2f,target_y:%.2f",target_x,target_y);
    ROS_INFO("======================");
    if(close(delta_x,delta_y))
        k++;
}

bool Controller::close(float d_x,float d_y)
{
    ROS_INFO("dx:%.2f,dy:%.2f,range is %.2f",d_x,d_y,sqrtf(d_x*d_x+d_y*d_y));
    if(sqrt(d_x*d_x+d_y*d_y) < range)
    {
        return true;
    }
        
    return false;
}

void poseCallback(const std_msgs::Float64MultiArrayConstPtr &pose)
{
    status = pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "final");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("/status", 1, poseCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::Rate rate(10);
    Controller controller;
    /*sub pub*/
    while(ros::ok())
    {
        if(status != nullptr)
        {
            controller.calc_state();
            controller.update_cmd();
            vel_pub.publish(controller.cmd);
        }
        
        ros::spinOnce();         
        rate.sleep();
    }
    return 0;
}