#include "iostream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "CANComunication.h"
#include "stdio.h"
#include "string.h"
#include "canlib.h"
#include "canstat.h"
#include "math.h"
#define KPH_1 1
#define KPH_2 2
#define KPH_3 3
#define KPH_4 4
#define KPH_5 5

CANComunication can;
float goal[2];
float total_dist;
float current_dist;
bool once = true;
int base_speed = KPH_2;


double calc_angle(float x1, float y1)
{
     double a = 0;
     if (x1 == 0)
        a = 0;
     else if (x1 < 0 && y1 > 0)
         a =  -M_PI/2 - atan(y1/x1);
     else if ( x1 > 0 && y1 > 0)
         a = M_PI/2 - atan(y1/x1); 
     else if (x1 > 0 && y1 < 0)
         a = atan(y1/x1) + M_PI;
     else
         a = -M_PI/2 + atan(y1/x1); 
       
    a = a*180/M_PI; 
	
    std::cout << "a_before: " << a << std::endl;
    
	if (a > 40)
		a = 40;
	else if(a < -40)
		a = -40;
    std::cout << "a_after: " << a << std::endl;
    if(a>40)
        a=40;
    else if(a < -40)
        a = -40;
		return a*0.6;
}
double calc_distance(double x1, double y1)
{
	double d = sqrt(pow(0 - x1,2) + pow(0 - y1,2));
	return d;
}
double calc_speed()//in km/h
{   
    float frac = calc_distance(goal[0],goal[1]) / total_dist; 
	float speed = base_speed;
	if(frac < 0.1)
	{
        speed = speed * frac;
	}
	return speed;
}
double adjust_angle(double angle, double dec)
{
	double new_angle;
	if(angle > 0)
	{
		new_angle = angle - dec;
		if (new_angle < 0)
		{
			new_angle = 0;
		}
	}
	else if(angle < 0)
	{
		new_angle = angle + dec;
		if (new_angle > 0)
		{
			new_angle = 0;
		}
	}
	return new_angle;
}

void send(int speed, int angle)
{
	can.speed = speed;
	can.steer = angle;
	std::cout << "angle: (" << angle << ")" << std::endl;
	can.SendMessage();

}


void callback_pos(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data[0] == 0 && msg->data[1] == 0)
    {
        send(0,0);
        std::cout << "Stopping" << std::endl;  
    }
    else
    {
	    send(	calc_speed(),
	        (int)round(calc_angle(		msg->data[0], msg->data[1]	)));
	 
    }
}
void callback_goal(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    goal[0] = msg->data[0];
    goal[1] = msg->data[1];
    if(once)
    {
        total_dist = calc_distance(goal[0],goal[1]);        
    }
    
}
/*void callback_path(const std_msgs::String::ConstPtr& msg)
{
    std::string stopp_signal = "stopp";
	if (stopp_signal.compare(msg->data) == 0)
	{
		send(0,0);
		std::cout << "stop" << std::endl;
	}*/

int main(int argc, char **argv){
    std::cout << "main" << std::endl;
	can.StartCan();
	ros::init(argc,argv,"control_listener");
	ros::NodeHandle n;
	//ros::Subscriber sub_path = n.subscribe("aim_coords",3,callback_path);
	ros::Subscriber sub_pos = n.subscribe("aim_goal",2,callback_pos);
	ros::Subscriber sub_goal = n.subscribe("relative_goal",1,callback_goal);
	

	
	ros::spin();
}

