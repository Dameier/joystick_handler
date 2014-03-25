#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/LedAnim.h>
#include <ardrone_autonomy/Navdata.h>

class ControlARDrone
{
public:
  ControlARDrone();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void navCallback(const ardrone_autonomy::Navdata::ConstPtr& nav);

  ros::NodeHandle nh_;

  bool hold_a, hold_b, hold_x, hold_y, hold_lbump, hold_rbump, hold_start, hold_reset, hold_xbox, auto_on;
  int linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, buttoncount;
  double l_scale_, a_scale_;

  ros::Publisher vel_pub_;
  ros::Publisher ar_launch;
  ros::Publisher ar_land;
  ros::Subscriber joy_sub_;
  ros::Subscriber tag_sub;
  ros::Publisher ar_reset;
  ros::ServiceClient cam_toggle;
  ros::ServiceClient led_change;
  ardrone_autonomy::CamSelect camsrv;
  ardrone_autonomy::LedAnim ledsrv;
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response response;
  std_msgs::Empty empty_msg;
};

ControlARDrone::ControlARDrone() {
  linear_x = 4; linear_y=3; linear_z=1; angular_x=0; angular_y=0; angular_z=0;
  a_scale_= 4.0; l_scale_= 4.0; auto_on = false;
  nh_.param("axis_linearx", linear_x, linear_x);
  nh_.param("axis_lineary", linear_y, linear_y);
  nh_.param("axis_linearz", linear_z, linear_z);
  nh_.param("axis_angularx", angular_x, angular_x);
  nh_.param("axis_angulary", angular_y, angular_y);
  nh_.param("axis_angularz", angular_z, angular_z);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ar_land = nh_.advertise<std_msgs::Empty>("/ardrone/land", 10);
  ar_launch = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
  ar_reset = nh_.advertise<std_msgs::Empty>("/ardrone/reset", 1);

  cam_toggle = nh_.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel");
  led_change = nh_.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");
  ledsrv.request.duration = 5;
  ledsrv.request.freq = 4;
  camsrv.request.channel = 0;
  buttoncount=0;

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &ControlARDrone::joyCallback, this);
  tag_sub = nh_.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 100, &ControlARDrone::navCallback, this);
}

void ControlARDrone::navCallback(const ardrone_autonomy::Navdata::ConstPtr& nav){
    geometry_msgs::Twist vel;
    if (nav->tags_count == 1 && auto_on){
        int x = nav->tags_xc[0];
        int y = nav->tags_yc[0];
        if(x < 400){
            vel.linear.x = .25;
            vel.linear.y=0;
            vel.linear.z=0;
            vel.angular.z = 0;
            vel_pub_.publish(vel);
        }else if(x > 600){
            vel.linear.x=-.25;
            vel.linear.y=0;
            vel.linear.z=0;
            vel.angular.z = 0;
            vel_pub_.publish(vel);
        }else if(y < 400){
            vel.linear.x=0;
            vel.linear.y=.25;
            vel.linear.z=0;
            vel.angular.z = 0;
            vel_pub_.publish(vel);
        }else if(y > 600){
            vel.linear.x=0;
            vel.linear.y=-.25;
            vel.linear.z=0;
            vel.angular.z = 0;
            vel_pub_.publish(vel);
        }else{
            int degrees = nav->tags_orientation[0];
            if (degrees < 170) {
                vel.linear.x=0;
                vel.linear.y=0;
                vel.linear.z=0;
                vel.angular.z = -.25;
                vel_pub_.publish(vel);
            }else if (degrees > 190) {
                vel.linear.x=0;
                vel.linear.y=0;
                vel.linear.z=0;
                vel.angular.z = .25;
                vel_pub_.publish(vel);
            }
        }
    }
}

void ControlARDrone::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    if(joy->buttons[0]&&!hold_a){
        ar_launch.publish(empty_msg);
        hold_a=true;
    }else if(joy->buttons[0]==0){hold_a=false;}

    if(joy->buttons[1]&&!hold_b){
        ar_land.publish(empty_msg);
        hold_b=true;
    }else if(joy->buttons[1]==0){hold_b=false;}

    if(joy->buttons[2]&&!hold_x){
        buttoncount++;
        if (buttoncount>13){
            buttoncount = 0;
        }
        ledsrv.request.type = buttoncount;
        led_change.call(ledsrv);
        hold_x=true;
    }else if(joy->buttons[2]==0){hold_x=false;}

    if(joy->buttons[3]&&!hold_y){
        if (camsrv.request.channel == 0){camsrv.request.channel =1;}else{camsrv.request.channel =0;}
        cam_toggle.call(camsrv);
        hold_y=true;
    }else if(joy->buttons[3]==0){hold_y=false;}

    if(joy->buttons[4]&&!hold_rbump){

        hold_rbump=true;
    }else if(joy->buttons[4]==0){hold_rbump=false;}

    if(joy->buttons[5]&&!hold_lbump){

        hold_lbump=true;
    }else if(joy->buttons[5]==0){hold_lbump=false;}

    if(joy->buttons[6]&&!hold_reset){
        ar_reset.publish(empty_msg);
        hold_reset=true;
        auto_on = false;
    }else if(joy->buttons[6]==0){hold_reset=false;}

    if(joy->buttons[7]&&!hold_start){
        ar_reset.publish(empty_msg);
        hold_start=true;
    }else if(joy->buttons[7]==0){hold_start=false;}

    if(joy->buttons[8]&&!hold_xbox){
        if (auto_on){
            auto_on = false;
            ledsrv.request.type = 0;
            led_change.call(ledsrv);
        }else{
            auto_on = true;
            ledsrv.request.type = 6;
            led_change.call(ledsrv);
        }
        hold_xbox=true;
    }else if(joy->buttons[8]==0){hold_xbox=false;}

  geometry_msgs::Twist vel;

  vel.linear.x = 3*joy->axes[linear_x];
  vel.linear.y = 3*joy->axes[linear_y];
  vel.linear.z = 3*joy->axes[linear_z];
  vel.angular.z = 3*joy->axes[angular_z];
  vel_pub_.publish(vel);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "tele_op");
  ControlARDrone tele_op;

  ros::spin();
}
