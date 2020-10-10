#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include <mobile_control/motorMsg.h>
#include <ethercat_test/vel.h>
#include <geometry_msgs/Twist.h>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using ethercat_test::vel;
double motor_input[4];

void motorsCallback(const vel::ConstPtr& motors){
  for(int i = 0; i < 4; ++i)
    motor_input[i] = motors->velocity[i];

  motor_input[1] = -motor_input[1];
  motor_input[2] = -motor_input[2];
}

// Add gaussian noise
double AGN(double mean, double stddev)
{
  double result;
  std::mt19937 generator(std::random_device{} ());
  std::normal_distribution<double> dist(mean,stddev);
  result = dist(generator);
  return result;
}


int main(int argc,char **argv){

  ros::init(argc,argv,"motor_dynamics");
  ros::NodeHandle nh;

  ros::Subscriber motor_sub = nh.subscribe("/input_msg",1,motorsCallback);

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/vel",1);
  ros::Publisher measure_pub = nh.advertise<vel>("/measure",1);
  ros::Rate loop_rate(200);

  // Mecanum platform specification //
  double wheel_radius = 0.076;
  double wheel_separation_a = 0.2170;
  double wheel_separation_b = 0.1687;
  double l = wheel_separation_a + wheel_separation_b;

  // Motor specification //
  double gear_ratio = 74.5;
  double rpm_to_radps = 2*M_PI/60;
  double radps_to_rpm = 60/2.0/M_PI;
  double motor_act[4]={0,0,0,0};
  double alp[4]={0,0,0,0};
  const double alp_positive = 6000;
  const double alp_negative = -6000;
  const double w_max = 8;
  const double alp_max = 6000;
  const double Kp[4] = {1,1,1,1};
  const double Kd[4] = {0.1,0.1,0.1,0.1};
  double dt=0.01;
  double curr_time=0;
  double last_time=0;
  int index=1;

  double vx;
  double vy;
  double vp;

  double del1=0;
  double del2=0;
  double del3=0;
  double del4=0;
  double p[4];
  double mean=0;
  double stddev=0.04;


  while(ros::ok()){
      geometry_msgs::Twist cmd_vel;
      vel measure_msg;
      curr_time = ros::Time::now().toSec();
      dt = curr_time - last_time;

      // Time //
      if(index==1||dt==0||dt>1){
          dt=0.01;
          index=2;
      }
      last_time = curr_time;


      // input msg : unit conversion //
      motor_input[0] = (double) motor_input[0];
      motor_input[1] = (double) motor_input[1];
      motor_input[2] = (double) motor_input[2];
      motor_input[3] = (double) motor_input[3];

      // RPS to RPM //
      motor_act[0] = motor_act[0]*gear_ratio*radps_to_rpm;
      motor_act[1] = motor_act[1]*gear_ratio*radps_to_rpm;
      motor_act[2] = motor_act[2]*gear_ratio*radps_to_rpm;
      motor_act[3] = motor_act[3]*gear_ratio*radps_to_rpm;

      // angular acceleration : P control //
      alp[0] = Kp[0]*(motor_input[0]-motor_act[0]);
      alp[1] = Kp[1]*(motor_input[1]-motor_act[1]);
      alp[2] = Kp[2]*(motor_input[2]-motor_act[2]);
      alp[3] = Kp[3]*(motor_input[3]-motor_act[3]);

      // When error reaches maximum angular acceleration,
      // the angular acceleration sets to the maximum angular acceleration.
    if(fabs(motor_input[0]-motor_act[0])/dt>alp_max){
        alp[0] = motor_input[0]>motor_act[0] ? alp_positive:alp_negative;
    }

    if(fabs(motor_input[1]-motor_act[1])/dt>alp_max){
        alp[1] = motor_input[1]>motor_act[1] ? alp_positive:alp_negative;
    }


    if(fabs(motor_input[2]-motor_act[2])/dt>alp_max){
        alp[2] = motor_input[2]>motor_act[2] ? alp_positive:alp_negative;
    }


    if(fabs(motor_input[3]-motor_act[3])/dt>alp_max){
        alp[3] = motor_input[3]>motor_act[3] ? alp_positive:alp_negative;
    }

    // Calculate Motor velocities //
      motor_act[0] = motor_act[0] + alp[0] * dt;
      motor_act[1] = motor_act[1] + alp[1] * dt;
      motor_act[2] = motor_act[2] + alp[2] * dt;
      motor_act[3] = motor_act[3] + alp[3] * dt;

      measure_msg.velocity[0] = motor_act[0];
      measure_msg.velocity[1] = -motor_act[1];
      measure_msg.velocity[2] = -motor_act[2];
      measure_msg.velocity[3] = motor_act[3];
      measure_pub.publish(measure_msg);    

    // RPM to RADPS conversion to offer velocity to simulator //
      motor_act[0] = motor_act[0]/gear_ratio*rpm_to_radps;
      motor_act[1] = motor_act[1]/gear_ratio*rpm_to_radps;
      motor_act[2] = motor_act[2]/gear_ratio*rpm_to_radps;
      motor_act[3] = motor_act[3]/gear_ratio*rpm_to_radps;

      for(int i=0;i<4;i++)
      {
	        if(fabs(motor_act[i]) > w_max) motor_act[i]=motor_act[i] > 0? w_max:-w_max;
      }


      // Gaussian noise //
      del1 = AGN(mean,stddev);
      del2 = AGN(mean,stddev);
      del3 = AGN(mean,stddev);
      del4 = AGN(mean,stddev);

      del1 = fabs(del1);
      del2 = fabs(del2);
      del3 = fabs(del3);
      del4 = fabs(del4);

      del1 = del1>=1?1:del1;
      del2 = del2>=1?1:del2;
      del3 = del3>=1?1:del3;
      del4 = del4>=1?1:del4;

      del1=0;
      del2=0;
      del3=0;
      del4=0;
      


      // Forward Kinematics //
      vx = wheel_radius/4.0 * (motor_act[0]*(1-del1) + motor_act[1]*(1-del2) + motor_act[2]*(1-del3) + motor_act[3]*(1-del4));
      vy = wheel_radius/4.0 * (-motor_act[0]*(1-del1) + motor_act[1]*(1-del2) - motor_act[2]*(1-del3) + motor_act[3]*(1-del4));
      vp = wheel_radius/4.0/l * (-motor_act[0]*(1-del1) + motor_act[1]*(1-del2) + motor_act[2]*(1-del3) - motor_act[3]*(1-del4));

      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vp;
      vel_pub.publish(cmd_vel);  

      ros::spinOnce();
      loop_rate.sleep();

  }
  return 0;
}
