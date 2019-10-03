#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Illuminance.h>

#include <geometry_msgs/Twist.h>

#include <turtlesim/SetPen.h>
#include <turtlesim/Color.h>

#include <trabalho1/thresh.h>

using namespace sensor_msgs;
using namespace geometry_msgs;

float xl, za;
float thresh = 1.0, sens = 2;

turtlesim::SetPen pen;
ros::ServiceClient client;

void escuta_imu(const ImuConstPtr& msg)
{

  float xt = msg->linear_acceleration.x, yt = msg->linear_acceleration.y;
  ROS_INFO("Xt: %.2f    Yt: %.2f", xt, yt);
  // MOVIMENTO ANGULAR
  if(xt > thresh){
      za = (xt - thresh)/sens;
  } else if(xt <= thresh && xt > -thresh) {
      za = 0;
  } else {
      za = (xt + thresh)/sens;
  }
  // MOVIMENTO LINEAR
  if(yt < -thresh){ // Frente
      xl = -(yt + thresh)/sens;
      pen.request.r = 0; pen.request.g = 250; pen.request.b = 0;
      client.call(pen);
  } else if(yt < thresh && yt >= -thresh) {
      xl = 0;
  } else { // Tras
      xl = -(yt - thresh)/sens;
      pen.request.r = 250; pen.request.g = 0; pen.request.b = 0;
      client.call(pen);
  }

}

bool ajusta_thresh(trabalho1::thresh::Request &req, trabalho1::thresh::Response &res){
    thresh = req.thresh;
    sens   = req.vel;

    if(sens <= 2){
        res.alerta = "A tartaruga esta nervosa!";
    } else {
        res.alerta = "A tartaruga esta de boa !";
    }
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_position_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/phone1/android/imu", 10, escuta_imu);
  ros::Publisher  pub = nh.advertise<Twist>("/turtle1/cmd_vel", 10);
  client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  ros::ServiceServer server = nh.advertiseService("set_thresh", ajusta_thresh);

  // Criar a mensagem Twist
  Twist tartaruga;

  ros::Rate r(5);
  while(ros::ok()){
      tartaruga.linear.x  = xl;
      tartaruga.angular.z = za;
      // Publicar a mensagem
      pub.publish(tartaruga);
      // Mandar o mestre fucionar
      ros::spinOnce();
      // Dormir o resto
      r.sleep();
  }

  return 0;
}
