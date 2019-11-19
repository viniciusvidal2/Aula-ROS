#include "ros/ros.h"
#include "std_msgs/String.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <mavros/mavros.h>
#include <mavros/utils.h>
#include <mavlink/config.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

using namespace std;

/// Variaveis globais
///
float current_lat = 0, current_lon = 0, current_alt_global = 0;
float current_x = 0, current_y = 0, current_alt_local = 0;
float current_yaw = 0, des_alt = 50;

/// Callback Global
///
void escutaPosicaoGlobal(const sensor_msgs::NavSatFixConstPtr& msg)
{
    // Pega informacao de coordenadas geograficas para as variaveis globais
    current_lat = msg->latitude; current_lon = msg->longitude;
    current_alt_global = msg->altitude;
}

/// Callback Bussola
///
void escutaBussola(const std_msgs::Float64ConstPtr& msg)
{
    // Pega orientacao pela Bussola
    current_yaw = msg->data;
//    cout << "\n\nAtual orientacao: " << current_yaw << "\n\n";
}

/// Callback Local
///
void escutaPosicaoLocal(const geometry_msgs::PoseStampedConstPtr& msg)
{
    current_x = msg->pose.position.x; current_y = msg->pose.position.y; current_alt_local = msg->pose.position.z;
    // Checa altura se ja chegou, enquanto nao chegar nao tem conversa
    if(abs(current_alt_local - des_alt) < 1){
        // Le posicao atual a partir da mensagem

        // Se estiver dentro do desejado, altera o valor e publica, senao espera
    } else {
        ROS_INFO("Altitude atual: %.2f", msg->pose.position.z);
    }
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controle");
    ros::NodeHandle nh;

    // Declara subscriber posicao local
    ros::Subscriber subloc = nh.subscribe("/mavros/local_position/pose", 1000, escutaPosicaoLocal);

    // Declara subscriber posicao global
    ros::Subscriber subglo = nh.subscribe("/mavros/global_position/global", 1000, escutaPosicaoGlobal);

    // Declara subscriber orientacao
    ros::Subscriber subcom = nh.subscribe("/mavros/global_position/compass_hdg", 1000, escutaBussola);

    // Altera a taxa de comunicacao da placa pra ajudar a gente de alguma forma
//    ros::ServiceClient srvRate = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
//    mavros_msgs::StreamRate rate;
//    rate.request.stream_id = 0;
//    rate.request.message_rate = 30; // X Hz das mensagens que vem
//    rate.request.on_off = 1; // Nao sei
//    if(srvRate.call(rate))
//      ROS_INFO("Taxa do Mavlink mudada para %d Hz", rate.request.message_rate);
//    else
//      ROS_INFO("Nao pode alterar a taxa Mavlink.");

    // Inicia cliente e chama o servico de armar
    ros::ServiceClient srvArm = nh.serviceClient<mavros_msgs::CommandBool>("/marvos/cmd/arming");
    mavros_msgs::CommandBool arm;
    arm.request.value = 1;
    arm.response.result = 0;
    arm.response.success = true;
    for(int i=0; i < 4; i++){
        if(srvArm.call(arm))
            ROS_INFO("Drone armando... %d", arm.response.result);
        else
            ROS_INFO("Nao pode armar o drone meu consagrado. %d", arm.response.result);

    }

    // Espera um tempo de prache
    sleep(2);

    // Inicia cliente e chama o servico de takeoff, dai vai embora
    ros::ServiceClient srvTO = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL takeoff;
    takeoff.request.min_pitch = 0;
    takeoff.request.yaw       = current_yaw;
    takeoff.request.latitude  = current_lat;
    takeoff.request.longitude = current_lon;
    takeoff.request.altitude  = des_alt; // Aqui em metros e relativa

    if(srvTO.call(takeoff))
        ROS_INFO("Drone subindo para altitude de %.2f metros...", des_alt);
    else
        ROS_INFO("Nao foi possivel subir o drone.");

    // Spin eterno a principio
    ros::spin();

    return 0;
}
