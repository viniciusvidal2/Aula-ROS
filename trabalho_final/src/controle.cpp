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
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

using namespace std;

/// Variaveis globais
///
double current_lat, current_lon, current_alt_global;
double current_x = 0, current_y = 0, current_alt_local = 0;
double current_yaw = 0, des_alt = 2.5;

bool inicio = true;

ros::Publisher pub_setVel;

mavros_msgs::State current_state;

/// Callback estado de voo
///
void escutaEstado(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/// Callback Global
///
void escutaPosicaoGlobal(const sensor_msgs::NavSatFixConstPtr& msg)
{
    // Pega informacao de coordenadas geograficas para as variaveis globais
    current_lat = msg->latitude; current_lon = msg->longitude;
    current_alt_global = msg->altitude;
    // Verifica se e o ou nao inicio de voo, dai realiza takeoff
//    if(inicio && abs(current_alt_global - 537) < 5){
//        mavros_msgs::CommandTOL takeoff;
//        while(current_lat == 0){ // garantir que chegou coisa nova
//            ROS_INFO("Aguardando leitura de GPS FIX...");
//            sleep(1);
//        }
//        takeoff.request.min_pitch = 0;
//        takeoff.request.yaw       = current_yaw;
//        takeoff.request.latitude  = current_lat;
//        takeoff.request.longitude = current_lon;
//        takeoff.request.altitude  = des_alt; // Aqui em metros e relativa
//        if(srvTO.call(takeoff))
//            ROS_INFO("Drone subindo para altitude de %.2f metros, LAT: %f  LON %f ...", des_alt, current_lat, current_lon);
//        else
//            ROS_INFO("Nao foi possivel subir o drone.");
//        inicio = false; // So uma vez e necessario
//    }
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
    // Le posicao atual a partir da mensagem
    current_x = msg->pose.position.x; current_y = msg->pose.position.y; current_alt_local = msg->pose.position.z;
    // Checa altura se ja chegou, enquanto nao chegar nao tem conversa
    if(abs(current_alt_local - des_alt) < 0.3){

        // Cria a mensagem que vai para o controle
        mavros_msgs::PositionTarget pt;
        // Ajusta com a metrica o quanto voar
        pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        pt.type_mask = mavros_msgs::PositionTarget::IGNORE_PZ;
        pt.velocity.x = 0.2;
        pt.velocity.y = 0;
        pt.velocity.z = 0;
        // Envia para o drone
        pub_setVel.publish(pt);

        sleep(10);

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
    ros::Subscriber subloc = nh.subscribe("/mavros/local_position/pose", 100, escutaPosicaoLocal);

    // Declara subscriber posicao global
    ros::Subscriber subglo = nh.subscribe("/mavros/global_position/global", 100, escutaPosicaoGlobal);

    // Declara subscriber orientacao
    ros::Subscriber subcom = nh.subscribe("/mavros/global_position/compass_hdg", 100, escutaBussola);

    // Declara subscriber de estado de voo da aeronave
    ros::Subscriber substt = nh.subscribe("mavros/state", 10, escutaEstado);

    // Declara o Publisher de controle
    pub_setVel = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

    // Inicia o servico de takeoff sobre o drone
    ros::ServiceClient srvTO = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    // Inicia aqui o servico para mudar modo de controle da simulacao
    ros::ServiceClient srvMode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // Espera inicio de comunicacao
    ros::Rate r(20);
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        r.sleep();
    }

    // Altera a taxa de comunicacao da placa pra ajudar a gente de alguma forma
    ros::ServiceClient srvRate = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    mavros_msgs::StreamRate rate;
    rate.request.stream_id = 0;
    rate.request.message_rate = 30; // X Hz das mensagens que vem
    rate.request.on_off = 1; // Nao sei
    if(srvRate.call(rate))
        ROS_INFO("Taxa do Mavlink mudada para %d Hz", rate.request.message_rate);
    else
        ROS_INFO("Nao pode alterar a taxa Mavlink.");

    // Espera um tempo de prache
    sleep(5);

    // Inicia cliente e chama o servico de armar
    ros::ServiceClient srvArm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool arm;
    arm.request.value = true;
    if(srvArm.call(arm))
        ROS_INFO("Drone armando...");
    else
        ROS_INFO("Nao pode armar o drone meu consagrado. %d", arm.response.result);

    // Espera um tempo de prache
    sleep(2);

    // Muda de modo aqui - talvez nao va
    srvMode.call(offb_set_mode);

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        // Checa e envia o novo modo a cada tempo
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){
            if(srvMode.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                ROS_INFO("Offboard enabled.");
            last_request = ros::Time::now();
        }
        // Manda takeoff uma vez
        if(inicio && current_lat != 0 && current_yaw !=0){
            mavros_msgs::CommandTOL takeoff;
            takeoff.request.min_pitch = 0;
            takeoff.request.yaw       = current_yaw;
            takeoff.request.latitude  = current_lat;
            takeoff.request.longitude = current_lon;
            takeoff.request.altitude  = des_alt; // Aqui em metros e relativa
            if(srvTO.call(takeoff))
                ROS_INFO("Drone subindo para altitude de %.2f metros, LAT: %f  LON %f ...", des_alt, current_lat, current_lon);
            else
                ROS_INFO("Nao foi possivel subir o drone.");
            inicio = false; // So uma vez e necessario
        }
        // Spin
        ros::spinOnce();
        r.sleep();
    }

    // Spin eterno a principio
    ros::spin();

    return 0;
}
