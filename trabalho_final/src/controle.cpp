#include "ros/ros.h"
#include "std_msgs/String.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <math.h>

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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

/// Variaveis globais
///
double current_lat, current_lon, current_alt_global;
double current_x = 0, current_y = 0, current_alt_local = 0;
double current_yaw = 0, des_alt = 14;

bool inicio = true, alcancou_altitude = false, alcancou_inicio = false;

ros::Publisher pub_setVel;
ros::Publisher local_pos_pub;

mavros_msgs::State current_state;

cv_bridge::CvImagePtr image_ptr;

float controle_roll, controle_alt;
float Kp_r = 0.02, Ki_r = 0.00001, Kd_r = 0.0005;
float Kp_h = 2, Ki_h = 0, Kd_h = 0;
float erro_acc_roll = 0, erro_anterior_r = 0;
float erro_acc_h    = 0, erro_anterior_h = 0;
float velocidade_linear = 0.4; // [m/s]

/// Callback imagem da camera
///
void escutaCamera(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv
    image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

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
}

/// Verifica se chegou na altitude desejada antes de mover ao inicio do poste
///
bool chegouNaAltitude(float g){
    // Enquando nao estamos na altitude, publicar e retornar false
    if(abs(current_alt_local - g) > 0.1){
        // Enviar comando
        geometry_msgs::PoseStamped pose;
        pose.pose.position.z = g;
        pose.pose.position.x = current_x;
        pose.pose.position.y = current_y;
        local_pos_pub.publish(pose);
        ROS_INFO("Subindo o drone ate %.2f metros antes de ir para o inicio....", g);

        // Retornar false - nao chegamos ainda
        return false;
    } else { // Se chegamos, mandar logo true e seguir em frente

        return true;

    }
}

/// Verifica se chegou no inicio do poste
///
bool chegouNoInicio(){
    float inix = 6.5, iniy = 5, iniz = 14; // Inicio do poste [m]
    // Enquanto nao estamos proximos do ponto de inicio, enviar comando para la
    // Uma vez que chegar, nao entrar mais
    if(sqrt( pow(inix - current_x        , 2) +
             pow(iniy - current_y        , 2) +
             pow(iniz - current_alt_local, 2) ) >= 0.1){
        // Enviar comando
        geometry_msgs::PoseStamped pose;
        pose.pose.position.z = iniz;
        pose.pose.position.x = inix;
        pose.pose.position.y = iniy;
        local_pos_pub.publish(pose);
        ROS_INFO("Caminhando para o inicio....");

        return false;
    } else {

        return true;

    }
}

/// Callback medicao do laser
///
void escutaLaser(const sensor_msgs::LaserScanConstPtr &msg_laser){
    // Variaveis
    std::vector<int> indices_validos;
    std::vector<float> dist_validas;
    float erro = 0, centro = float(msg_laser->ranges.size()/2);
    /// Aplica tecnica de controle ///
    // Varre vetor de leituras atras dos indices com leituras validas e suas medidas
    for(size_t i = 0; i < msg_laser->ranges.size(); i++){
        if(msg_laser->ranges[i] < msg_laser->range_max){
            indices_validos.push_back(i);
            dist_validas.push_back(msg_laser->ranges[i]);
            // Atualiza o erro de YAW
            erro += (centro - i); ///float(msg_laser->ranges.size());
        }
    }
    // Atualiza erro de YAW se estiver muito baixo
    erro = (abs(erro) > 4) ? erro : 0;
    // Controla a existencia de velocidade linear
    velocidade_linear = (abs(erro) > 8) ? abs(erro)*Kp_r : 1;

    // Para erro de altitude, usar a primeira leitura somente por padrao
    // Angulo = pct / meio_range * range_angulo/2  ou  diferenca_para_o_centro * incremento_angular_cada_medida
    //
    //   |\
    //   |a\
    //  h|  \leitura
    //   |   \
    //   X---cabo
    //
    float distancia_manter = 3, erro_h, h, angulo_leitura;
    if(dist_validas.size() > 0){
        // Calculo do angulo e altura h
        angulo_leitura = float(abs(indices_validos[0] - int(msg_laser->ranges.size())/2)) * msg_laser->angle_increment; // [RAD]
        h = dist_validas[0]*cosf(angulo_leitura); // [m]
        // Erro de altitude
        erro_h = distancia_manter - h;
        erro_h = (erro_h > 0.2) ? erro_h : 0;
    }
    // Calculo do controlador de YAW - atualiza variavel de controle
    controle_roll   = Kp_r*erro + Ki_r*erro_acc_roll + Kd_r*(erro - erro_anterior_r);
    erro_acc_roll  += erro;
    erro_anterior_r = erro;

    // Calculo do controlador de altitude - atualiza variavel de controle
    controle_alt    = Kp_h*erro_h;
    erro_acc_h     += erro_h;
    erro_anterior_h = erro_h;
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controle");
    ros::NodeHandle nh;

    // Subscriber posicao local
    ros::Subscriber subloc = nh.subscribe("/mavros/local_position/pose", 100, escutaPosicaoLocal);
    // Subscriber posicao global
    ros::Subscriber subglo = nh.subscribe("/mavros/global_position/global", 100, escutaPosicaoGlobal);
    // Subscriber orientacao
    ros::Subscriber subcom = nh.subscribe("/mavros/global_position/compass_hdg", 100, escutaBussola);
    // Subscriber de estado de voo da aeronave
    ros::Subscriber substt = nh.subscribe("mavros/state", 10, escutaEstado);
    // Subscriber do laser
    ros::Subscriber sublsr = nh.subscribe("/scan1", 100, escutaLaser);
    // Subscriber da imagem
    ros::Subscriber subima = nh.subscribe("/cgo3_camera/image_raw", 10, escutaCamera);

    // Publisher de controle
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
    ROS_INFO("Conexao iniciada com a Pixhawk.");

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

    // Inicia enviando algumas posicoes para mudar bem de modo - exemplo online, nao faz tanta logica
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = des_alt;
    for(int i = 50; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Iniciou controle por comandos OFFBOARD.");

    // Espera um tempo de prache
    sleep(1);

    // Inicia cliente e chama o servico de armar
    ros::ServiceClient srvArm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool arm;
    arm.request.value = true;
    if(srvArm.call(arm))
        ROS_INFO("Drone armando...");
    else
        ROS_INFO("Nao pode armar o drone meu consagrado. %d", arm.response.result);

    // Espera um tempo de prache
    sleep(1);

    // Muda de modo aqui - talvez nao va
    srvMode.call(offb_set_mode);
    ROS_INFO("Modo OFFBOARD.");

    // Cria a mensagem que vai para o controle
    mavros_msgs::PositionTarget pt;
    pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // Enviamos aqui no frame do corpo do drone, Y a frente, X a esquerda, Z para cima
    pt.type_mask = mavros_msgs::PositionTarget::IGNORE_PX  | mavros_msgs::PositionTarget::IGNORE_PY  | mavros_msgs::PositionTarget::IGNORE_PZ  | // Ignora posicao
                   mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | // Ignora aceleracao
                   mavros_msgs::PositionTarget::IGNORE_YAW;                                                                                      // Ignora YAW

    ////////////////////////
    /// LOOP DE CONTROLE ///
    ////////////////////////
    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        // Checa e envia o modo de VOO OFFBOARD a cada tempo
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
            if(srvMode.call(offb_set_mode))
                ROS_INFO("Modo OFFBOARD.");
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
            // Inicio nao existira mais, e manda subir aqui so por desencargo
            pose.pose.position.z = des_alt;
            pose.pose.position.x = current_x;
            pose.pose.position.y = current_y;
            local_pos_pub.publish(pose);
            ROS_INFO("Subindo !!");
            inicio = false; // So uma vez e necessario
        }
        // Enquanto nao alcancamos altitude, nem ao ponto vamos
        if(!alcancou_altitude)
            alcancou_altitude = chegouNaAltitude(des_alt);
        // Enquanto nao estamos no inicio desejado, continuar enviando essa posicao para o drone
        if(!alcancou_inicio && alcancou_altitude)
            alcancou_inicio = chegouNoInicio();

        // Se chegamos no inicio do poste, parar de fazer subir e comecar a controlar
        if(alcancou_inicio && alcancou_altitude){


                ///////// AQUI SIM ENVIA COMANDOS PARA SEGUIR LINHA /////////
                // Ajusta com a metrica o quanto voar
                pt.velocity.y = velocidade_linear; // Avanco com Y [m/s] (positivo para frente)
                pt.velocity.z = controle_alt;      // No eixo de altitude [m/s] (positivo para cima)
                pt.velocity.x = controle_roll;     // Taxa de correcao do controle de YAW
                // Envia para o drone
                pub_setVel.publish(pt);
                ROS_INFO("Enviando comando de CONTROLE. Controle ROLL: %.4f", controle_roll);
                ///////// AQUI SIM ENVIA COMANDOS PARA SEGUIR LINHA /////////


        }

        // Spin
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
