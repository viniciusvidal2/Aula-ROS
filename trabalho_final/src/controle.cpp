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
double current_x = 0, current_y = 0, current_z = 0;
double current_yaw = 0, des_alt = 14;

bool inicio = true, alcancou_altitude = false, alcancou_inicio = false, chegamos_ao_fim = false;
bool torre_alcancada = false; // Vai medir se chegamos a um poste e ai aponta para o proximo poste

ros::Publisher pub_setVel;
ros::Publisher local_pos_pub;

mavros_msgs::State current_state;

cv_bridge::CvImagePtr image_ptr;

float controle_roll, controle_alt;
float Kp_r = 0.02, Ki_r = 0.00000, Kd_r = 0.0005;
float Kp_h = 1, Ki_h = 0, Kd_h = 0;
float erro_acc_roll = 0, erro_anterior_r = 0;
float erro_acc_h    = 0, erro_anterior_h = 0;
float velocidade_linear = 1.0; // [m/s]
// Estrutura de waypoint das torres e vetor para armazenar o caminho
struct wpt
{
    float x; // [m]
    float y; // [m]
};
std::vector<wpt> wpts;
int wpt_atual = 0; // Indice da torre que estamos viajando para ela

/// Inicia a lista de pontos das nuvens
///
void preencheWaypoints(){
    // Coordenadas dos pontos
    std::vector<float> xs{54.91, 76.10, 126.02}; // [m]
    std::vector<float> ys{ 5.60, 51.95,  51.75}; // [m]
    // Coloca no vetor de waypoints para ser seguido
    for(size_t i=0; i < xs.size(); i++){
        wpt w = {xs[i], ys[i]};
        wpts.push_back(w);
    }
    wpt_atual = 0; // Garante que estamos indo para a primeira nuvem
}

/// Callback imagem da camera
///
void escutaCamera(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv
//    image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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

/// Callback Local
///
void escutaPosicaoLocal(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // Le posicao atual a partir da mensagem
    current_x = msg->pose.position.x; current_y = msg->pose.position.y; current_z = msg->pose.position.z;
}

/// Verifica se chegou na altitude desejada antes de mover ao inicio do poste
///
bool chegouNaAltitude(float g){
    // Enquando nao estamos na altitude, publicar e retornar false
    if(abs(current_z - g) > 0.1){
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
             pow(iniz - current_z, 2) ) >= 0.1){
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
            erro += (centro - i);
        }
    }
    // Atualiza erro de YAW se estiver muito baixo
    erro = (abs(erro) > 4) ? erro : 0;
    // Controla a existencia de velocidade linear
    velocidade_linear = (indices_validos.size() > 0) ? 2.0 : 0;

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
        erro_h = (abs(erro_h) > 0.1) ? erro_h : 0;
    }
    // Calculo do controlador de YAW - atualiza variavel de controle
    controle_roll   = (torre_alcancada == false) ? Kp_r*erro + Ki_r*erro_acc_roll + Kd_r*(erro - erro_anterior_r) : 0;
    erro_acc_roll  += erro;
    erro_anterior_r = erro;

    // Calculo do controlador de altitude - atualiza variavel de controle
    controle_alt    = (torre_alcancada == false) ? Kp_h*erro_h : 0;
    erro_acc_h     += erro_h;
    erro_anterior_h = erro_h;

    // Caso chegue a um local com muitas leituras, ha perigo, por enquanto parar, se nao for torre
    if(indices_validos.size() > 80 && !torre_alcancada){
        velocidade_linear = 0;
        controle_alt      = 0;
        controle_roll     = 0;
    }
}

/// Verificar se perto das torres e navegar corretamente
///
void comportamentoEmTorres(){
    // Verifica se esta proximo da proxima torre marcada em waypoint
    ROS_INFO("Distancia da torre = %.2f", sqrt( pow(wpts[wpt_atual].x - current_x, 2) + pow(wpts[wpt_atual].y - current_y, 2) ));
    if(sqrt( pow(wpts[wpt_atual].x - current_x, 2) + pow(wpts[wpt_atual].y - current_y, 2) ) < 3)
        torre_alcancada = true;
    // Se estamos sobre a torre, verificar o comportamento segundo a torre em questao e agir
    if(torre_alcancada){
        if(wpt_atual < wpts.size() - 1){ // Nao estamos na ultima torre, temos que virar
            // Descobrir angulo que virar
            float theta = atan2(wpts[wpt_atual+1].y - wpts[wpt_atual].y, wpts[wpt_atual+1].x - wpts[wpt_atual].x);
            // Cria mensagem
            mavros_msgs::PositionTarget pt;
            pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // Enviamos aqui no frame do corpo do drone, Y a frente, X a esquerda, Z para cima
            pt.type_mask = mavros_msgs::PositionTarget::IGNORE_VX  | mavros_msgs::PositionTarget::IGNORE_VY  | mavros_msgs::PositionTarget::IGNORE_VZ  | // Ignora velocidade
                           mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | // Ignora aceleracao
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;                                                                                 // Ignora YAW_RATE
            // Primeiro enviar YAW
            pt.position.x = current_x;
            pt.position.y = current_y;
            pt.position.z = current_z;
            pt.yaw = theta;
            ros::Rate r(7);
//            for(int i=0; i < 50; i++){
//                pub_setVel.publish(pt);
//                r.sleep();
//                ROS_INFO("Estamos virando ...");
//            }
            // Agora avancar posicao
//            pt.type_mask = pt.type_mask | mavros_msgs::PositionTarget::IGNORE_YAW;
            pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            pt.position.x = wpts[wpt_atual].x + 2*cos(theta);
            pt.position.y = wpts[wpt_atual].y + 2.5*sin(theta);
            pt.position.z = current_z;
            for(int i=0; i < 100; i++){
                pub_setVel.publish(pt);
            ROS_INFO("Estamos avancando ...");
                r.sleep();
            }
        } else { // Estamos na ultima torre, temos que parar
            chegamos_ao_fim = true;
        }
        // Vamos para o proximo waypoint
        wpt_atual++;
        // Torre nao esta mais alcancada seu animal
        torre_alcancada = false;
    }
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
    // Subscriber de estado de voo da aeronave
    ros::Subscriber substt = nh.subscribe("mavros/state", 10, escutaEstado);
    // Subscriber do laser
    ros::Subscriber sublsr = nh.subscribe("/scan1", 100, escutaLaser);
    // Subscriber da imagem
    ros::Subscriber subima = nh.subscribe("/cgo3_camera/image_raw", 10, escutaCamera);

    // Publisher de controle
    pub_setVel = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

    // Inicia a lista de Waypoints das torres
    preencheWaypoints();

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
                ROS_INFO("Sucesso TakeOff.");
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
        if(alcancou_inicio && alcancou_altitude && !torre_alcancada && !chegamos_ao_fim){


                ///////// AQUI SIM ENVIA COMANDOS PARA SEGUIR LINHA /////////
                // Ajusta com a metrica o quanto voar
                pt.velocity.y = velocidade_linear; // Avanco com Y [m/s] (positivo para frente)
                pt.velocity.z = controle_alt;      // No eixo de altitude [m/s] (positivo para cima)
                pt.velocity.x = controle_roll;     // Taxa de correcao do controle de YAW
                // Envia para o drone
                pub_setVel.publish(pt);
//                ROS_INFO("Enviando comando de CONTROLE.");
                ///////// AQUI SIM ENVIA COMANDOS PARA SEGUIR LINHA /////////


        }

        // Verifica se torre alcancada, se sim, tomar decisao
        comportamentoEmTorres();

        // Se chegamos ao fim do trajeto, paramos
        if(chegamos_ao_fim){
            pt.velocity.y = 0;
            pt.velocity.z = 0;
            pt.velocity.x = 0;
            pub_setVel.publish(pt);
            ROS_WARN("Chegamos ao fim, ta doendo sim...");
            ROS_WARN("Eu chego a perder a voz       ...");
            ROS_WARN("So resta chorar e se lamentar ...");
            ROS_WARN("Pelo que restou de nos        ...");
            ros::shutdown();
        }

        // Spin
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
