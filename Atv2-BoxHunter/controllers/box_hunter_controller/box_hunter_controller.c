#include <stdio.h>
#include <stdlib.h>

#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/led.h>

/*
 * Valor de limitação para igualdade de ângulos.
 * Previne erros por conta de "ruídos".
 */
#define THETA_THRESHOLD 1 //Grau
/*
 * Velocidade angular máxima das rodas. 
 */
#define MAX_SPEED 6.28 //rad/s

// velocidade tangencial/linear em m/s.
// Tangensial speed = angular speed * wheel radius
// Tangensial speed = 6.28 rad * 2.05 cm = 0.12874 m/s
#define TANGENSIAL_SPEED 0.12874

// Speed of robot to spinning in place (in cycles per second)
// 1 cycle = 360 degrees.
// Robot rotational speed = tangensial speed / (phi * axle length)
// note: axle length is distance between wheels
// Robot rotational speed = 0.12874 / (phi*0.052) = 0.787744755
#define ROBOT_ROTATIONAL_SPEED 0.772881647

// Velocidade do robô para rodar no lugar (graus/segundos)
// velocidade angular do robô = velocidade rotacional do robô * 360 
// Velocidade angulas do robô = 0.787744755*360 = 283.588111888 
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.587111888

// "Raio" de colisão válido
#define BOX_THRESHOLD 0.30

// Tempo de amostragem do sensor GPS
#define GPS_SAMPLING_PERIOD 1 //em milissegundos

// Tempo de amostragem do sensor de bússola
#define COMPASS_SAMPLING_PERIOD 1 //em milissegundos

// Motores
static WbDeviceTag left_motor, right_motor;

// gps
WbDeviceTag gps;

// compasso/bússola
WbDeviceTag compass;

// Nó do supervisor do E-Puck
WbNodeRef robot_node;

// Nós supervisores das caixas.
WbNodeRef box_0_node, box_1_node, box_2_node,
          box_3_node, box_4_node, box_5_node,
          box_6_node, box_7_node, box_8_node;

// Nó do supervisor da translação (X,Y,Z)
WbFieldRef trans_field[9];

// Nó do supervisor da rotação (X,Y,Z,ângulo) do robô
// o ângulo é dado em radianos
WbFieldRef rotac_field;

// LED frontal do robô
WbDeviceTag led;

// Sensores de proximidade
WbDeviceTag ps[8];

// Leitura da saída dos sensores
double ps_values[8];
const double *robot_position;	
const double *robot_rotation;

int time_step;

// Mapa de caixas dispostas no cenário (X,Y),
// Já convertido para cartesiano: @see convert_coords_to_cartesian;
double boxes[9][2] = {
  {-0.25,-0.25},  // Wooden box(0)
  {-0.25,0},     // Wooden box(1)
  {-0.25,0.25}, // Wooden box(2)
  {0, -0.25},     // Wooden box(3)
  {0, 0},        // Wooden box(4)
  {0,0.25},     // Wooden box(5)
  {0.25, -0.25},  // Wooden box(6)
  {0.25,0},      // Wooden box(7)
  {0.25,0.25}   // Wooden box(8)
};


// Vetor de Status da caixa, que é o estado das caixas e
// que variam entre: 0: Não visitada ou 1: Visitada.
// Isso previne o cálculo de menor distância para a mesma caixa sempre.
int visited_boxes[9];


/*
 * Retorna o tempo da simulação.
 */ 
int get_time_step(){
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  
  return time_step;
}

void step()
{
    if (wb_robot_step(time_step) == -1)
    {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

//==========================//
//   TRATAMENTO DA BÚSSOLA  //
//==========================//

/*
 * Obtém o direcionamento do robô através da bússola.
 * 
 * Código disponibilizado por: 
 * https://cyberbotics.com/doc/reference/compass
 */
double getRobotBearing()
{
    /* calculate bearing angle in degrees */
    const double *north = wb_compass_get_values(compass);
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0) {
        bearing = bearing + 360.0;
	}
	
    return bearing;
}


//==========================//
//TRATAMENTO DE CAIXAS LEVES//
//==========================//

bool target_box_moved(int target, const double current_coords[2]){
  double x_diff = fabs(current_coords[0] - boxes[target][0]);
  double y_diff = fabs(current_coords[1] - boxes[target][1]);
  
  if (x_diff > 0.01 || y_diff > 0.01)
    return true;
  else
    return false;
}


//==========================//
//  COORDENADAS CARTESIANAS //
//==========================//

/*
 * Conversão para coordenadas cartesianas. Função que converte as
 * coordenadas (X,Y,Z) para coordenadas (X,Y) que podem ser
 * utilizadas como cartesianas.
 * @param double[3] Coordenadas para serem transformadas.
 * @return double[2] Coordenadas convertidas para cartesianas.
 */
double *convert_coords_to_cartesian(const double coordinates[3]){
  double *coordenada_cartesiana = malloc(2);
  coordenada_cartesiana[0] = coordinates[0];
  coordenada_cartesiana[1] = -coordinates[2];
  return coordenada_cartesiana;
}

/*
 * Conversão da direção para cartesiano. Função que converte a
 * direção do robô no mundo do webots para o equivalente cartesiano.
 * @param double Ângulo de direção (webots)  do robô (em graus).
 * @return double Ângulo de direção (cartesiano) do robô (em graus).
 */
double convert_heading_to_cartesian(double heading){	

 heading = 360 - heading;
 
 /*
  * No Webots, a direção 0 (ângulo 0) ocorre caso o
  * robô esteja direcionado para o eixo y positivo (ou Z negativo)
  * Portanto, a direção cartesiana é a direção_robo + 90.
  *
  */
  heading = heading + 90;

  if (heading > 360.0)
    heading = heading - 360;

  return heading;
}

/*
 * Verifica se o ângulo de direção é o ângulo correto.
 * Função que verifica se o ângulo de direção do robô condiz
 * com o ângulo correto. Note que o ruído é tratado com a
 * diferença de THETA_THRESHOLD.
 * @param double Ângulo de comparação
 * @param double Ângulo de comparação
 * @return bool Se ambos os angulos estão alinhados.
 */
bool is_cartesian_theta_equal(const double theta, const double theta2){
  if(fabs(theta-theta2) < THETA_THRESHOLD)
    return true;
  else
    return false;
}

/*
 * Calcula o ângulo entre robô e destino.
 * Calcula o ângulo em graus, baseado nas coordenadas
 * atuais do robô e das coordenadas da caixa alvo.
 * @param double Coordenadas do robô (convertidas para o equivalente cartesiano)
 * @param double Coordenadas do alvo (convertidas para o equivalente cartesiano)
 * @return double Ângulo theta calculado via arcotangente (atan2)
 */
double calculate_destination_theta_in_degrees(const double robot_coord[2], const double dest_coord[2]){
  return atan2(dest_coord[1] - robot_coord[1], dest_coord[0] - robot_coord[0]) * 180 / M_PI;
}


/*
 * Calcula o ângulo correto entre robô e caixa.
 * Calcula o ângulo a partir da diferença da orientação
 * do robô (qual localidade o robô está direcionado) e o
 * ângulo correto para a caixa alvo.
 *
 * @param heading Ângulo de direção do robô.
 * @param dest_theta Ângulo da caixa alvo.
 * 
 * @return double Ângulo que o robô deve seguir para colidir
 * com a caixa alvo.
 *
 */
double calculate_robot_theta(double heading, double dest_theta){
  double theta = dest_theta - heading;
  printf("Dest_theta: %4.4f\nHeading: %4.4f\n",dest_theta,heading);
  if (theta > 180)
    theta = -(360-theta);
  else if (theta < -180)
    theta = (360+theta);

  return theta;
}


/*
 * Retorna a distância entre os pontoWbDeviceTags.
 *
 */
double calculate_distance(const double robot_coord[2], const double dest_coord[2]){
  return sqrt(pow(dest_coord[0] - robot_coord[0], 2) + pow(dest_coord[1] - robot_coord[1], 2));
}

/*
 * Retorna a direção do robô em graus.
 *
 */
double convert_heading_in_degrees(double heading_in_rad){
  return heading_in_rad * 180 / M_PI;
}


/**
* Adquire a caixa mais próxima do robô e a indica.
*
* Calcula a distância mínima entre o robô e todas as caixas mapeadas,
* dessa forma, independente do ponto inicial do robô ele poderá se
* localizar corretamente.
*
* @param robot_coord Array com o posiciWbDeviceTagonamento do robô já convertido para cartesiano.
* @param boxes Matriz de dimensões conhecidas e contém as posições X
*              e Y das caixas.
* @param visited_boxes Array contendo status das caixas, que varia entre
*              0: Não visitada e 1: Visitada.
*
* @return int índice da matriz com as coordenadas da caixa alvo mais próxima.
*/
int get_min_distance_box_info(const double robot_coord[2], double boxes[9][2], int visited_boxes[9]){
  int i = 0;
  double min_distance = 1000;
  double temp = 0.0;
  int caixa_alvo = 0;
  printf("Caixas visitadas\n");
  for (i = 0; i < 9; i++){
    printf("--->Caixa %d: %d\n",i,visited_boxes[i]);

    // Caixa não foi visitada, pode ser considera na descoberta de menor distância.
    if(visited_boxes[i] == 0){
      // Fórmula de distância entre dois pontos.
      temp = calculate_distance(robot_coord, boxes[i]);
      if (temp <= min_distance){
        min_distance = temp;
        // Salva a caixa que possui a menor distância.
        caixa_alvo = i;
      }
    } 
  }
  return caixa_alvo;
}

//==========================//
//          MOTORES         //
//==========================//

/*
 * Função que define velocidade 0 à ambos motores.
 * Fazendo o robô parar.
 */
void motor_stop(){
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

/*
 * Função que define velocidade máxima à ambos motores.
 * Fazendo que o robô vá para frente.
 */
void motor_move_forward(){
  wb_motor_set_velocity(left_motor, 0.53*MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.53*MAX_SPEED);
}

/*
 * Função que define velocidade negativa máxima ao motor esquerdo
 * e vlocidade positiva máxima ao motor direito.
 * Fazendo que o robô rode (em seu próprio eixo) para a esqueda.
 */
void motor_rotate_left(){
  wb_motor_set_velocity(left_motor, -0.53*MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.53*MAX_SPEED);
}

/*
 * Função que define velocidade positiva máxima ao motor esquerdo
 * e vlocidade negativa máxima ao motor direito.
 * Fazendo que o robô rode (em seu próprio eixo) para a direita.
 */
void motor_rotate_right(){
  wb_motor_set_velocity(left_motor, 0.53*MAX_SPEED);
  wb_motor_set_velocity(right_motor, -0.53*MAX_SPEED);
}

/*
 * Direciona o robô para a caixa alvo.
 * Define qual a rotação deve ser empregada pelo robô
 * a fim que a caixa alvo seja alcançada.
 *
 * @param theta Ângulo que o robô deve girar para que a diferença entre
 * direção do robô e o ângulo entre robô caixa seja zerado.
 * @param time_step Tempo da simulação.
 *
 */
void rotate_to_box(const double theta){
  
  if (!is_cartesian_theta_equal(theta,0)){
    double duration = abs(theta) / (ROBOT_ANGULAR_SPEED_IN_DEGREES * 0.6);	
    if(theta>0)
      motor_rotate_left();
    else if (theta < 0)
      motor_rotate_right();

    double start_time = wb_robot_get_time();
    do{
      step();
    }while (wb_robot_get_time() < start_time + duration);	
  }
}

/*
 * Realiza o deslocamento em linha reta.
 * A partir da distância, o robô desloca-se em linha reta
 * e sem interrupção.
 *
 * @param distance Distância entre o robô e a caixa alvo.
 * @param time_step Tempo da simulação.
 */
void move_forward(double distance){

  double duration = (distance*0.8) / TANGENSIAL_SPEED;
  printf("Duração até o destino: %4.4f\n", duration);

  motor_move_forward();

  double start_time = wb_robot_get_time();
  do{
    step();
  }while (wb_robot_get_time() < start_time + duration);
        
  motor_stop();
}


/*
 * Função utilitária para mover o robô conforme a necessidade.
 * Toda a movimentação dessa função é realizada de maneira
 * ininterrupta.
 *
 * @param distance Distância (ou ângulo) que deverá ser percorrida (girado).
 * @param option Opção de movimentação (1: frente, 2: esquerda, 3: direita).
 */
void move_robot_special(double distance, int option){
  
  double duration = 0.0;
  if (option == 1){
    duration = (distance*0.8) / TANGENSIAL_SPEED;
    printf("Duração até o destino: %4.4f\n", duration);
    motor_move_forward();
  }
  else if (option == 2){
    duration = distance / (ROBOT_ANGULAR_SPEED_IN_DEGREES * 0.6);
    motor_rotate_left();
  }
  else if (option == 3){
    duration = distance / (ROBOT_ANGULAR_SPEED_IN_DEGREES * 0.6);
    motor_rotate_right();
  }

  double start_time = wb_robot_get_time();
  do{
    step();
  }while (wb_robot_get_time() < start_time + duration);
        
           //motor_stop();
}



//==========================//
//     CÓDIGO PRINCIPAL     //
//==========================//


void init(){
  time_step = get_time_step();
	
  // Nome dos sensores de proximidade
  char ps_names[8][4] ={
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  
  led = wb_robot_get_device("led0");
  
  // Inicialização dos sensores.
  for (int i = 0; i < 8 ; i++) {
    // Liga sensores de proximidade
    ps[i] = wb_robot_get_device(ps_names[i]);
            
    // Ativa todos os sensores, os resultados serão coletados
    // periódicamente em milissegundos TIME_STEP. 
    wb_distance_sensor_enable(ps[i], time_step);
  }
    	
  // Inicialização dos motores das rodas direita e esquerda.
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Mapeia o robô através do supervisor
  robot_node = wb_supervisor_node_get_from_def("e-Puck");
  
  // Nós supervisores das caixas
  box_0_node = wb_supervisor_node_get_from_def("Caixa0");
  box_1_node = wb_supervisor_node_get_from_def("Caixa1");
  box_2_node = wb_supervisor_node_get_from_def("Caixa2");
  box_3_node = wb_supervisor_node_get_from_def("Caixa3");
  box_4_node = wb_supervisor_node_get_from_def("Caixa4");
  box_5_node = wb_supervisor_node_get_from_def("Caixa5");
  box_6_node = wb_supervisor_node_get_from_def("Caixa6");
  box_7_node = wb_supervisor_node_get_from_def("Caixa7");
  box_8_node = wb_supervisor_node_get_from_def("Caixa8");
  
  trans_field[0] = wb_supervisor_node_get_proto_field(box_0_node,"translation");
  trans_field[1] = wb_supervisor_node_get_proto_field(box_1_node,"translation");
  trans_field[2] = wb_supervisor_node_get_proto_field(box_2_node,"translation");
  trans_field[3] = wb_supervisor_node_get_proto_field(box_3_node,"translation");
  trans_field[4] = wb_supervisor_node_get_proto_field(box_4_node,"translation");
  trans_field[5] = wb_supervisor_node_get_proto_field(box_5_node,"translation");
  trans_field[6] = wb_supervisor_node_get_proto_field(box_6_node,"translation");
  trans_field[7] = wb_supervisor_node_get_proto_field(box_7_node,"translation");
  trans_field[8] = wb_supervisor_node_get_proto_field(box_8_node,"translation");
          
}

int main(int argc, char **argv){
  wb_robot_init();

  init();

  int target = -1;

  int error_counter = 0;
  
  int collision_counter = 0;
  	
  double *current_coord;
  double theta_destination;
  double robot_heading;
  double destination_distance;
  double theta_dot;
  double *box_pos;
  
  double boxes_current[9][2];
  
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, GPS_SAMPLING_PERIOD);

  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, COMPASS_SAMPLING_PERIOD);
	
  while(true){
  
    
    wb_led_set(led, 0);
    //if (is_final_state())
      //break;
      
    // Atualiza sensores de proximidade
    for (int i = 0; i < 8 ; i++){
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
      box_pos = convert_coords_to_cartesian(wb_supervisor_field_get_sf_vec3f(trans_field[i]));
      
      boxes_current[i][0] = roundf(box_pos[0] * 100) / 100;
      boxes_current[i][1] = roundf(box_pos[1] * 100) / 100;
                   
      printf("Node Caixa %d: X = %4.4f Y = %4.4f \n", i, boxes_current[i][0], boxes_current[i][1]);
      
    }
    		
    bool right_obstacle =  ps_values[0] > 140.0 ||
                          ps_values[1] > 140.0;
    bool left_obstacle =   ps_values[6] > 140.0 ||
                          ps_values[7] > 140.0;
                       
    // Adquire a coordenada cartesiana do robô.
    current_coord = convert_coords_to_cartesian(wb_gps_get_values(gps));

    if(target == -1){
      error_counter = 0;
      target = get_min_distance_box_info(current_coord, boxes, visited_boxes);
      printf("Entidade X | Y\nRobô: %4.4f | %4.4f\nCaixa Destino: %d \n\t %4.4f | %4.4f\n",
            current_coord[0], current_coord[1], target, boxes[target][0], boxes[target][1]
	 );	
		

      //theta_destination = calculate_destination_theta_in_degrees(current_coord, boxes[target]);
      robot_heading = convert_heading_to_cartesian(getRobotBearing());
      
      theta_destination = calculate_destination_theta_in_degrees(current_coord, boxes[target]);
      
      printf("Theta antes de calcular: %4.4f\n", theta_destination);
      
      theta_dot = calculate_robot_theta(robot_heading, theta_destination);
            
      printf("Theta p/ Destino: %4.4f\n", theta_dot);
      
      rotate_to_box(theta_dot);
                
      destination_distance = calculate_distance(current_coord, boxes[target]);
                
      printf("Distância até o destino: %4.4f\n", destination_distance);
              
      move_forward(destination_distance);
      
    }
    
    //printf("Caixa alvo: %d\n",target);
    destination_distance = calculate_distance(current_coord, boxes[target]);
    //printf("Distância até a caixa alvo: %4.4f\n",destination_distance);
		
    if(right_obstacle || left_obstacle){
       
      if(target_box_moved(target, boxes_current[target])){
        printf("TARGET: %d, BOXES X: %4.4f Y: %4.4f \n", target, boxes_current[target][0], boxes_current[target][1]);
        wb_led_set(led, 1);
      }
               		  
      if (destination_distance <= 0.1){
        visited_boxes[target] = 1;
        collision_counter = 0;
        printf(">>>>>>>>>>> COLISÃO <<<<<<<<<<\n");
      }
             
      if(left_obstacle && right_obstacle){
        printf("Colisão frontal\n");
        move_robot_special(180,2);
      }
      else if(left_obstacle){
        printf("Colisão esquerda\n");
        collision_counter += 1;
        move_robot_special(30,3);
      }
      else if(right_obstacle){
        printf("Colisão direita\n");
        collision_counter += 1;
        move_robot_special(30,2);
      } 
      move_robot_special(0.2,1);
      target = -1;
    }
    
    if (collision_counter >= 3){
      printf(">>>> Colisão repetitiva: Recalculando rota\n");
      move_robot_special(90,3);
      move_robot_special(0.2,1);
      target = -1;
    }
                      
    if (error_counter > 10){
      printf("Recalculando rota\n");
      target = -1;
    }
    
    error_counter += 1;
    step();                                    
  }
}