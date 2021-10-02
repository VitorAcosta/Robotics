#include <stdio.h>
#include <stdlib.h>

#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/distance_sensor.h>

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
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.587

#define BOX_THRESHOLD 0.35


// Motores
static WbDeviceTag left_motor, right_motor;
// Nó do supervisor do E-Puck
WbNodeRef robot_node;
// Nó do supervisor da translação (X,Y,Z) do robô
WbFieldRef trans_field;
// Nó do supervisor da rotação (X,Y,Z,ângulo) do robô
// o ângulo é dado em radianos
WbFieldRef rotac_field;
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

           // heading = 360 - heading;
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

	if (theta > 180 - THETA_THRESHOLD)
		theta = -(360-theta);
	else if (theta < -180 + THETA_THRESHOLD)
		theta = (360+theta);

	return theta + 1;
}


/*
 * Retorna a distância entre os pontos.
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
* @param robot_coord Array com o posicionamento do robô já convertido para cartesiano.
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
	wb_motor_set_velocity(left_motor, MAX_SPEED);
	wb_motor_set_velocity(right_motor, MAX_SPEED);
}

/*
 * Função que define velocidade negativa máxima ao motor esquerdo
 * e vlocidade positiva máxima ao motor direito.
 * Fazendo que o robô rode (em seu próprio eixo) para a esqueda.
 */
void motor_rotate_left(){
	wb_motor_set_velocity(left_motor, -MAX_SPEED);
	wb_motor_set_velocity(right_motor, MAX_SPEED);
}

/*
 * Função que define velocidade positiva máxima ao motor esquerdo
 * e vlocidade negativa máxima ao motor direito.
 * Fazendo que o robô rode (em seu próprio eixo) para a direita.
 */
void motor_rotate_right(){
	wb_motor_set_velocity(left_motor, MAX_SPEED);
	wb_motor_set_velocity(right_motor, -MAX_SPEED);
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
void rotate_to_box(const double theta, int time_step){
	if (!is_cartesian_theta_equal(theta,0)){
		double duration = abs(theta) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
	 	printf("Duração para chegar ao destino: %4.4f\n",duration);
		
		if(theta>0)
			motor_rotate_left();
		else if (theta < 0)
			motor_rotate_right();

		double start_time = wb_robot_get_time();
		do{
			wb_robot_step(time_step);
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
void move_forward(double distance, int time_step){
	double duration = (distance/2) / TANGENSIAL_SPEED;
 	printf("Duração até o destino: %4.4f\n", duration);

	motor_move_forward();

	double start_time = wb_robot_get_time();
	do{
		wb_robot_step(time_step);
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
             duration = distance / TANGENSIAL_SPEED;
             printf("Duração até o destino: %4.4f\n", duration);
          
             motor_move_forward();
           }
           else if (option == 2){
             duration = distance / ROBOT_ANGULAR_SPEED_IN_DEGREES;
             motor_rotate_left();
           }
           else if (option == 3){
              duration = distance / ROBOT_ANGULAR_SPEED_IN_DEGREES;
              motor_rotate_right();
           }

	double start_time = wb_robot_get_time();
	do{
		wb_robot_step(time_step);
	}while (wb_robot_get_time() < start_time + duration);
        
           //motor_stop();
}



//==========================//
//     CÓDIGO PRINCIPAL     //
//==========================//

/*
 * Retorna o tempo da simulação.
 */ 
int get_time_step(){
	static int time_step = -1;
	if (time_step == -1)
		time_step = (int)wb_robot_get_basic_time_step();
	return time_step;
}

void init(){
	time_step = get_time_step();
	
	// Nome dos sensores de proximidade
           char ps_names[8][4] ={
             "ps0", "ps1", "ps2", "ps3",
             "ps4", "ps5", "ps6", "ps7"
           };
           // Inicialização dos sensores.
           for (int i = 0; i < 8 ; i++) {
             // Salva o valor apontado no sensor de proximidade
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
    // Translação do robô (x,y,z)
    trans_field = wb_supervisor_node_get_proto_field(robot_node, "translation");
    // Rotação do robô (x,y,z,ângulo)
    rotac_field = wb_supervisor_node_get_proto_field(robot_node, "rotation");
          
}

int main(int argc, char **argv){
	wb_robot_init();
	
	init();

	int target = -1;
	
	int error_counter = 0;

	
	double *current_coord;
	double theta_destination;
	double robot_heading;
	double robot_heading_deg;

	double converted_theta_to_destination;
	double destination_distance;
	
	while(wb_robot_step(time_step) != -1){
		 // Atualiza sensores de proximidade
                       for (int i = 0; i < 8 ; i++)
                         ps_values[i] = wb_distance_sensor_get_value(ps[i]);
                        		
                       bool right_obstacle =  ps_values[0] > 140.0 ||
                                              ps_values[1] > 140.0;
                       bool left_obstacle =   ps_values[6] > 140.0 ||
                                              ps_values[7] > 140.0;
                                           
		// Atualiza o vetor de posições do robô ([0]: X, [1]: Y, [2]: Z)
                      robot_position = wb_supervisor_field_get_sf_vec3f(trans_field);	
                      // Atualiza o vetor de rotação do robô ([0]: X, [1]: Y, [2]: Z, [3]: ângulo)
                      robot_rotation = wb_supervisor_field_get_sf_rotation(rotac_field);
		// Adquire a coordenada cartesiana do robô.
		current_coord = convert_coords_to_cartesian(robot_position);

		if(target == -1){
      		           error_counter = 0;
			target = get_min_distance_box_info(current_coord, boxes, visited_boxes);
        		           printf("Entidade X | Y\nRobô: %4.4f | %4.4f\nCaixa Destino: %d \n\t %4.4f | %4.4f\n",
					current_coord[0], current_coord[1], target, boxes[target][0], boxes[target][1]
					);	
		

                  		theta_destination = calculate_destination_theta_in_degrees(current_coord, boxes[target]);
                  		robot_heading_deg = convert_heading_in_degrees(robot_rotation[3]);
                  		robot_heading = convert_heading_to_cartesian(robot_heading_deg);
                  		converted_theta_to_destination = calculate_robot_theta(robot_heading, theta_destination);
                  
                  		printf("Theta p/ Destino: %4.4f\n", converted_theta_to_destination);
                  		
                  		rotate_to_box(converted_theta_to_destination, time_step);
                  
                  		destination_distance = calculate_distance(current_coord, boxes[target]);
                  
                  		printf("Distância até o destino: %4.4f\n", destination_distance);
                  
        		           move_forward(destination_distance, time_step);
		}
		
		if(right_obstacle || left_obstacle){
          		            		  
          		  if (destination_distance <= BOX_THRESHOLD)
                          visited_boxes[target] = 1;
                        
                          
                        //motor_stop();
                        
                        if(left_obstacle && right_obstacle){
                          printf("Colisão frontal\n");
                          move_robot_special(180,2);
                        }
                        else if(left_obstacle){
                          printf("Colisão esquerda\n");
                          move_robot_special(30,3);
                        }
                        else if(right_obstacle){
                          printf("Colisão direita\n");
                          move_robot_special(30,2);
                        } 
                        target = -1;
                        move_robot_special(0.1,1);
                      }
                      
                      if (error_counter > 10){
                        printf("Recalculando rota\n");
                        target = -1;
                      }
                      
                      error_counter += 1;
                                            
	}
}
