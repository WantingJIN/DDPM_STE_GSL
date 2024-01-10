#include "webots_functions.h"
#include "nvwa/debug_new.h"


//webots libraries

WbDeviceTag compass;
WbDeviceTag emitter;
WbDeviceTag receiver;
WbDeviceTag odor_tag;
WbDeviceTag wind_tag;
WbDeviceTag sensors[SENSOR_NUMBER];
WbDeviceTag infrared_sensors[12];
WbDeviceTag left_motor, right_motor;

long long start_time;

void init(){
	if(VERBOSE) printf("initialization\n");
	char* pPath;

	start_time = current_timestamp();
	
	// seed the RNG
	srand(time(NULL));
	
	//robot
	wb_robot_init();
	khepera4_drive_init();
	const char * ID_char = wb_robot_get_name();
	ID = atoi(ID_char);

	//odor and wind sensors
	char tag_name[256];
	sprintf(tag_name,"sensor_odor_%d",ID);
	odor_tag = wb_robot_get_device(tag_name);
	wb_receiver_enable(odor_tag,TIME_STEP);
	sprintf(tag_name,"sensor_wind_%d",ID);
	wind_tag = wb_robot_get_device(tag_name);
	wb_receiver_enable(wind_tag,TIME_STEP);

	//GPS
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);

    //Compass
    compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, TIME_STEP);
    
    //Wheel speeds 
    //wb_differential_wheels_enable_encoders(TIME_STEP);
    //wb_differential_wheels_set_speed(0,0);
    // get the motors and set target position to infinity (speed control)
  	left_motor = wb_robot_get_device("left wheel motor");
  	right_motor = wb_robot_get_device("right wheel motor");
  	wb_motor_set_position(left_motor, INFINITY);
  	wb_motor_set_position(right_motor, INFINITY);
  	set_wheels_speed(0,0);

    //distance sensors
    /*char sensors_name[] = "ds0";
	for (int i = 0; i < SENSOR_NUMBER; i++){
		sensors[i] = wb_robot_get_device(sensors_name);
		wb_distance_sensor_enable(sensors[i], TIME_STEP);
		(sensors_name[2])++;
	}*/
	static const char *infrared_sensors_names[SENSOR_NUMBER+4] = {
  	// turret sensors
  	"rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
  	"front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
  	// ground sensors
  	"ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
  	"ground right infrared sensor"};
  	// get and enable the infrared sensors
  	for (int i = 0; i < 12; ++i) {
    	infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
    	wb_distance_sensor_enable(infrared_sensors[i], TIME_STEP);
  	}

    //radio emitter and receiver
    emitter = wb_robot_get_device("emitter_K4");
    receiver = wb_robot_get_device("receiver_K4");
    wb_receiver_enable(receiver, TIME_STEP);

	//set the state to wait for stationary plume
	FSM.state = STATE_WAITFORPLUME;
	FSM.hook  = state_waiting;
	FSM.n_cycles = 0;

	

	//initialize the current position
	get_gps_position(&current_position);
	goal_position.copy_position(current_position);
	printf("initial goal position %f %f %f\n", goal_position.x,goal_position.y,goal_position.heading);

	//log files names
	pPath = getenv ("WB_WORKING_DIR");
    if (pPath!=NULL){
    	if(VERBOSE) printf ("The current path is: %s\n",pPath);
    	sprintf(folder_name, "%s/%lld",  pPath, current_timestamp());
    	mkdir(folder_name, 0755);
    }else{
    	sprintf(folder_name, "../../../Results/current"); //create a new folder for log files
    	mkdir(folder_name, 0755);
    	sprintf(folder_name, "../../../Results/current/%d",ID); //create a new folder for log files
    	mkdir(folder_name, 0755);
    }
    sprintf(measurements_logfile_name, "%s/measurements.csv", folder_name);
    sprintf(goalposition_logfile_name, "%s/goal_position.csv", folder_name);
    sprintf(posterior_logfile_name, "%s/posterior.csv", folder_name);
    sprintf(expected_a_posteriori_logfile_name, "%s/expected_a_posteriori.csv", folder_name);
    sprintf(max_a_posteriori_logfile_name, "%s/max_a_posteriori.csv", folder_name);
    sprintf(likelihood_logfile_name, "%s/likelihood_comparison.csv", folder_name);
    sprintf(measurement_history_logfile_name, "%s/measurement_history.csv", folder_name);
    sprintf(position_history_logfile_name, "%s/position_history.csv", folder_name);
    sprintf(posterior_pdf_3D_name, "%s/posterior_pdf_3D.csv", folder_name);
    sprintf(entropy_logfile_name, "%s/entropy.csv", folder_name);
    sprintf(config_logfile_name, "%s/config_log.csv", folder_name);
    sprintf(trajectory_logfile_name, "%s/trajectory.csv", folder_name);

	//create / reinitialize log files (might be unnecessary)
	measurements_logfile = fopen(measurements_logfile_name, "w");
    fclose(measurements_logfile);
    goalposition_logfile = fopen(goalposition_logfile_name, "w");
    fclose(goalposition_logfile);
    expected_a_posteriori_logfile = fopen(expected_a_posteriori_logfile_name, "w");
    fclose(expected_a_posteriori_logfile);
    max_a_posteriori_logfile = fopen(max_a_posteriori_logfile_name, "w");
    fclose(max_a_posteriori_logfile);
    entropy_logfile = fopen(entropy_logfile_name, "w");
    fclose(entropy_logfile);
    trajectory_logfile = fopen(trajectory_logfile_name, "w");
    fclose(trajectory_logfile);

    // get the configuration
    //default values for the simulation parameters
    simulation_parameters = (struct simulation_parameters) {30, 0.5, 1000, 1, 1, 0, 0};

	//read simulation parameters
	FILE* sim_param_file;
	int ok = 0;
	if(access("../../data/simulation_parameters/likelihood_error.txt", F_OK) != -1 ){
		sim_param_file = fopen("../../data/simulation_parameters/likelihood_error.txt", "r");
		ok = fscanf(sim_param_file, "%f", &simulation_parameters.likelihood_error);
		if(ok) printf("likelihood_error : %f\n", simulation_parameters.likelihood_error);
		fclose(sim_param_file);
	}
	if(access("../../data/simulation_parameters/gloabl_local_search_ratio.txt", F_OK) != -1 ){
		sim_param_file = fopen("../../data/simulation_parameters/gloabl_local_search_ratio.txt", "r");
		ok = fscanf(sim_param_file, "%f", &simulation_parameters.gloabl_local_search_ratio);
		if(ok) printf("gloabl_local_search_ratio : %f\n", simulation_parameters.gloabl_local_search_ratio);
		fclose(sim_param_file);
	}
	if(access("../../data/simulation_parameters/MCMC_iteration_num.txt", F_OK) != -1 ){
		sim_param_file = fopen("../../data/simulation_parameters/MCMC_iteration_num.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.MCMC_iteration_num);
		if(ok) printf("MCMC_iteration_num : %d\n", simulation_parameters.MCMC_iteration_num);
		fclose(sim_param_file);
	}
	if(access("../../data/simulation_parameters/MCMC_or_exhaustive_eval.txt", F_OK) != -1 ){
		sim_param_file = fopen("../../data/simulation_parameters/MCMC_or_exhaustive_eval.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.MCMC_or_exhaustive_eval);
		if(ok) printf("MCMC_or_exhaustive_eval : %d\n", simulation_parameters.MCMC_or_exhaustive_eval);
		fclose(sim_param_file);
	}
	if(access("../../data/simulation_parameters/number_of_robots.txt", F_OK) != -1 ){
		sim_param_file = fopen("../../data/simulation_parameters/number_of_robots.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.number_of_robots);
		if(ok) printf("number_of_robots : %d\n", simulation_parameters.number_of_robots);
		fclose(sim_param_file);
	}
	if(access("../../data/simulation_parameters/scenario.txt", F_OK) != -1 ){
		sim_param_file = fopen("../../data/simulation_parameters/scenario.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.scenario);
		if(ok) printf("scenario : %d\n", simulation_parameters.scenario);
		fclose(sim_param_file);
	}
	if(access("../../data/map_nbr.txt", F_OK) != -1 ){
		sim_param_file = fopen("../../data/map_nbr.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.map_nbr);
		if(ok) printf("scenario : %d\n", simulation_parameters.map_nbr);
		fclose(sim_param_file);
	}
	//write the simulation parameters
	sprintf(config_logfile_name, "%s/param_log.csv", folder_name);
	sim_param_file = fopen(config_logfile_name, "w");
	fprintf(sim_param_file, "likelihood_error %f\n", simulation_parameters.likelihood_error);
	fprintf(sim_param_file, "gloabl_local_search_ratio %f\n", simulation_parameters.gloabl_local_search_ratio);
	fprintf(sim_param_file, "MCMC_iteration_num %d\n", simulation_parameters.MCMC_iteration_num);
	fprintf(sim_param_file, "MCMC_or_exhaustive_eval %d\n", simulation_parameters.MCMC_or_exhaustive_eval);
	fprintf(sim_param_file, "number_of_robots %d\n", simulation_parameters.number_of_robots);
	fprintf(sim_param_file, "scenario %d\n", simulation_parameters.scenario);
	fprintf(sim_param_file, "map_nbr %d\n", simulation_parameters.map_nbr);
	fprintf(sim_param_file, "nbins %d %d %d %d %d\n", Q_nbins, x_nbins, y_nbins, a_y_nbins, b_y_nbins);
	fprintf(sim_param_file, "min_maxs %d %d %f %f %d %d %f %f %f %f\n", Q_min, Q_max, source_x_min, source_x_max, source_y_min, source_y_max, a_y_min, a_y_max, b_y_min, b_y_max);
	fclose(sim_param_file);

	if((ID-100) > simulation_parameters.number_of_robots){
		cleaning_up();
		exit(0); 
	}
}

double odor_read(){
	double odor_sample=0;
	while (wb_receiver_get_queue_length(odor_tag) > 0) {
		assert(wb_receiver_get_data_size(odor_tag) == sizeof(double));
		odor_sample = *(const double *)wb_receiver_get_data(odor_tag);
		wb_receiver_next_packet(odor_tag);
	}
	//printf("odor measurement : %f \n",odor_measurement);
	return odor_sample;
}

void wind_read(double* intensity, double* angle){
	struct vect3D{
  		double x;
  		double y;
  		double z;
	};
	struct vect3D v = {0};
	while (wb_receiver_get_queue_length(wind_tag) > 0) {
		assert(wb_receiver_get_data_size(wind_tag) == sizeof(struct vect3D));
		// not confident in the wind sensor, we round up the value to 1 digit
		v = *(const struct vect3D*)wb_receiver_get_data(wind_tag);
		wb_receiver_next_packet(wind_tag);
	}
	//convert the 3D vector from the sensor to intensity and heading
	*intensity = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	*angle = 0;
}

void get_gps_position(Position* external_gps_position) {
    const double *gps_values = NULL;
    gps_values = wb_gps_get_values(gps);
    
    external_gps_position->x = gps_values[0];// + 1500; //position in cm
    external_gps_position->y = gps_values[1];// +  200; //position in cm
    external_gps_position->heading = current_position.heading;  //just for it to have a theta to give back...

    //printf("Coord from GPS: x = %lf; y = %lf\n", external_gps_position[0], external_gps_position[1]);
}

void update_position() {
    const double *compass_heading = NULL;

	get_gps_position(&current_position);

    compass_heading = wb_compass_get_values(compass);
    current_position.heading = (-atan2(compass_heading[0], compass_heading[2])+M_PI);
}

int obstacle_or_not(){
	double sensors_value[SENSOR_NUMBER];

	for (int i = 0; i < SENSOR_NUMBER; i++){
		sensors_value[i] = wb_distance_sensor_get_value(infrared_sensors[i]);
		//if one of the sensors detect something, then obstacle
		if(sensors_value[i] > 200){
			//printf("[%d] obstacle detected..!!!!! %d %f\n", ID, i, sensors_value[i]);
			return 1;
		}
	}
	//otherwise no obstacle
	return 0;
}

void obstacle_avoidance_k3(){
	//if(VERBOSE) printf("[%d] avoiding obstacles .......\n", ID);
	double khepera3_matrix[9][2] = {{-5000,-5000},{-20000,40000},{-30000,50000},{-70000,70000},{70000,-60000},
	{50000,-40000},{40000,-20000},{-5000,-5000},{-10000,-10000}};
	//{ {-5000, -5000}, {-20000, 40000}, {-30000,50000}, {-70000, 70000}, {70000, -60000},
									//{50000, -40000}, {40000, -20000}, {-5000, -5000}, {-10000, -10000} };
	double sensors_value[SENSOR_NUMBER];
	double range = 2000;
	
	//if no obstacle,
	if(obstacle_or_not() == 0 || dist_to_goal(goal_position) <= 0.05){
		//printf("no obstacle anymore..XXXXXXX\n");
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		//back to moving
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		return;
	}

	//get the values of the sensors
	//printf("sensor values : ");
	for (int i = 0; i < SENSOR_NUMBER; i++){
		sensors_value[i] = wb_distance_sensor_get_value(infrared_sensors[i]);
		//printf("%f ", sensors_value[i]);
	}
	//printf("\n");

	//adjust the speed to avoid the obstacle
	for (int i = 0; i < 2; i++){
		//printf("i = %d\n", i);
		speed[i] = 0.0;
		//printf("speed[%d] = %f\n", i, speed[i]);
		for (int j = 0; j < SENSOR_NUMBER; j++){
			//printf("j = %d\n", j);
			speed[i] += khepera3_matrix[j][i] * (1.0 - (sensors_value[j] / range));
			//printf("speed[%d] = %f\n", i, speed[i]);
		}
		//printf("works until here\n");
		speed[i] = BOUND(speed[i], -MAX_SPEED, MAX_SPEED);
	}
	//wb_differential_wheels_set_speed(speed[0],speed[1]);
	set_wheels_speed(speed[0],speed[1]);
}

void obstacle_avoidance_k4(){
	//if(VERBOSE) printf("[%d] avoiding obstacles .......\n", ID);
	//IR sensors on K4 are respectively : {back_left, left, front_left, front, front_right, right, back_right, back}
	//double khepera4_matrix[SENSOR_NUMBER][2] = {{2,1},{3,1},{3,0},{1,1},{0,3},{1,3},{1,2},{2,2}};
	//double khepera4_matrix[SENSOR_NUMBER][2] = {{1, 1}, {1, -1}, {2, -2}, {-2, -3}, {-2, 2}, {-1, 1}, {0, 1}, {1, 1}};//{{-2, 4}, {-3, 5}, {-7, 7}, {7, -6}, {5, -4}, {4, -2}, {-0.5, -0.5}, {-0.5, -0.5}};//{{-0.5,-0.5}, {-2, 4}, {-4, 3}, {3,3}, {4, -5}, {4, -2}, {-0.5,-0.5}, {-0.5,-0.5}};
	//double khepera4_matrix[SENSOR_NUMBER][2] ={{-2, 4}, {-3, 5}, {-7, 7}, {7, -6}, {5, -4}, {4, -2}, {-0.5, -0.5}, {-0.5, -0.5}}; //from webots' breitenberg controller
	double khepera4_matrix[SENSOR_NUMBER][2] = {{0,1},{1,-1},{2,-2},{-3, -2.5},{-2,2},{-1,1},{1,0},{0,0}};
	double sensors_value[SENSOR_NUMBER];
	double range = 1024;
	
	//if no obstacle,
	if(obstacle_or_not() == 0 || dist_to_goal(goal_position) <= 0.05){
		//printf("no obstacle anymore..XXXXXXX\n");
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		//back to moving
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		return;
	}

	//get the values of the sensors
	//printf("sensor values : ");
	for (int i = 0; i < SENSOR_NUMBER; i++){
		sensors_value[i] = wb_distance_sensor_get_value(infrared_sensors[i]);
		//printf("%f ", sensors_value[i]);
	}
	//printf("\n");
	//printf("\n%f ", sensors_value[4]);

	//adjust the speed to avoid the obstacle
	for (int i = 0; i < 2; i++){
		//printf("i = %d\n", i);
		//speed[i] = 0.0;
		//printf("speed[%d] = %f\n", i, speed[i]);
		for (int j = 0; j < SENSOR_NUMBER; j++){
			//printf("j = %d\n", j);
			//speed[i] += khepera4_matrix[j][i] * sensors_value[j]/20.0;
			speed[i] += (khepera4_matrix[j][i]*-12) * (1.0 - (sensors_value[j] / range));
			//printf("speed[%d] = %f\n", i, speed[i]);
		}
		//printf("works until here\n");
		speed[i] = BOUND(speed[i], -MAX_SPEED, MAX_SPEED);
	}
	//wb_differential_wheels_set_speed(speed[0],speed[1]);
	//printf("[%d] speed final = %f %f\n", ID, speed[0], speed[1]);
	set_wheels_speed(speed[0],speed[1]);
}

void obstacle_avoidance_k4_cyrill(){ //
	//if(VERBOSE) printf("[%d] avoiding obstacles .......\n", ID);
	static double weights [SENSOR_NUMBER] = {0, 0.3, 0.3, 0.05, 0.3, 0.3, 0, 0}; 
	float speed_delta_l = 0;
    float speed_delta_r = 0;
	double sensors_value[SENSOR_NUMBER];
	double range = 1024;
	double aggressivity = 5*3;
	double threshold = 0.146*range;
	float speed_offset = 0.25*MAX_SPEED - (wb_distance_sensor_get_value(infrared_sensors[3])/range);
	
	//if no obstacle,
	if(obstacle_or_not() == 0 || dist_to_goal(goal_position) <= 0.05){
		//printf("no obstacle anymore..XXXXXXX\n");
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		//back to moving
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		return;
	}

	for (int i = 0; i < SENSOR_NUMBER; i++){
		sensors_value[i] = wb_distance_sensor_get_value(infrared_sensors[i]);
		//printf("%f ", sensors_value[i]);
	}

	
	for(int i=0; i<(SENSOR_NUMBER/2)-1; i++){
      if (sensors_value[i]*aggressivity >= threshold){
        speed_delta_l = speed_delta_l + weights[i]*(sensors_value[i]/range)*aggressivity;
      }
    }
    for(int i=SENSOR_NUMBER/2; i<SENSOR_NUMBER; i++){
      if (sensors_value[i]*aggressivity >= threshold){
        speed_delta_r = speed_delta_r + weights[i]*(sensors_value[i]/range)*aggressivity;
      }
    }

	/* Speed input for each wheel */
    float speed_left = 0.03151771*(speed_offset + speed_delta_l - speed_delta_r)*MAX_SPEED;
    float speed_right = 0.03151771*(speed_offset + speed_delta_r - speed_delta_l)*MAX_SPEED;

    set_wheels_speed(speed_left,speed_right);
}

void obstacle_avoidance_k4_fa(){
	double khepera4_matrix[8][2] = {{0,0},{1,0},{2,0},{-2, 2},{0,2},{0,1},{0,0},{0,0}};
	double sensors_value[SENSOR_NUMBER];
	//double range = 1024;
	
	//if no obstacle,
	if(obstacle_or_not() == 0 || dist_to_goal(goal_position) <= 0.05){
		set_wheels_speed(0,0);
		//back to moving
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		return;
	}

	//get the values of the sensors
	for (int i = 0; i < SENSOR_NUMBER; i++){
		sensors_value[i] = wb_distance_sensor_get_value(infrared_sensors[i]);
	}

	//adjust the speed to avoid the obstacle
	for (int i = 0; i < 2; i++){
		//speed[i] = 0.0;
		for (int j = 0; j < SENSOR_NUMBER; j++){
			speed[i] += (khepera4_matrix[j][i] * sensors_value[j]/20.0);
		}
		speed[i] = BOUND(speed[i], -MAX_SPEED, MAX_SPEED);
	}
	set_wheels_speed(speed[0],speed[1]);
}

void receive_from_friends(){
	//printf("I am receiving\n");
	const void * msg_from_friends;
	//get all the messages in the queue
	while(wb_receiver_get_queue_length(receiver)){
		//get a message
		msg_from_friends = wb_receiver_get_data(receiver);
		//convert it to a Message instance 
		Message msg(msg_from_friends);
		//msg.print_message();
		
		//if the message contains a measurement value -> put it in history
		if(msg.measurement0_goal1 == 0){
			//std::cout << "COMMUNICATION++++++++++++++++++" << std::endl;
			//put the relelvant values in history
			history.measurement[history.size] = msg.concentration;
			//memcpy(history.position[history.size], msg.position, sizeof(double)*3);
			history.position[history.size].copy_position(msg.position);
			//printf("I, robot %d, received concentration %f at (%f,%f,%f) from robot %d.\n", ID, history.measurement[history.size], history.position[history.size][0], history.position[history.size][1], history.position[history.size][2], msg.sender_ID);
			history.size++;
		}else{ //if the message contains the current position and goal of another robot -> update the friends list
			//std::cout << "JOINT PLANNING****************" << std::endl;
			//if the message is sent to my ID specifically, then it's a new goal for me!
			if(msg.receiver_ID == ID){
				//std::cout << "JOINT PLANNING****************" << std::endl;
				//std::cout << "Someone swapped with me" << std::endl;
				//memcpy(goal_position, msg.goal, sizeof(double)*3);
				goal_position.copy_position(msg.goal);
				printf("[%d] my goal has changed to %f %f %f \n", ID, goal_position.x, goal_position.y, goal_position.heading);
				//inform the supervisor again to move the target to the new place
				wb_emitter_set_channel(emitter, 600);
    			Message msg(ID, 0, goal_position, -1, goal_position, 0);
    			wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
			}else {
				for(int i=0; i<simulation_parameters.number_of_robots; i++){
					//std::cout << "received news from friends" << std::endl;
					if(friends[i].ID == 0) friends[i].ID = msg.sender_ID;
					if(friends[i].ID == msg.sender_ID){
						//printf("message goal %f %f %f\n", msg.goal[0], msg.goal[1], msg.goal[2]);
						//memcpy(friends[i].position, msg.position, sizeof(double)*3);
						friends[i].position.copy_position(msg.position);
						//memcpy(friends[i].goal, msg.goal, sizeof(double)*3);
						friends[i].goal.copy_position(msg.goal);
						//printf("[%d] : robot %d is at %f %f %f and goes to %f %f %f\n", ID, friends[i].ID, friends[i].position[0],friends[i].position[1],friends[i].position[2],friends[i].goal[0],friends[i].goal[1],friends[i].goal[2]);
						break;
					}
					//printf("[%d] : robot %d is at %f %f %f and goes to %f %f %f\n", ID, friends[i].ID, friends[i].position[0],friends[i].position[1],friends[i].position[2],friends[i].goal[0],friends[i].goal[1],friends[i].goal[2]);
				}
			}
		}
		//delete the head packet so the next packet in the queue, if any, becomes the new head packet
		wb_receiver_next_packet(receiver);
	}
}

double get_exp_time(){
	return wb_robot_get_time();
}

void send_msg(Message msg){
	//set the channel
	if(msg.receiver_ID == 0) //msg is meant for supervisor
		wb_emitter_set_channel(emitter, 600);
	else	//for other robots
		wb_emitter_set_channel(emitter, 601);
	//send
	wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
}

void set_wheels_speed(double speed1, double speed2){
	//wb_differential_wheels_set_speed(speed1,speed2);
	wb_motor_set_velocity(left_motor, speed1);
    wb_motor_set_velocity(right_motor, speed2);
}

void cleaning_up(){
	int systemRet = 0;
	char file_name[256];
	FILE * ack_file;
	char * pPath;

	// end of program notif for cluster
	pPath = getenv ("WB_WORKING_DIR");
    if (pPath!=NULL){
    	sprintf(file_name, "%s/ack.txt",  pPath);
    	ack_file = fopen(file_name, "w");
    	fclose(ack_file);
    }else{ //for local PC
    	if(access("/home/rahbar/bell.wav", F_OK) != -1 )
			//systemRet = system("aplay -q /home/rahbar/bell.wav");
		if(systemRet == -1)
			printf("end of program notification didn't work.\n");
    }
    //disable the sensors
    wb_receiver_disable(odor_tag);
    wb_receiver_disable(wind_tag);
    
    //ask the supervisor to quit the simulation
    //wb_emitter_set_channel(emitter, 600);
    //goal_position.x = -1; goal_position.y = -1; goal_position.heading = -1;
    goal_position.set_to(-1, -1, -1);
    Message msg(ID, 0, goal_position, -1, goal_position, 0);
    //wb_emitter_send(emitter, msg.buffer, msg.buffer_size); 
    send_msg(msg);   

    //delete plume_model;

    wb_robot_cleanup();
}

void next_iteration_time(){
	wb_robot_step(TIME_STEP);
	//check for timeout
	// if(current_timestamp()-start_time > TIMEOUT)
	// 	FSM.state = STATE_FINISHED;
}