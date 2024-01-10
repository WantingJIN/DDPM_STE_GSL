/*
* File:          controller_STE_multi-robot.cpp
* Date:          Jan 2019
* Description:   The goal is to estimate the source terms including the position
* Author:        Faezeh Rahbar
* Author:        Wanting Jin
* Modifications: 01/09/2022
*/

//custom libraries
#include <fstream>
#include <iostream>
#include "controller_STE_multi-robot.h"
#include "nvwa/debug_new.h"
#include "Posterior.h"
#include "Message.h"
#include "webots_functions.h"
#include "aStar.h"
#include "visilibity.hpp"

#include <webots/supervisor.h>

using namespace std;

//Global Variables

// log files
char folder_name[256];
FILE* goalposition_logfile;
FILE* measurements_logfile;
FILE* posterior_logfile;
FILE* posteriorpdf_logfile;
FILE* expected_a_posteriori_logfile;
FILE* likelihood_logfile;
FILE* measurement_history_logfile;
FILE* position_history_logfile;
FILE* max_a_posteriori_logfile;
FILE* posterior_pdf_3D;
FILE* entropy_logfile;
FILE* tmp_file;
FILE* config_logfile;
FILE* trajectory_logfile;
char goalposition_logfile_name[256];
char measurements_logfile_name[256];
char posterior_logfile_name[256];
char posteriorpdf_logfile_name[256];
char expected_a_posteriori_logfile_name[256];
char max_a_posteriori_logfile_name[256];
char likelihood_logfile_name[256];
char measurement_history_logfile_name[256];
char position_history_logfile_name[256];
char posterior_pdf_3D_name[256];
char entropy_logfile_name[256];
char config_logfile_name[256];
char trajectory_logfile_name[256];

// finite state machine
struct FSM FSM;

// robot's measurements
SampleBuffer sampleBuffer(ODOR_FILTER_LENGTH);

//position in m
Position goal_position;
Position current_position;
double speed[2] = {0};
// robot's position and measurement history
struct history history;

//plume model
DeepCFDPlumeModel2D plume_model;

// current posterior distribution 
Posterior posterior(&plume_model);

//robot's ID
int ID;

//message structure
struct message message;

//list of other robots
struct friends friends[MAX_NB_ROBOTS];

int n_time_step = 0;

struct simulation_parameters simulation_parameters;

// obstacles definitions
int nbr_obs = 4;
float margin = 0.16;
float ** obstacles = NULL;
// float obstacles[][4] = {{5-margin, static_cast<float>(5.5+margin), static_cast<float>(2.5-margin), 4+margin}, 
							// {5-margin, 8+margin, static_cast<float>(2.5-margin), 3+margin}, 
							// {9-margin, 12+margin, 1-margin, static_cast<float>(1.5+margin)}, 
							// {static_cast<float>(11.5-margin), 12+margin, 0-margin, static_cast<float>(1.5+margin)}};
std::list<Position> path;
std::list<Position>::iterator i;

std::string environment_file("../../data/test_map.environment");
VisiLibity::Environment my_environment(environment_file);
double epsilon = 0.000000001;
VisiLibity::Visibility_Graph vg(my_environment, epsilon);

// =========================== FUNCTIONS ================================ //
//random value sampled from a nromal distribution
double randn(double mu, double sigma){
	//taken from https://phoxis.org/2013/05/04/generating-random-numbers-from-normal-distribution-in-c/
	double U1, U2, W, mult;
	static double X1, X2;
	static int call = 0;
	
	if(call == 1){
    	call = !call;
      	return (mu + sigma * (double) X2);
    }

	do{
      	U1 = -1 + ((double) rand () / RAND_MAX) * 2;
    	U2 = -1 + ((double) rand () / RAND_MAX) * 2;
      	W = pow (U1, 2) + pow (U2, 2);
    } while (W >= 1 || W == 0);
 
  	mult = sqrt ((-2 * log (W)) / W);
  	X1 = U1 * mult;
  	X2 = U2 * mult;
 
  	call = !call;
 
  	return (mu + sigma * (double) X1);
}

//random value sampled from a uniform distribution
double randu(){
	//random variable between 0 and 1
    return (double)rand() / (double)RAND_MAX ;
}

//get the current time in timestamp (ms)
long long current_timestamp(){
	struct timeval te; 
	gettimeofday(&te, NULL); // get current time
	long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
	// printf("milliseconds: %lld\n", milliseconds);
	return milliseconds;
}

//Euclidean distance between two points
double pt_distance(Position p1, Position p2){
	return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2));
}

//min distance between two robots position (p1 and p2) and their goals (g1 and g2)
double min_distance(Position p1, Position g1, Position p2, Position g2){
	double d_original = pt_distance(p1, g1) + pt_distance(p2, g2);
	double d_swap = pt_distance(p1, g2) + pt_distance(p2, g1);
	if(d_swap < d_original)
		return d_swap;
	else
		return d_original;
}

//Euclidean distance between current position to the goal position
double dist_to_goal(Position goal){
    double dx, dy;
    dx = goal.x - current_position.x;
    dy = goal.y - current_position.y;
    return sqrt(dx*dx + dy*dy);
}

//calculate wheel speeds to go toward goal position
void go_to_goal(Position goal){
	// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, goal.x,goal.y);
	//vector to the goal
    double movement[2];
    movement[0] = (goal.x - current_position.x);
    movement[1] = (goal.y - current_position.y);
    //printf("movement : %f %f \n", movement[0], movement[1]);

    //vector to the goal in robot coordinates
    float x =  movement[0]*cosf(current_position.heading) + movement[1]*sinf(current_position.heading); // x in robot coordinates
   
    float z = -movement[0]*sinf(current_position.heading) + movement[1]*cosf(current_position.heading); // z in robot coordinates
 
    float bearing = atan2(z, x) - PI/2;    //[-PI, PI]
    //printf("bearing : %f \n", bearing/PI*180);

    //angular and linear velocity
    float w = 1 * sinf(bearing);
    float u = 0.2 * cosf(bearing);
    // printf("linear v : %f; angular w : %f \n", u, w);
    // wheels speed
	speed[0] = (u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;   //10 is a speed-up factor
	speed[1] = (u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
	//printf("speed : %f %f \n", speed[0], speed[1]);
}

//show the measurement history on screen
void history_print(struct history history){
	printf("_____ Sample History _____ \n");
	for(int i=0; i<history.size; i++){
		printf("%d : %f @ (%f, %f)\n", i, history.measurement[i], history.position[i].x, history.position[i].y);
	}
	printf("__________________________ \n");
}

void read_obstacle_list(){
	// open file
	std::ifstream infile("../../data/obstacle_list.txt");
	// first line is the number of obstacles
	infile >> nbr_obs;
	printf("There is %d obsatcle in total:\n",nbr_obs);
	// allocate memory for obstacle list array
	obstacles = new float*[nbr_obs];
	//read all obstacles now
	for(int i = 0; i < nbr_obs; i++){
		obstacles[i] = new float[4];
		infile >> obstacles[i][0] >> obstacles[i][1] >> obstacles[i][2] >> obstacles[i][3];
		obstacles[i][0] -= margin;
		obstacles[i][1] -= margin;
		obstacles[i][2] += margin;
		obstacles[i][3] += margin;
	}

	for(int i = 0; i < nbr_obs; i++){
		printf("obstacle %d: %f %f %f %f\n", i, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
	}
}

// ===================== LOG FILES FUNCTIONS ========================== //

void log_position(){
	double time = wb_robot_get_time();
	goalposition_logfile = fopen(goalposition_logfile_name, "a");
	fprintf(goalposition_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, current_position.x, current_position.y, current_position.heading, time);
	fclose(goalposition_logfile);
}

void log_measurements(){
	//int i = 0;
	double time = wb_robot_get_time();
	measurements_logfile = fopen(measurements_logfile_name, "a");
	fprintf(measurements_logfile,"%d, %f, %f, %f\n", FSM.n_cycles, sampleBuffer.average_wind_intensity, sampleBuffer.average_concentration, time);
	//for (i = 0; i < ODOR_FILTER_LENGTH; i++)
	//	fprintf(measurements_logfile,"%f, ", measurements.odor_sample_memory.buffer[i]);
	//fprintf(measurements_logfile,"\n");
	fclose(measurements_logfile);
}

void log_expected_a_posteriori(){
	expected_a_posteriori_logfile = fopen(expected_a_posteriori_logfile_name, "a");
	fprintf(expected_a_posteriori_logfile,"%d, %f, %f, %f, %f, %f\n", FSM.n_cycles, 
		plume_model.expected_value[0], plume_model.expected_value[1], plume_model.expected_value[2], 
		plume_model.expected_value[3], plume_model.expected_value[4]);
	fclose(expected_a_posteriori_logfile);
}

void log_max_a_posteriori(){
	max_a_posteriori_logfile = fopen(max_a_posteriori_logfile_name, "a");
	fprintf(max_a_posteriori_logfile,"%d, %f, %f, %f, %f, %f\n", FSM.n_cycles,
		plume_model.max_a_post_value[0], plume_model.max_a_post_value[1], plume_model.max_a_post_value[2], 
		plume_model.max_a_post_value[3], plume_model.max_a_post_value[4]);
	fclose(max_a_posteriori_logfile);
}

void log_entropy(){
	double time = get_exp_time();
	entropy_logfile = fopen(entropy_logfile_name, "a");
	fprintf(entropy_logfile, "%f, %f\n", posterior.entropy, time);
	fclose(entropy_logfile);
}

void log_history(){
	// position
	//FILE* position_history_logfile;
	position_history_logfile = fopen(position_history_logfile_name, "w");
	for (int i = 0; i < history.size; i++){
		fprintf(position_history_logfile,"%d, %f, %f, %f\n", i, history.position[i].x,history.position[i].y,history.position[i].heading);
	}
    fclose(position_history_logfile);
    // measurement
    //FILE * measurement_history_logfile;
    measurement_history_logfile = fopen(measurement_history_logfile_name, "w");
	for (int i = 0; i < history.size; i++){
		fprintf(measurement_history_logfile,"%d, %f\n", i, history.measurement[i]);
	}
    fclose(measurement_history_logfile);
   
	//posterior 3D
	posterior.log_pdf(posterior_pdf_3D_name, 'w');
}

void log_trajectory(Position p){
	double time = wb_robot_get_time();
	trajectory_logfile = fopen(trajectory_logfile_name, "a");
	fprintf(trajectory_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, p.x, p.y, p.heading, time);
	fclose(trajectory_logfile);
}


// ====================== ESTIMATIOM functions ========================= //

//ASSUMPTION : Using the history of measurements, but only one robot is operating
double likelihood(struct history history, double *parameters, PlumeModel *model){ 
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; //125;
	double sum_tmp = 0;
	if(sampleBuffer.average_wind_intensity == 0) printf("NO WIND, BYE BYE!\n");

	// getting all model concentrations for all sample positions and then calculate the likelihood
	// printf("likelihood... history size %d\n", history.size);
	vector<vector<float>> model_c_avg(history.size, vector<float> (1)); 
	vector<vector<float>> model_c_std(history.size, vector<float> (1)); 
	model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, parameters, model_c_avg, model_c_std);
	for (int i = 0; i < history.size; i++)
		sum_tmp += pow(history.measurement[i] - model_c_avg[i][0], 2) / (sigmaE*sigmaE + sigmaD*sigmaD);
	
	return (exp(-0.5 * sum_tmp));
}

bool prior(double *parameters, int nbr_dimension, PlumeModel *model){
	int i = 0;
	bool output = 1;
	
	// verify if all the parameters are between the right intervals
	for(i = 0; i < nbr_dimension; i++)
		output = output && (parameters[i] < model->param_list[i].upper_limit && parameters[i] >= model->param_list[i].lower_limit);
	
	// verify the x and y elements of the parameters are not inside obstacles
	Position tmp_p(parameters[0], parameters[1], 0);
	if(tmp_p.is_in_obstacle_list(obstacles, nbr_obs) != -1){
		// printf("point %f %f is in obstacle\n", parameters[0], parameters[1]);
		output = 0;
	}

	//if(output == 0)
	//	printf("parameters: %f %f %f %f %f\n", parameters[0], parameters[1], parameters[2], parameters[3], parameters[4]);
	
	return(output);
}


// ======================= NAVIGATION functions ========================= //
void random_navigation(){
	goal_position.x = (randu() * (XMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;  //position in m
    goal_position.y = (randu() * (YMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;
    goal_position.heading = 0;//(randu() * (ZMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;
}

bool position_outside_arena(Position p){
	bool outside = 0;
	if(p.x < XMIN || p.x > XMAX) outside = 1;
	if(p.y < YMIN || p.y > YMAX) outside = 1;
	//if(position[2] < ZMIN || position[2] > ZMAX) outside = 1;
	return outside;
}

void information_gain_navigation_approx(Position current_position){
	float p0 = 0.5; //50 cm
	double DKL=0, max_DKL=-100;
	//double position[1][3] = {{0}};
	Position tmp_position;
	struct history fake_history;
	memcpy(fake_history.position, history.position, (sizeof(Position))*history.size);
	memcpy(fake_history.measurement, history.measurement, (sizeof(double))*history.size);
	fake_history.size = history.size;

	// add fake prediction for friends goal position, so we don't choose the same ones
	for(int i = 0; i < simulation_parameters.number_of_robots; i++){
		if(friends[i].ID == 0 || friends[i].ID  == ID) continue;
		fake_history.position[fake_history.size].copy_position(friends[i].goal);
		fake_history.measurement[fake_history.size] = plume_model.approx_concentration(friends[i].goal, sampleBuffer.average_wind_intensity);
		(fake_history.size)++;
	}

	for(int i =- 1; i <= 1; i++){
		for (int j = -1; j <= 1; j++){
			// for each possible action
			tmp_position.x = current_position.x + i*p0;
			tmp_position.y = current_position.y + j*p0;
			tmp_position.heading = 0;
			// printf("position : %f %f\n", tmp_position.x, tmp_position.y);
			//printf("position : %f %f %f\n", j*p0 * cos(l*theta0) * sin(k*phi0),j*p0 * sin(l*theta0) * sin(k*phi0),j*p0 * cos(k*phi0));
			if(position_outside_arena(tmp_position)) continue;
			//calculate the expected concentration on position
			fake_history.measurement[fake_history.size] = plume_model.approx_concentration(tmp_position, sampleBuffer.average_wind_intensity);
			fake_history.position[fake_history.size].copy_position(tmp_position);
			(fake_history.size)++;
			DeepCFDPlumeModel2D plume_model_fake;
			Posterior future_posterior(&plume_model_fake, 0);
			//future_posterior.copy_posterior(zero);
			future_posterior.MCMC_evaluation(fake_history);
			//calculate the expected information gain
			DKL = posterior.Kullback_Leibler_divergence(future_posterior);
			// printf("DKL: %f _______________________\n", DKL);
			if(DKL > max_DKL ){
				max_DKL = DKL;
				goal_position.x = tmp_position.x; // + (randu()-0.5)/5.0
				goal_position.y = tmp_position.y;
				goal_position.heading = 0;
			}
		}
	}
}


void balance_exploration_exploitation(float gloabl_local_search_ratio){
	float ratio = gloabl_local_search_ratio;//simulation_parameters.gloabl_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
	float vec1[2] = {0}, vec2[2] = {0};
	printf("current_position : %f %f , goal position : %f %f , estimated source : %f %f \n", current_position.x,current_position.y,
		goal_position.x,goal_position.y, plume_model.max_a_post_value[0],plume_model.max_a_post_value[1]);
	vec1[0] = goal_position.x - current_position.x; //exploration
	vec1[1] = goal_position.y - current_position.y;

	vec2[0] = plume_model.max_a_post_value[0] - current_position.x; //TODO: to improve
	vec2[1] = plume_model.max_a_post_value[1] - current_position.y;

	goal_position.x = current_position.x + ratio*vec1[0] + (1-ratio)*vec2[0];
	goal_position.y = current_position.y + ratio*vec1[1] + (1-ratio)*vec2[1];
	goal_position.heading = 0;//current_position[2] + ratio*vec1[2] + (1-ratio)*vec2[2];
	//printf("new goal position : %f %f %f\n", goal_position[0],goal_position[1],goal_position[2]);
}


void DKL_MAP_distance(float gloabl_local_search_ratio){
	double best_point[2]={0};
	posterior.XYmarginal_distance_combined(current_position, gloabl_local_search_ratio, best_point);
	printf("best point %f %f\n", best_point[0], best_point[1]);

	float ratio = 0.3;//gloabl_local_search_ratio;//simulation_parameters.gloabl_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
	float vec1[2] = {0}, vec2[2] = {0};
	//printf("current_position : %f %f %f, goal position : %f %f %f, estimated source : %f %f %f\n", current_position[0],current_position[1],current_position[2],
	//	goal_position[0],goal_position[1],goal_position[2], estimated_source_parameters.source_position[0],estimated_source_parameters.source_position[1],estimated_source_parameters.source_position[2]);
	vec1[0] = goal_position.x - current_position.x; //exploration
	vec1[1] = goal_position.y - current_position.y;
 	//vec1[2] = goal_position[2] - current_position[2]; 
	vec2[0] = best_point[0] - current_position.x; //exploitation (source) //estimated_source_parameters //max_a_post_source_parameters
	vec2[1] = best_point[1] - current_position.y;
	//vec2[2] = estimated_source_parameters.source_position[2] - current_position[2];
	goal_position.x = current_position.x + ratio*vec1[0] + (1-ratio)*vec2[0];
	goal_position.y = current_position.y + ratio*vec1[1] + (1-ratio)*vec2[1];
	goal_position.heading = 0;//current_position[2] + ratio*vec1[2] + (1-ratio)*vec2[2];
}

void goal_outside_obstacles(Position * goal){
	// is the goal position in an obstacle?
	int idx = goal->is_in_obstacle_list(obstacles, nbr_obs);
	// if not, then back to business
	if(idx == -1 && !position_outside_arena(*goal)){
		printf(">>> the goal is not in any obstacles or outside of the region\n");
		return;
	}

	//set concentration to 0 in the original goal position
	history.position[history.size].x = goal->x;
	history.position[history.size].y = goal->y;
	history.position[history.size].heading = 0;
	history.measurement[history.size] = 0;
	history.size++;
    
	if (idx != -1){
		printf(">>> ! position %f %f is in an obstacle\n", goal->x, goal->y);
		// otherwise, lets try 4 different candidates
		float min_dist = 10000.0;
		Position nearest_position(*goal);
		Position candidates[4] = {	Position(obstacles[idx][0], goal->y, 0), 
									Position(obstacles[idx][2], goal->y, 0),
									Position(goal->x, obstacles[idx][1], 0),
									Position(goal->x, obstacles[idx][3], 0)};
		// for each candidate position
		for(int i=0; i<4; i++){
			//check if the distance to candidate is smaller than others
			// printf(">>> distance from candidate %f %f = %f\n", candidates[i].x, candidates[i].y, goal->distance_from(candidates[i]));
			if(goal->distance_from(candidates[i]) < min_dist){
				if(candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i])){
					//if so, then this is the nearest position
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
					// printf(">>> chose it as new position !!! \n");
				}
			}
		}
		//change the goal position using the pointer
	    printf(">>> Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
	}

    if (position_outside_arena(*goal)){
		printf(">>> ! position %f %f is outside the region\n", goal->x, goal->y);
		// otherwise, lets try 4 different candidates
		float min_dist = 10000.0;
		Position nearest_position(*goal);
		Position candidates[4] = {	Position(XMIN, goal->y, 0), 
									Position(XMAX, goal->y, 0),
									Position(goal->x, YMIN, 0),
									Position(goal->x, YMAX, 0)};
		// for each candidate position
		for(int i=0; i<4; i++){
			//check if the distance to candidate is smaller than others
			// printf(">>> distance from candidate %f %f = %f\n", candidates[i].x, candidates[i].y, goal->distance_from(candidates[i]));
			if(goal->distance_from(candidates[i]) < min_dist){
				if(candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i])){
					//if so, then this is the nearest position
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
					// printf(">>> chose it as new position !!! \n");
				}
			}
		}
		//change the goal position using the pointer
		printf(">>> Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
	}
	
}

// ============================== STATES ================================= //
void state_waiting(){
	// initial waiting for the plume to get established
	static int counter = 0;
	counter++;

	if(counter > 300){
		FSM.state = STATE_MEASURING;
		FSM.hook  = state_measuring;
	}
}

void state_measuring(){
	//printf("measuring sample number %d\n", measurements.n_samples);
	if(sampleBuffer.n_samples < ODOR_FILTER_LENGTH){
		//continue measuring odor concentration
        sampleBuffer.instant_concentration = odor_read();
		//measuring wind
		wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle);
		//add samples to buffers
		sampleBuffer.add_all_to_buffer();
		//printf("n %d wind intensity: %f, angle %f \n", sampleBuffer.n_samples, sampleBuffer.instant_wind_intensity, sampleBuffer.instant_wind_angle);
	}else{
		//average all samples
		sampleBuffer.average_all();
		//log and save
		if(VERBOSE) printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
		log_position();
		log_measurements();
		history.position[history.size].x = current_position.x;
		history.position[history.size].y = current_position.y;
		history.position[history.size].heading = 0;
		history.measurement[history.size] = sampleBuffer.average_concentration;
		history.size++;
		history_print(history);

		// send it to friends
		if(simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR){
			Message msg(ID, 1, history.position[(history.size)-1], sampleBuffer.average_concentration, goal_position, 0);
    		send_msg(msg);
    	}
		char label[255];
        sprintf(label,"Iteration: %d\n",history.size);
        wb_supervisor_set_label(2,label,0.1,0.8,0.1,0x61A8E8,0,"Impact");
	    char label2[255];
        sprintf(label2,"Reading: %.3f\n",sampleBuffer.average_concentration);
        wb_supervisor_set_label(1,label2,0.1,0.85,0.1,0xFF0000,0,"Impact");

		// go to estimation
		FSM.state = STATE_ESTIMATING;
		FSM.hook  = state_estimating;
	}
}

void state_estimating(){
	if(VERBOSE) printf("[%d] estimating...\n", ID);
	//reinitialize
	posterior.pdf_reset();
	printf("going for MCMC\n");
	//estimation
	if(simulation_parameters.MCMC_or_exhaustive_eval)
		posterior.MCMC_evaluation(history);
	else
		posterior.exhaustive_evaluation(history);
	if(posterior.dirac_flag){
		if(VERBOSE) printf("[%d] exhaustive search!\n", ID);
		posterior.exhaustive_evaluation(history);
	}
	
	printf("sum posterior : %f \n", posterior.sum);
	printf("Before process. \n");
	posterior.process();
	printf("After Process. Start logging.\n");
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();
	printf("Finish logging.\n");
	posterior.log_pdf("/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/current/101/posterior.csv",'a');

	FSM.n_cycles++;
	if(VERBOSE) printf("[%d] cycle number %d\n", ID, FSM.n_cycles);
	if(VERBOSE) printf("[%d] entropy : %f\n", ID, posterior.entropy);
	
	//check if time is up
	//if(FSM.n_cycles == MAX_SAMPLE_NUMBER || posterior.entropy < 0.1 || posterior.dirac_flag){
	if(FSM.n_cycles == MAX_SAMPLE_NUMBER || posterior.entropy < 0.1){
		//the end
		FSM.state = STATE_FINISHED;
		printf("Current state is STATE_FINISHED");
	}else{
		//update goal
		FSM.state = STATE_UPDATEGOAL;
		FSM.hook  = state_updategoal;
	}
}

void state_updategoal(){
	if(VERBOSE) printf("[%d] updating the goal\n", ID);

	//find the best point to move to 
	
	information_gain_navigation_approx(current_position);
	// printf("goal position here 1 %f %f\n", goal_position.x, goal_position.y);
    		
    DKL_MAP_distance(0.75);

    		
    goal_outside_obstacles(&goal_position);
    // printf("goal position here %f %f 3\n", goal_position.x, goal_position.y);
    //heterogeneous_navigation();

    printf("from %f %f to goal %f %f \n", current_position.x, current_position.y, goal_position.x, goal_position.y);
    VisiLibity::Point start(current_position.x, current_position.y);
    VisiLibity::Point finish(goal_position.x, goal_position.y);
    VisiLibity::Polyline Polyline_path = my_environment.shortest_path(start, finish, vg, epsilon);
    // copy the path in another variable
    for(unsigned k=0; k<Polyline_path.size(); k++){
    	Position p(Polyline_path[k].x(), Polyline_path[k].y(),0);
        path.push_back(p);
        log_trajectory(p);
    }
    i = path.begin();
    printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
    //send the goal to the supervisor
    Message msg(ID, 0, goal_position, -1, goal_position, 0);
    send_msg(msg);
	//move to the goal position
	FSM.state = STATE_MOVING;
	FSM.hook = state_moving;
}

void state_moving(){
	//printf("moving toward the goal\n");

	//verify if arrived to the goal if applicable
	if(dist_to_goal(goal_position) > 0.05){
		//move
		if(dist_to_goal(*i) <= 0.05 && i != path.end()){
			i++;
			printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
		}
		go_to_goal(*i);
		set_wheels_speed(speed[0],speed[1]);
		//log the position
		//log_position();
	}else{
		set_wheels_speed(0,0);
		sampleBuffer.reset_all();
		path.clear();
		//back to measuring
		FSM.state = STATE_MEASURING;
		FSM.hook  = state_measuring;
	}
}

void state_obstacle_avoidance(){
	//printf("avoiding obstacle\n");
	//obstacle_avoidance_k3();
	obstacle_avoidance_k4();
}


// =========================== MAIN ============================== //


int main(int argc, char **argv){
	// python instance (should be called ONLY ONCE and only in main)
	CPyInstance hInstance;
	init();
	read_obstacle_list();

	//visibility graph (visilibity library)
	std::cout << "Validating environment model . . . ";
  	if(  my_environment.is_valid( epsilon )  )
    	std::cout << "OK" << std::endl;
  	else{
    	std::cout << std::endl << "Warning:  Environment model "
	    << "is invalid." << std::endl
	    << "A valid environment model must have" << std::endl
	    << "   1) outer boundary and holes pairwise "
	    << "epsilon -disjoint simple polygons" << std::endl
	    << "   (no two features should come "
	    << "within epsilon of each other)," << std::endl 
	    << "   2) outer boundary is oriented ccw, and" 
	    << std::endl
	    << "   3) holes are oriented cw."
	    << std::endl;
    	exit(0);
  	}
  	VisiLibity::Visibility_Graph vg(my_environment, epsilon);
	char label[255];
    sprintf(label,"Iteration: %d\n",0);
    wb_supervisor_set_label(2,label,0.1,0.8,0.1,0x61A8E8,0,"Impact");
	char label2[255];
    sprintf(label2,"Reading: %d\n",0);
    wb_supervisor_set_label(1,label2,0.1,0.85,0.1,0xFF0000,0,"Impact");

	// //main loop
	while (FSM.state != STATE_FINISHED){
		update_position();
		//if we are not stopped to sample, let's log the trajectory
		/*if(speed[0]!=0 && speed[1]!=0){
			log_trajectory();
		}*/
		//send info (current position and goal position) to friends for joint planning
		if(simulation_parameters.scenario == COM_COOR){
			// //if a robot is already sampling (i.e. STATE_MEASURING), its goal should not be swapped with anyone. Therefore we send 0 as goal and current position
			if(FSM.state == STATE_MEASURING){
				Position tmp;
				Message msg(ID, 1, tmp, sampleBuffer.average_concentration, tmp, 1);
				send_msg(msg);
			}else{
				Message msg(ID, 1, current_position, sampleBuffer.average_concentration, goal_position, 1);
				//printf("[%d] I am sendig my position %f %f %f and my goal %f %f %f\n", msg.sender_ID, msg.position[0],msg.position[1], msg.position[2], msg.goal[0],msg.goal[1],msg.goal[2]);
    			send_msg(msg);
    		}
    	}
		//receive info from colleagues for joint planning and history
		if(simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR){
			//printf("Robot %d has %d unread messages! \n", ID, wb_receiver_get_queue_length(receiver));
			receive_from_friends();
			//printf("[%d] my history size is %d\n", ID, history.size);
		}
	   	//change state
    	FSM.hook();
    	//next
    	next_iteration_time();
    	n_time_step++;
	}
	// //final logs
	log_history();
	cleaning_up();
    //free the allocated memory to obstacle array
    for(int i = 0; i < nbr_obs; i++)
    	delete[] obstacles[i];
	delete[] obstacles;
	exit(0);
}

