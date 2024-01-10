#ifndef CONTROLLER_STE_CLEAN_H
#define CONTROLLER_STE_CLEAN_H




//C libraries
#include <assert.h>
#include <math.h>
#include <stdio.h> //printf
#include <stdlib.h> //srand
#include <sys/stat.h> // mkdir
#include <sys/time.h>
#include <time.h> //time
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include "PlumeModel.h"
#include "SampleBuffer.h"
#include "Position.h"


//Constant variables
#define BOUND(x, a, b) (((x)<(a))?(a):((x)>(b))?(b):(x))
#define PI 					3.14159265358979
#define TIME_STEP 			64 //adjusts the speed (ms)
#define SAFETY_MARGIN 		.2 // how far in meters are you ready to approach the walls
#define WHEEL_RADIUS	        0.021  	//K4: 0.021 		//K3: 0.0205	  // Wheel radius (meters)
#define AXLE_LENGTH             0.1054 	//K4: 0.0527×2 	//K3: 0.08841
#define ODOR_FILTER_LENGTH 	150 //~10sec //number of measurements before averaging
#define MAX_SAMPLE_NUMBER 	50 
#define N_CONCENTRATION_LEVEL 4
#define MAX_SPEED 			47.61 //K3: 35748.4
#define SENSOR_NUMBER  		8
#define MAX_NB_ROBOTS          	3
//wind tunnel boundaries
#define XMIN 3.0
#define YMIN 0.16
//#define ZMIN 0.1
#define XMAX 12.5
#define YMAX 3.8
//#define ZMAX 1.9
// min and max for each parameter
#define Q_min  1
#define Q_max  20
#define source_x_min  2.75
#define source_y_min  0
//#define source_z_min  0
#define source_x_max  12.75
#define source_y_max  4
//#define source_z_max  2
#define a_y_min  0.01
#define a_y_max  0.05
#define b_y_min  0.1
#define b_y_max  0.5
//#define a_z_min  0.01
//#define a_z_max  0.05
//#define b_z_min  0.1
//#define b_z_max  0.5
// number of bins for posterior PDF
#define Q_nbins 	10 //(Q_max - Q_min) /20
#define x_nbins 	40 //(source_x_max - source_x_min)/0.1 //0.1 m is the granuality
#define y_nbins 	20 //(source_y_max - source_y_min)/0.1 //0.1 m is the granuality
//#define z_nbins 	10 //(source_z_max - source_z_min)/0.1 //0.1 m is the granuality
#define a_y_nbins 	10 //(a_y_max - a_y_min) / 0.0025
#define b_y_nbins 	10 //(b_y_max - b_y_min) / 0.025
//#define a_z_nbins 	10 //(a_z_max - a_z_min) / 0.0025
//#define b_z_nbins 	10 //(b_z_max - b_z_min) / 0.025
#define MAX_DISTANCE 10.77 //from (2.75,0) to (12.75,4)

#define VERBOSE 		1

#define NO_COM_NO_COOR	0
#define COM_NO_COOR		1
#define COM_COOR 		2
#define SCENARIO 		NO_COM_NO_COOR

#define TIMEOUT         1000*1000 // miliseconds

extern int nbr_obs;
extern float margin;
extern float ** obstacles;
// extern float obstacles[][4];

//parameters to estimate
struct parameters_set{
	double Q;
	double source_position[2];
	double a_y;
	double b_y;
	//double a_z;
	//double b_z;
};

// simulation paramters
struct simulation_parameters{
	float likelihood_error;
	float gloabl_local_search_ratio;
	int MCMC_iteration_num;
	int MCMC_or_exhaustive_eval; //1 for MCMC, 0 for exhaustive evaluation
	int number_of_robots;
	int scenario;
	int map_nbr;
};
extern struct simulation_parameters simulation_parameters;

//states of the finite state machine
typedef enum state {
  STATE_UNKNOWN			=	-1,
  STATE_WAITFORPLUME	=   0,
  STATE_MEASURING 		=	4,
  STATE_ESTIMATING		=	1,
  STATE_UPDATEGOAL		=	2,
  STATE_MOVING    		=	3,
  STATE_FINISHED    	=	5,
  STATE_OBSTACLE_AVOIDANCE = 6,
}eState;

//finite state machine
struct FSM{
    int n_cycles;
    eState state;
    void (*hook)();
};
extern struct FSM FSM; 

//measurement history
struct history{
	int size;
	Position position[MAX_SAMPLE_NUMBER*MAX_NB_ROBOTS];
	double measurement[MAX_SAMPLE_NUMBER*MAX_NB_ROBOTS];
};

// message structure
struct message{
	int sender_ID;
	double position[3];
	double concentration;
	int termination_flag;
};

// info from other robots
struct friends{
	int ID;
	Position position;
	Position goal;
};

// functions
void state_measuring();
void state_estimating();
void state_updategoal();
void state_moving();
void state_waiting();
void state_obstacle_avoidance();

double pseudo_gaussian_plume_model(double robot_position[], double wind_intensity, struct parameters_set parameters);
double likelihood(struct history history, double *parameters, PlumeModel *model);
bool prior(double *parameters, int nbr_dimension, PlumeModel *model);

double randu();
long long current_timestamp();
double dist_to_goal(Position goal);
void goal_outside_obstacles(Position * goal);

//double target_evaluation(double position[][3]);
//void get_gps_position(Position external_gps_position);
int obstacle_or_not();
void do_odometry();


#endif