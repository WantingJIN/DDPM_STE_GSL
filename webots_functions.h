#ifndef WEBOTS_HEADER_H
#define WEBOTS_HEADER_H

//webots libraries
#include <webots/robot.h>
//#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

extern "C" {
	//#include "odor_filter.h"
	//#include "odometry_track.h"
	//#include "vector3d.h"
	#include "khepera4.h"
	#include "khepera4_drive.h"
}
#include "controller_STE_multi-robot.h"
#include "Message.h"



#define WEBOTS 1
#define KHEPERA 0

extern WbDeviceTag gps;
extern WbDeviceTag compass;
extern WbDeviceTag emitter;
extern WbDeviceTag receiver;
// this supervisor gets the odor and wind readings through the radio and these tags are radio tags
extern WbDeviceTag odor_tag;
extern WbDeviceTag wind_tag;
extern WbDeviceTag sensors[SENSOR_NUMBER];

//Global Variables
extern struct sObject robot;
extern struct sObject source;
extern struct sOdometryTrack ot;

// log files
extern char folder_name[256];
extern FILE* goalposition_logfile;
extern FILE* measurements_logfile;
extern FILE* posterior_logfile;
extern FILE* posteriorpdf_logfile;
extern FILE* expected_a_posteriori_logfile;
extern FILE* likelihood_logfile;
extern FILE* measurement_history_logfile;
extern FILE* position_history_logfile;
extern FILE* max_a_posteriori_logfile;
extern FILE* posterior_pdf_3D;
extern FILE* entropy_logfile;
extern FILE* tmp_file;
extern FILE* config_logfile;
extern FILE* trajectory_logfile;
extern char goalposition_logfile_name[256];
extern char measurements_logfile_name[256];
extern char posterior_logfile_name[256];
extern char posteriorpdf_logfile_name[256];
extern char expected_a_posteriori_logfile_name[256];
extern char max_a_posteriori_logfile_name[256];
extern char likelihood_logfile_name[256];
extern char measurement_history_logfile_name[256];
extern char position_history_logfile_name[256];
extern char posterior_pdf_3D_name[256];
extern char entropy_logfile_name[256];
extern char config_logfile_name[256];
extern char trajectory_logfile_name[256];

// finite state machine
extern struct FSM FSM;
// robot's measurements
extern struct measurements measurements;

//extern double goal_position[3]; //position in m
extern Position goal_position;
//extern double current_position[3]; //position in m
extern Position current_position;
extern double speed[2];
// robot's position and measurement history
extern struct history history;

//robot's ID
extern int ID;

//message structure
extern struct message message;

//list of other robots
extern struct friends friends[MAX_NB_ROBOTS];

extern int n_time_step;

extern struct simulation_parameters simulation_parameters;

void init();
double odor_read();
void wind_read(double* intensity, double* angle);
void get_gps_position(Position* external_gps_position);
void update_position();
int obstacle_or_not();
void receive_from_friends();
double get_exp_time();
void send_msg(Message msg); 
void set_wheels_speed(double speed1, double speed2);
void cleaning_up();
void next_iteration_time();
void obstacle_avoidance_k3();
void obstacle_avoidance_k4();
void obstacle_avoidance_k4_fa();

#endif