#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nvwa/debug_new.h"
#include "SampleBuffer.h"
using namespace std;


SampleBuffer::SampleBuffer(){
	instant_concentration = 0;
	average_concentration = 0;
	instant_wind_intensity = 0;
	average_wind_intensity = 0;
	instant_wind_angle = 0;
	average_wind_angle = 0;
	
	n_samples = 0;

	buffer_length = 10; //default value when no length mentioned

	concentration_buffer = (double*) calloc(buffer_length, sizeof(double));
	wind_intensity_buffer = (double*) calloc(buffer_length, sizeof(double));
	wind_angle_buffer = (double*) calloc(buffer_length, sizeof(double));
}

SampleBuffer::SampleBuffer(int length){
	instant_concentration = 0;
	average_concentration = 0;
	instant_wind_intensity = 0;
	average_wind_intensity = 0;
	instant_wind_angle = 0;
	average_wind_angle = 0;
	
	n_samples = 0;

	buffer_length = length;

	concentration_buffer = (double*) calloc(buffer_length, sizeof(double));
	wind_intensity_buffer = (double*) calloc(buffer_length, sizeof(double));
	wind_angle_buffer = (double*) calloc(buffer_length, sizeof(double));
}

SampleBuffer::SampleBuffer(const SampleBuffer & to_copy_from){
	int i=0;
	instant_concentration = to_copy_from.instant_concentration;
	average_concentration = to_copy_from.average_concentration;
	instant_wind_intensity = to_copy_from.instant_wind_intensity;
	average_wind_intensity = to_copy_from.average_wind_intensity;
	instant_wind_angle = to_copy_from.instant_wind_angle;
	average_wind_angle = to_copy_from.average_wind_angle;
	
	n_samples = to_copy_from.n_samples;

	buffer_length = to_copy_from.buffer_length;

	concentration_buffer = (double*) calloc(buffer_length, sizeof(double));
	for(i = 0; i < buffer_length; i++)
		concentration_buffer[i] = to_copy_from.concentration_buffer[i];

	wind_intensity_buffer = (double*) calloc(buffer_length, sizeof(double));
	for(i = 0; i < buffer_length; i++)
		wind_intensity_buffer[i] = to_copy_from.wind_intensity_buffer[i];

	wind_angle_buffer = (double*) calloc(buffer_length, sizeof(double));
	for(i = 0; i < buffer_length; i++)
		wind_angle_buffer[i] = to_copy_from.wind_angle_buffer[i];
}

SampleBuffer::~SampleBuffer(){
	free(concentration_buffer);
	free(wind_intensity_buffer);
	free(wind_angle_buffer);
}



void SampleBuffer::set_buffer_length(int length){
	buffer_length = length;

	concentration_buffer 	= (double*) realloc(concentration_buffer, 	buffer_length * sizeof(double));
	wind_intensity_buffer 	= (double*) realloc(wind_intensity_buffer, 	buffer_length * sizeof(double));
	wind_angle_buffer 		= (double*) realloc(wind_angle_buffer, 		buffer_length * sizeof(double));
}

void SampleBuffer::reset_all(){
	instant_concentration = 0;
	average_concentration = 0;
	instant_wind_intensity = 0;
	average_wind_intensity = 0;
	instant_wind_angle = 0;
	average_wind_angle = 0;
	
	n_samples = 0;
}



void SampleBuffer::add_all_to_buffer(){
	add_odor_to_buffer();
	add_wind_to_buffer();
	n_samples--; //it was done twice
}

void SampleBuffer::add_odor_to_buffer(){
	concentration_buffer[n_samples] = instant_concentration;
	average_concentration += instant_concentration; //use it as a sum temporarily
	n_samples++;
}

void SampleBuffer::add_wind_to_buffer(){
	wind_intensity_buffer[n_samples] = instant_wind_intensity;
	average_wind_intensity += instant_wind_intensity;
	wind_angle_buffer[n_samples] = instant_wind_angle;
	n_samples++;
}



void SampleBuffer::average_all(){
	average_odor();
	average_wind();
}

void SampleBuffer::average_odor(){
	//mean
	average_concentration /= n_samples;
	// standard deviation
	double variance = 0;
	for(int i = 0; i < n_samples; i++)
      	variance += pow(concentration_buffer[i] - average_concentration, 2);
    variance /= n_samples;
    stdDev_concentration = sqrt(variance);
}

void SampleBuffer::average_wind(){
	int i=0;
	double tmp_sin = 0, tmp_cos = 0;

	average_wind_intensity /= n_samples;

	for(i = 0; i < n_samples; i++){
		tmp_sin += sin(wind_angle_buffer[i]);
		tmp_cos += cos(wind_angle_buffer[i]);
	}
	average_wind_angle = atan2(tmp_sin/n_samples, tmp_cos/n_samples);
	//averaging technique from https://rosettacode.org/wiki/Averages/Mean_angle
}