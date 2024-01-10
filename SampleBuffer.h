#ifndef SAMPLEBUFFER_H
#define SAMPLEBUFFER_H

class SampleBuffer{ 
    public: 

    //buffer length
    int buffer_length;
    //sample counter
    int n_samples;
    //odor
    double* concentration_buffer;
    double instant_concentration;
    double average_concentration;
    double stdDev_concentration;
    //wind 
    double* wind_intensity_buffer;
    double instant_wind_intensity;
    double average_wind_intensity;
    double* wind_angle_buffer;
    double instant_wind_angle;
    double average_wind_angle;
    //!\ angle must be in radian [0,2PI]

    //default constructor
    SampleBuffer();
    //constructor with parameter
    SampleBuffer(int length);
    //copy constructor
    SampleBuffer(const SampleBuffer & to_copy_from);

    //destructor
    ~SampleBuffer();

    //set the buffer length (after using the default constructor)
    void set_buffer_length(int length);
    //reset everything to 0
    void reset_all();

    //add element to buffer
    void add_all_to_buffer();
    void add_odor_to_buffer();
    void add_wind_to_buffer();

    //average calculator
    void average_all();
    void average_odor();
    void average_wind();
}; 

#endif