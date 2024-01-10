#ifndef MESSAGE_H
#define MESSAGE_H

#include "Position.h"

class Message{ 
    public: 
    
    int sender_ID; 
    int receiver_ID; //if == 0 -->> msg for supervisor, otherwise it's for robots
    Position position;
    double concentration;
    Position goal;
    int measurement0_goal1; //binary flag that shows what is the values we are sharing 
    double buffer[10];
    int buffer_size;


    //constructor : everithing set to 0
    Message();
    //constructor : set parameters
    Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag);
    Message(const void * m_buffer);

    void params_to_buffer();
    void buffer_to_params();
    void print_message();
    
}; 

#endif