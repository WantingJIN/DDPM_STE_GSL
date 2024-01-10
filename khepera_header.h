#ifndef KHEPERA_HEADER_H
#define KHEPERA_HEADER_H


#include "controller_STE_clean.h"


#include <sys/stat.h>
#include <sys/time.h>
#include <iostream>
//#include <Eigen/Dense>

extern "C" {
    #include "khepera4.h"
    #include "odorboard.h"
    #include "windsensorboard.h"
    #include "odometry_track.h"
    #include "odometry_goto.h"
    #include "odor_filter.h"
}
#include "udp_msg.h"
#include "wind_filter.h"
#include "vector3d.h" //defines the sVector3d type used in wind read

#define WEBOTS 0
#define KHEPERA 1

extern struct sKhepera4 khepera4;
extern struct sOdorboard odorboard;
extern struct sWindSensorBoard windsensorboard;


#endif