//Author: Tucker Haydon
#pragma once

#include <mutex>
#include <mg_msgs/Attitude2D.h>
#include <mg_msgs/SingleBaselineRTK.h>


// Position of primary antenna with respect to reference in ECEF
// Position is in meters
// Pose is in radians
typedef struct {
    double x;
    double y;
    double z;
    float posCov[6];
    double az;
    double el;
    double elSigma;
    double azSigma;
    float attCov[6];
} GPSSolution;

extern GPSSolution gps_solution;
extern std::mutex gps_solution_mutex;

void PositionMessageHandler(const mg_msgs::SingleBaselineRTK msg);
void AttitudeMessageHandler(const mg_msgs::Attitude2D msg);
