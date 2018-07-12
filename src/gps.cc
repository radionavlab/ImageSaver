// Author: Tucker Haydon
#include "gps.h"

// Extern Variable
GPSSolution gps_solution;
std::mutex gps_solution_mutex;

void AttitudeMessageHandler(const mg_msgs::Attitude2D msg) {
    const double rx                     = msg.rx;
    const double ry                     = msg.ry;
    const double rz                     = msg.rz;
    const double rxRov                  = msg.rxRov;
    const double ryRov                  = msg.ryRov;
    const double rzRov                  = msg.rzRov;
    const mg_msgs::BaseTime tSolution   = msg.tSolution;
    const double deltRSec               = msg.deltRSec;
    const std::vector<float> P          = msg.P;
    const uint32_t nCov                 = msg.nCov;
    const double azAngle                = msg.azAngle;
    const double elAngle                = msg.elAngle;
    const double azSigma                = msg.azSigma;
    const double elSigma                = msg.elSigma;
    const double testStat               = msg.testStat;
    const uint8_t numDD                 = msg.numDD;
    const uint8_t bitfield              = msg.bitfield;

    // Lock mutex for write
    std::lock_guard<std::mutex> guard(gps_solution_mutex);

    // Copy the attitude
    gps_solution.az      = azAngle;
    gps_solution.el      = elAngle;
    gps_solution.azSigma = azSigma;
    gps_solution.elSigma = elSigma;

    // Copy the covariance
    constexpr size_t FLOAT_SIZE = sizeof(float);
    static uint32_t NUM_COV = (nCov*(nCov+1))/2;
    memcpy(gps_solution.attCov, P.data(), NUM_COV * FLOAT_SIZE);
}

void PositionMessageHandler(const mg_msgs::SingleBaselineRTK msg) {
    const double rx                     = msg.rx;
    const double ry                     = msg.ry;
    const double rz                     = msg.rz;
    const double rxRov                  = msg.rxRov;
    const double ryRov                  = msg.ryRov;
    const double rzRov                  = msg.rzRov;
    const mg_msgs::BaseTime tSolution   = msg.tSolution;
    const double deltRSec               = msg.deltRSec;
    const std::vector<float> P          = msg.P;
    const uint32_t nCov                 = msg.nCov;
    const double testStat               = msg.testStat;
    const double ageOfReferenceData     = msg.ageOfReferenceData;
    const uint8_t numDD                 = msg.numDD;
    const uint8_t bitfield              = msg.bitfield;

    // Lock mutex for write
    std::lock_guard<std::mutex> guard(gps_solution_mutex);

    // Copy the position
    gps_solution.x = rxRov;
    gps_solution.y = ryRov;
    gps_solution.z = rzRov;

    // Copy the covariance
    constexpr size_t FLOAT_SIZE = sizeof(float);
    static uint32_t NUM_COV = (nCov*(nCov+1))/2;
    memcpy(gps_solution.posCov, P.data(), NUM_COV * FLOAT_SIZE);
}
