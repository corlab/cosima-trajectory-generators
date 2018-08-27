/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#pragma once

#include "TrajectoryGenerator.hpp"
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/Timer.hpp>
#include <string>
#include <Eigen/Dense>
#include <kdl/frames.hpp>


class TrajectoryGeneratorElliptic : public TrajectoryGenerator {

public:
    TrajectoryGeneratorElliptic(std::string const & name);

    void samplePath(Eigen::VectorXf &target);
    void setStepSize(unsigned int stepSize);
    void setShift(float shiftX, float shiftY, float shiftZ);
    void setRadii(float radA, float radB);
    void setAngles(float angleX, float angleY, float angleZ);
    void computeEllipsoid();
    
    void printPositions();
    bool setStartPos(float a, float b, float c);
    

protected:

    Eigen::Matrix3Xf ellipsePositions;
    Eigen::Vector3f shift;
    Eigen::Matrix3f rotEuler;

    unsigned int stepSize, currentStep;
    float radA, radB;
    bool validEllipsoid;
};