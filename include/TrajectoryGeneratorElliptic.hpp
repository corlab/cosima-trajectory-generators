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
    void setRadii(float radA, float radB, float radC);
    void computeEllipsoid();
    
    bool setStartPos(float a, float b, float c);
    

protected:

    Eigen::VectorXf ellipsePositions;
    Eigen::Vector3f shift;

    unsigned int stepSize, currentStep;
    float radA, radB, radC;
    bool validEllipsoid;
};