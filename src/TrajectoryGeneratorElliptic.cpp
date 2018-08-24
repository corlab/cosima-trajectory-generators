/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#include "TrajectoryGeneratorElliptic.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

# define M_PI           3.14159265358979323846  /* pi */

using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

TrajectoryGeneratorElliptic::TrajectoryGeneratorElliptic(const std::string &name): TrajectoryGenerator::TrajectoryGenerator(name) {
    addOperation("setRadii", &TrajectoryGeneratorElliptic::setRadii, this, ClientThread);
    addOperation("setShift", &TrajectoryGeneratorElliptic::setShift, this, ClientThread);
    addOperation("setStepSize", &TrajectoryGeneratorElliptic::setStepSize, this, ClientThread);
    
    validEllipsoid = false;
    radA = 0;
    radB = 0;
    currentStep = 0;
    stepSize = 32;

    ellipsePositions = Eigen::VectorXf::Zero(stepSize * 3);

    shift.setZero();
}

void TrajectoryGeneratorElliptic::setStepSize(unsigned int _stepSize) {
    assert(stepSize > 1);
    this->stepSize = _stepSize;
    this->ellipsePositions = Eigen::VectorXf::Zero(stepSize * 3);
}

void TrajectoryGeneratorElliptic::setShift(float _shiftX, float _shiftY, float _shiftZ) {
    shift(0) = _shiftX;
    shift(1) = _shiftY;
    shift(3) = _shiftZ;
}

void TrajectoryGeneratorElliptic::setRadii(float _radA, float _radB, float _radC) {
    assert(radA > 0);
    assert(radB > 0);

    this->radA = _radA;
    this->radB = _radB;
    this->radC = _radC;

    computeEllipsoid();
}

void TrajectoryGeneratorElliptic::samplePath(Eigen::VectorXf &target) {
    //Enters this function when the last goal was reached
    currentStep = (currentStep+1 < stepSize) ? currentStep+1 : 0;
    startPosition.segment<3>(0) = ellipsePositions.segment<3>(currentStep*3);
    sampleLinearPath(startPosition);
    reachedStart = false;
}

void TrajectoryGeneratorElliptic::computeEllipsoid() {
    reachedStart = false;
    float phi;
    assert(numberOfEndEffectors == 1);
    assert(radA > 0);
    assert(radB > 0);

    for (int i = 1; i != stepSize; i++) {
        phi = 2*M_PI/stepSize;
        ellipsePositions.segment<3>((i-1)*3) = shift + desiredPosition.segment<3>(0) + Eigen::Vector3f(radA * cos(phi), radB * sin(phi), 1);
    }

    //startPosition is calculated automatially
    startPosition.segment<3>(0) = ellipsePositions.segment<3>(0);
}

bool TrajectoryGeneratorElliptic::setStartPos(float a, float b, float c) {
    RTT::log(RTT::Error) << "The elliptic trajectory generator sets its start position automatically.\nYou need to specify the center with setTarget and radii (and optionally you can add a shift)" << RTT::endlog();
    return false;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TrajectoryGeneratorElliptic)