/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#include "TrajectoryGeneratorElliptic.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define M_PI 3.14159265358979323846 /* pi */

using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

TrajectoryGeneratorElliptic::TrajectoryGeneratorElliptic(const std::string &name) : TrajectoryGenerator::TrajectoryGenerator(name)
{
    addOperation("setRadii", &TrajectoryGeneratorElliptic::setRadii, this, ClientThread);
    addOperation("setShift", &TrajectoryGeneratorElliptic::setShift, this, ClientThread);
    addOperation("setStepSize", &TrajectoryGeneratorElliptic::setStepSize, this, ClientThread);
    addOperation("setAngles", &TrajectoryGeneratorElliptic::setAngles, this, ClientThread);
    addOperation("printPostions", &TrajectoryGeneratorElliptic::printPositions, this);

    validEllipsoid = false;
    radA = 0;
    radB = 0;
    currentStep = 0;
    stepSize = 32;

    ellipsePositions = Eigen::MatrixXf::Zero(stepSize, 3);

    rotEuler.setZero();
    shift.setZero();
}

void TrajectoryGeneratorElliptic::setStepSize(unsigned int _stepSize)
{
    assert(stepSize > 1);
    this->stepSize = _stepSize;
    this->ellipsePositions = Eigen::MatrixXf::Zero(stepSize, 3);
}

void TrajectoryGeneratorElliptic::setShift(float _shiftX, float _shiftY, float _shiftZ)
{
    shift(0) = _shiftX;
    shift(1) = _shiftY;
    shift(3) = _shiftZ;
}

void TrajectoryGeneratorElliptic::setAngles(float _angleX, float _angleY, float _angleZ)
{
    rotEuler(0) = _angleX;
    rotEuler(1) = _angleY;
    rotEuler(2) = _angleZ;

    rotEuler = Eigen::AngleAxisf(_angleX, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(_angleY, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(_angleZ, Eigen::Vector3f::UnitZ());

    computeEllipsoid();
}

void TrajectoryGeneratorElliptic::setRadii(float _radA, float _radB)
{
    assert(radA > 0);
    assert(radB > 0);

    this->radA = _radA;
    this->radB = _radB;
}

void TrajectoryGeneratorElliptic::samplePath(Eigen::VectorXf &target)
{
    //Enters this function when the last goal was reached
    currentStep = (currentStep + 1 < stepSize) ? currentStep + 1 : 0;
    startPosition.segment<3>(0) = ellipsePositions.row(currentStep);
    sampleLinearPath(startPosition);
    reachedStart = false;
}

void TrajectoryGeneratorElliptic::computeEllipsoid()
{
    reachedStart = false;
    float phi;
    assert(numberOfEndEffectors == 1);
    assert(radA > 0);
    assert(radB > 0);

    for (int i = 0; i < stepSize; i++)
    {
        phi = 2 * M_PI / stepSize * (i + 1);
        ellipsePositions.row(i) = shift + desiredPosition.segment<3>(0) + rotEuler * Eigen::Vector3f(radA * cos(phi), radB * sin(phi), 0);
    }

    //startPosition is calculated automatially
    startPosition.segment<3>(0) = ellipsePositions.row(0);
}

void TrajectoryGeneratorElliptic::printPositions()
{
    std::cout << "Ellipse positions: \nmat = [";
    float phi;
    for (int i = 0; i < stepSize; i++)
    {
        phi = 2 * M_PI / stepSize * (i + 1);
        std::cout << (shift + desiredPosition.segment<3>(0) + rotEuler * Eigen::Vector3f(radA * cos(phi), radB * sin(phi), 0)).transpose() << ";\n";
    }
    std::cout << "];\nplot3(mat(:,1), mat(:,2), mat(:,3), \"o-\")\n"
              << std::endl;
}

bool TrajectoryGeneratorElliptic::setStartPos(float a, float b, float c)
{
    RTT::log(RTT::Error) << "The elliptic trajectory generator sets its start position automatically.\nYou need to specify the center with setTarget and radii (and optionally you can add a shift)" << RTT::endlog();
    return false;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TrajectoryGeneratorElliptic)