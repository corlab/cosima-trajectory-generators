/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#include "TrajectoryGenerator.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

TrajectoryGenerator::TrajectoryGenerator(const std::string &name) : RTT::TaskContext(name)
{
    addOperation("setStartPosition", &TrajectoryGenerator::setStartPos, this, ClientThread);
    addOperation("setTargetPostion", &TrajectoryGenerator::setTargetPos, this, ClientThread);

    addOperation("preparePorts", &TrajectoryGenerator::preparePorts, this);
    addOperation("setNumEndeffectors", &TrajectoryGenerator::setNumEndeffectors, this);

    addProperty("recoveryFactor", recoveryFactor);
    addProperty("timeScale", timeScale);
    addProperty("goForIt", goForIt);
}

void TrajectoryGenerator::preparePorts()
{
    if (portsArePrepared)
    {
        ports()->removePort("in_cartPos_port");
        ports()->removePort("in_cartVel_port");
        ports()->removePort("in_cartAcc_port");
        ports()->removePort("out_desiredTaskSpacePosition_port");
        ports()->removePort("out_desiredTaskSpaceVelocity_port");
        ports()->removePort("out_desiredTaskSpaceAcceleration_port");
    }

    in_cartPos_var.setZero();
    in_cartPos_port.setName("in_cartPos_port");
    in_cartPos_port.doc("Input port for reading current cartesian positions");
    ports()->addPort(in_cartPos_port);
    in_cartPos_flow = RTT::NoData;

    in_cartVel_var.setZero();
    in_cartVel_port.setName("in_cartVel_port");
    in_cartVel_port.doc("Input port for reading current cartesian velocities");
    ports()->addPort(in_cartVel_port);
    in_cartVel_flow = RTT::NoData;

    in_cartAcc_var.setZero();
    in_cartAcc_port.setName("in_cartAcc_port");
    in_cartAcc_port.doc("Input port for reading current cartesian accelerations");
    ports()->addPort(in_cartAcc_port);
    in_cartAcc_flow = RTT::NoData;

    out_desiredTaskSpacePosition_var.setZero();
    out_desiredTaskSpaceVelocity_var.setZero();
    out_desiredTaskSpaceAcceleration_var.setZero();

    out_desiredTaskSpacePosition_port.setName("out_desiredTaskSpacePosition_port");
    out_desiredTaskSpacePosition_port.doc("Output port to send the next desired position of the trajectory to the robot");
    out_desiredTaskSpacePosition_port.setDataSample(out_desiredTaskSpacePosition_var);
    ports()->addPort(out_desiredTaskSpacePosition_port);

    out_desiredTaskSpaceVelocity_port.setName("out_desiredTaskSpaceVelocity_port");
    out_desiredTaskSpaceVelocity_port.doc("Output port to send the next desired velocity of the trajectory to the robot");
    out_desiredTaskSpaceVelocity_port.setDataSample(out_desiredTaskSpaceVelocity_var);
    ports()->addPort(out_desiredTaskSpaceVelocity_port);

    out_desiredTaskSpaceAcceleration_port.setName("out_desiredTaskSpaceAcceleration_port");
    out_desiredTaskSpaceAcceleration_port.doc("Output port to send the next desired acceleration of the trajectory to the robot");
    out_desiredTaskSpaceAcceleration_port.setDataSample(out_desiredTaskSpaceAcceleration_var);
    ports()->addPort(out_desiredTaskSpaceAcceleration_port);

    portsArePrepared = true;
    RTT::log(RTT::Info) << "Done preparing ports" << RTT::endlog();
}

void TrajectoryGenerator::setNumEndeffectors(unsigned int num)
{
    assert(num > 0);
    numberOfEndEffectors = num;

    workspaceDimension = 3 * num;

    this->in_cartPos_var = Eigen::VectorXf::Zero(workspaceDimension);
    this->in_cartVel_var = Eigen::VectorXf::Zero(workspaceDimension);
    this->in_cartAcc_var = Eigen::VectorXf::Zero(workspaceDimension);

    this->out_desiredTaskSpacePosition_var = Eigen::VectorXf::Zero(workspaceDimension + 4);
    this->out_desiredTaskSpaceVelocity_var = Eigen::VectorXf::Zero(workspaceDimension + 3);
    this->out_desiredTaskSpaceAcceleration_var = Eigen::VectorXf::Zero(workspaceDimension + 3);

    this->startPosition = Eigen::VectorXf::Zero(workspaceDimension);
    this->desiredPosition = Eigen::VectorXf::Zero(workspaceDimension);
}

bool TrajectoryGenerator::configureHook()
{
    keepOrientation = false;
    recoveryFactor = 10;
    timeScale = 0.001;
    reachedStart = false;
    portsArePrepared = false;
    goForIt = false;
    return true;
}

void TrajectoryGenerator::stopHook()
{
    // stops the component (update hook wont be  called anymore)
}

void TrajectoryGenerator::cleanupHook()
{
    reachedStart = false;
    // cleaning the component data
}

bool TrajectoryGenerator::setStartPos(float x, float y, float z)
{
    reachedStart = false;
    startPosition = Eigen::Vector3f(x, y, z);
    return true;
}

bool TrajectoryGenerator::setTargetPos(float x, float y, float z)
{
    //TODO: Current way to test. Later on setup ports for this with arbitrary DOFSize

    //bool TrajectoryGenerator::setLinearTarget(Eigen::VectorXf &start, Eigen::VectorXf &target) {
    //if(startPosition.size() == DOFsize && start.size() == DOFsize && target.size() == DOFsize){
    desiredPosition = Eigen::Vector3f(x, y, z);
    Eigen::Quaternionf qtmp = Eigen::Quaternionf(0, 0, 1, 0);
    out_desiredTaskSpacePosition_var(3) = qtmp.w();
    out_desiredTaskSpacePosition_var(4) = qtmp.x();
    out_desiredTaskSpacePosition_var(5) = qtmp.y();
    out_desiredTaskSpacePosition_var(6) = qtmp.z();
    return true;
    //}

    // return false;
}

bool TrajectoryGenerator::checkDistanceToStart()
{
    for (int i = 0; i < workspaceDimension; i++)
    {
        if (fabs(startPosition(i) - in_cartPos_var(i)) > 0.01)
        {
            return false;
        }
    }
    return true;
}

void TrajectoryGenerator::sampleLinearPath(Eigen::VectorXf &target)
{
    for (int i = 0; i < numberOfEndEffectors; i++)
    {
        out_desiredTaskSpacePosition_var.segment<3>(i * (workspaceDimension + 4)) = (target - in_cartPos_var.segment<3>(i * (workspaceDimension + 4))) / recoveryFactor + in_cartPos_var.segment<3>(i * (workspaceDimension + 4));
        out_desiredTaskSpaceVelocity_var.segment<3>(i * (workspaceDimension + 3)) = (out_desiredTaskSpacePosition_var - in_cartPos_var.segment<3>(i * (workspaceDimension + 3))) / timeScale + in_cartPos_var.segment<3>(i * (workspaceDimension + 3));
        // out_desiredTaskSpaceAcceleration_var.segment<3>(i * (workspaceDimension + 3)) = (out_desiredTaskSpaceVelocity_var - in_cartVel_var) / timeScale;
        //TODO: Add maximal speed to dimensions, then check im sign of distance changed. If so, new pos would overshoot, so set to target.
    }
}

void TrajectoryGenerator::updateHook()
{

    //Update current position information
    if (in_cartPos_port.connected())
    {
        in_cartPos_flow = in_cartPos_port.read(in_cartPos_var);
    }
    else
    {
        return;
    }

    //Check if start position was already reached
    if (goForIt)
    {
        if (reachedStart || checkDistanceToStart())
        {
            samplePath(desiredPosition);
        }
        else
        {
            sampleLinearPath(startPosition);
        }

        out_desiredTaskSpacePosition_port.write(out_desiredTaskSpacePosition_var);
        out_desiredTaskSpaceVelocity_port.write(out_desiredTaskSpaceVelocity_var);
    }
}