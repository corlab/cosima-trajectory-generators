/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#include "TrajectoryGeneratorLinear.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

TrajectoryGeneratorLinear::TrajectoryGeneratorLinear(const std::string &name): RTT::TaskContext(name) {
    addOperation("setStartPosition", &TrajectoryGeneratorLinear::setLinearStart, this, ClientThread);
    addOperation("setTargetPostion", &TrajectoryGeneratorLinear::setLinearTarget, this, ClientThread);
    
    addOperation("preparePorts", &TrajectoryGeneratorLinear::preparePorts, this);
    addOperation("setLinearTarget", &TrajectoryGeneratorLinear::setLinearTarget, this);
    addOperation("setLinearStart", &TrajectoryGeneratorLinear::setLinearStart, this);
    addOperation("setNumEndeffectors", &TrajectoryGeneratorLinear::setNumEndeffectors, this);
    
    addProperty("recoveryFactor", recoveryFactor);
    addProperty("timeScale", timeScale);
    addProperty("keepOrientation", keepOrientation);
    addProperty("goForIt", goForIt);
}

void TrajectoryGeneratorLinear::preparePorts() {
    if (portsArePrepared) {
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

void TrajectoryGeneratorLinear::setNumEndeffectors(unsigned int num) {
    assert(num > 0);
    numberOfEndEffectors = num;

    workspaceDimension = 3 * num;
    
    this->in_cartPos_var = Eigen::VectorXf::Zero(workspaceDimension);
    this->in_cartVel_var = Eigen::VectorXf::Zero(workspaceDimension);
    this->in_cartAcc_var = Eigen::VectorXf::Zero(workspaceDimension);
    
    this->out_desiredTaskSpacePosition_var = Eigen::VectorXf::Zero(workspaceDimension);
    this->out_desiredTaskSpaceVelocity_var = Eigen::VectorXf::Zero(workspaceDimension);
    this->out_desiredTaskSpaceAcceleration_var = Eigen::VectorXf::Zero(workspaceDimension);

    this->startPosition = Eigen::VectorXf::Zero(workspaceDimension);
    this->desiredPosition = Eigen::VectorXf::Zero(workspaceDimension);
}

bool TrajectoryGeneratorLinear::configureHook() {
    keepOrientation = false;
    recoveryFactor = 10;
    timeScale = 0.001;
    reachedStart = false;
    portsArePrepared = false;
    goForIt = false;
    return true;
}

void TrajectoryGeneratorLinear::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void TrajectoryGeneratorLinear::cleanupHook() {
    reachedStart = false;
    // cleaning the component data
}

bool TrajectoryGeneratorLinear::setLinearStart(float x, float y, float z) {
    startPosition = Eigen::Vector3f(x,y,z);
    return true;
}

bool TrajectoryGeneratorLinear::setLinearTarget(float x, float y, float z) {
    //TODO: Current way to test. Later on setup ports for this with arbitrary DOFSize

//bool TrajectoryGeneratorLinear::setLinearTarget(Eigen::VectorXf &start, Eigen::VectorXf &target) {
    //if(startPosition.size() == DOFsize && start.size() == DOFsize && target.size() == DOFsize){
        reachedStart = false;
        desiredPosition = Eigen::Vector3f(x,y,z);

        return true;
    //}

    return false;
}

bool TrajectoryGeneratorLinear::checkDistanceToStart() {
    for (int i=0; i < workspaceDimension; i++) {
        if (fabs(startPosition(i) - in_cartPos_var(i)) > 0.01) {
            return false;
        }
    }
    return true;
}

void TrajectoryGeneratorLinear::samplePath(Eigen::VectorXf &target) {
    for (int i = 0; i < numberOfEndEffectors; i++) {
        out_desiredTaskSpacePosition_var.segment<3>(i * workspaceDimension) = (target - in_cartPos_var.segment<3>(i * workspaceDimension))/recoveryFactor + in_cartPos_var.segment<3>(i * workspaceDimension);
        out_desiredTaskSpaceVelocity_var.segment<3>(i * workspaceDimension) = (out_desiredTaskSpacePosition_var - in_cartPos_var.segment<3>(i * workspaceDimension))/timeScale;
        out_desiredTaskSpaceAcceleration_var.segment<3>(i * workspaceDimension)  = (out_desiredTaskSpaceVelocity_var - in_cartVel_var)/timeScale;
        //TODO: Add maximal speed to dimensions, then check im sign of distance changed. If so, new pos would overshoot, so set to target.
    }
}

void TrajectoryGeneratorLinear::updateHook() {

    //Update current position information
    if (in_cartPos_port.connected()) {
        in_cartPos_flow = in_cartPos_port.read(in_cartPos_var);
    } else {
        return;
    }

    //Check if start position was already reached
    if (goForIt) {
        if (reachedStart || checkDistanceToStart()) {
            samplePath(desiredPosition);
        } else {
            samplePath(startPosition);
        }

        out_desiredTaskSpacePosition_port.write(out_desiredTaskSpacePosition_var);
        out_desiredTaskSpaceVelocity_port.write(out_desiredTaskSpaceVelocity_var); 
    }
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(TrajectoryGeneratorLinear)