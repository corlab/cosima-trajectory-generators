/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#include <TrajectoryGenerator.hpp>

using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

TrajectoryGenerator::TrajectoryGenerator(const std::string &name): TaskContext(name) {

    addOperation("setLinearTarget", &TrajectoryGenerator::setLinearTarget, this);
    
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    out_desiredTaskSpacePosition_var.setZero();

    out_desiredTaskSpacePosition_port.setName("out_desiredTaskSpacePosition_port");
    out_desiredTaskSpacePosition_port.doc("Output port to send the next desired position of the trajectory to the robot");
    out_desiredTaskSpacePosition_port.setDataSample(out_desiredTaskSpacePosition_var);

    ports()->addPort(out_counter_port);
}

bool TrajectoryGenerator::configureHook() {
    reachedStart = false;

    return true;
}

void TrajectoryGenerator::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void TrajectoryGenerator::cleanupHook() {
    reachedStart = false;
    // cleaning the component data
}

void TrajectoryGenerator::setLinearTarget(rstrt::geometry::Translation &start, rstrt::geometry::Translation &target) {
    reachedStart = false;
}

bool TrajectoryGenerator::checkDistanceToStart() {
    return true;
}

void TrajectoryGenerator::samplePath(Eigen::Vector3f &target) {
    return true;
}

bool TrajectoryGenerator::isCloseToStart() {
    return true;
}

void TrajectoryGenerator::updateHook() {

    //Update current position information
    if (in_robotstatus_port.connected()) {
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        return;
    }

    in_robotstatus_var.angle;

    //Check if start position was already reached
    if (reachedStart || isCloseToStart()) {
        //sample path to target position
    } else {
        // sample path to start position
    }
}