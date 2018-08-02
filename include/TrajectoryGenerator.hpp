/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/Timer.hpp>
#include <string>
#include <Eigen/Dense>
#include <kdl/frames.hpp>

#include <rst-rt/geometry/Translation.hpp>

#include <rst-rt/geometry/Pose.hpp>
#include <rst-rt/kinematics/Twist.hpp>
#include <rst-rt/robot/JointState.hpp>

class TrajectoryGenerator : public RTT:TaskContext {

public:
    TrajectoryGenerator(std::string const & name);

    bool configureHook();
    bool startHook();
    void stopHook();
    void cleanupHook();
    void updateHook();

    void setLinearTarget(rstrt::geometry::Translation &start, rstrt::geometry::Translation &target);
    void samplePath(Eigen::Vector3f &target);
    bool checkDistanceToStart();
    bool isCloseToStart();

    ~TrajectoryGenerator();

protected:
    // Ports
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpacePosition_port;
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    
    // Data Flow
    RTT::FlowStatus out_desiredTaskSpacePosition_flow;
    RTT::FlowStatus in_robotstatus_flow;

    // Variables
    Eigen::VectorXf out_desiredTaskSpacePosition_var;
    rstrt::robot::JointState in_robotstatus_var;

    bool radialTrajectory;
    bool reachedStart;
    float radius;
    Eigen::VectorXf currentPosition;
    Eigen::VectorXf desiredPosition;
    Eigen::VectorXf nextPosition;
}