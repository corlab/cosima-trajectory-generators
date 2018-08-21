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

class TrajectoryGeneratorLinear : public RTT::TaskContext {

public:
    TrajectoryGeneratorLinear(std::string const & name);

    bool configureHook();
    void stopHook();
    void cleanupHook();
    void updateHook();

    void preparePorts();
    void setNumEndeffectors(unsigned int DOFsize);

    bool setLinearStart(float a, float b, float c);
    bool setLinearTarget(float a, float b, float c);
    void samplePath(Eigen::VectorXf &target);
    bool checkDistanceToStart();

protected:
    // Ports
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpacePosition_port;
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpaceVelocity_port;
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpaceAcceleration_port;

    RTT::InputPort<Eigen::VectorXf> in_cartPos_port;
    RTT::InputPort<Eigen::VectorXf> in_cartVel_port;
    RTT::InputPort<Eigen::VectorXf> in_cartAcc_port;
    
    // Data Flow
    RTT::FlowStatus out_desiredTaskSpacePosition_flow;
    RTT::FlowStatus out_desiredTaskSpaceVelocity_flow;
    RTT::FlowStatus out_desiredTaskSpaceAcceleration_flow;

    RTT::FlowStatus in_cartPos_flow;
    RTT::FlowStatus in_cartVel_flow;
    RTT::FlowStatus in_cartAcc_flow;

    // Variables
    Eigen::VectorXf out_desiredTaskSpacePosition_var;
    Eigen::VectorXf out_desiredTaskSpaceVelocity_var;
    Eigen::VectorXf out_desiredTaskSpaceAcceleration_var;

    Eigen::VectorXf in_cartPos_var;
    Eigen::VectorXf in_cartVel_var;
    Eigen::VectorXf in_cartAcc_var;

    bool radialTrajectory;
    bool reachedStart;
    bool portsArePrepared;
    bool goForIt;
    bool keepOrientation;

    int recoveryFactor;

    float radius, timeScale;
    unsigned int DOFsize, workspaceDimension, numberOfEndEffectors;
    Eigen::VectorXf currentPosition;
    Eigen::VectorXf desiredPosition, startPosition;
    Eigen::VectorXf nextPosition, nextVel, nextAcc;
};