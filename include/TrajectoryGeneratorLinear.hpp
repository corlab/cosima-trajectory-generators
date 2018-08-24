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


class TrajectoryGeneratorLinear : public TrajectoryGenerator {

public:
    TrajectoryGeneratorLinear(std::string const & name);

    void samplePath(Eigen::VectorXf &target);
};