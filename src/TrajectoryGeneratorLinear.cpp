/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#include "TrajectoryGeneratorLinear.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

TrajectoryGeneratorLinear::TrajectoryGeneratorLinear(const std::string &name): TrajectoryGenerator::TrajectoryGenerator(name) {
    
}

void TrajectoryGeneratorLinear::samplePath(Eigen::VectorXf &target) {
    sampleLinearPath(target);
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(TrajectoryGeneratorLinear)