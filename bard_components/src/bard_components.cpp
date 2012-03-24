
#include <ocl/Component.hpp>
#include <bard_components/wam.h>
#include <bard_components/controller_mux.h>
#include <bard_components/controllers/trivial.h>
#include <bard_components/controllers/gravity_compensation.h>
#include <bard_components/controllers/joint_pid.h>
#include <bard_components/controllers/wrench.h>
#include <bard_components/controllers/pose.h>
#include <bard_components/controllers/joint_trajectory.h>

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(bard_components::WAM)
ORO_LIST_COMPONENT_TYPE(bard_components::ControllerMux)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::Trivial)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::GravityCompensation)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::JointPID)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::CartesianWrench)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::CartesianPose)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::JointTrajectory)
