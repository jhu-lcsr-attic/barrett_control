
#include <ocl/Component.hpp>
#include <bard_controllers/controller_mux.h>
#include <bard_controllers/controllers/trivial.h>
#include <bard_controllers/controllers/gravity_compensation.h>
#include <bard_controllers/controllers/joint_pid.h>
#include <bard_controllers/controllers/wrench.h>
#include <bard_controllers/controllers/pose.h>
#include <bard_controllers/controllers/joint_trajectory.h>

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(bard_controllers::ControllerMux)
ORO_LIST_COMPONENT_TYPE(bard_controllers::controllers::Trivial)
ORO_LIST_COMPONENT_TYPE(bard_controllers::controllers::GravityCompensation)
ORO_LIST_COMPONENT_TYPE(bard_controllers::controllers::JointPID)
ORO_LIST_COMPONENT_TYPE(bard_controllers::controllers::CartesianWrench)
ORO_LIST_COMPONENT_TYPE(bard_controllers::controllers::CartesianPose)
ORO_LIST_COMPONENT_TYPE(bard_controllers::controllers::JointTrajectory)
