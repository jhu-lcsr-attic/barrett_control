
#include <ocl/Component.hpp>
#include <bard_components/wam.h>
#include <bard_components/controller_mux.h>
#include <bard_components/controllers/trivial.h>
#include <bard_components/controllers/gravity_compensation.h>

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(bard_components::WAM)
ORO_LIST_COMPONENT_TYPE(bard_components::ControllerMux)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::Trivial)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::GravityCompensation)
