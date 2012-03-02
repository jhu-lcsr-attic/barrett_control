
#include <ocl/Component.hpp>
#include <bard_components/wam.h>
#include <bard_components/controllers/trivial.h>

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(bard_components::WAM)
ORO_LIST_COMPONENT_TYPE(bard_components::controllers::Trivial)
