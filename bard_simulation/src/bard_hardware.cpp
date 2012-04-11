
#include <ocl/Component.hpp>
#include <bard_simulation/wam.h>
#include <bard_simulation/wam_stub.h>

ORO_CREATE_COMPONENT_LIBRARY()
#ifdef __XENO__
ORO_LIST_COMPONENT_TYPE(bard_simulation::WAM)
#endif
ORO_LIST_COMPONENT_TYPE(bard_simulation::WAMStub)
