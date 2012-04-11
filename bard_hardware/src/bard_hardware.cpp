
#include <ocl/Component.hpp>
#include <bard_hardware/wam.h>
#include <bard_hardware/wam_stub.h>

ORO_CREATE_COMPONENT_LIBRARY()
#ifdef __XENO__
ORO_LIST_COMPONENT_TYPE(bard_hardware::WAM)
#endif
ORO_LIST_COMPONENT_TYPE(bard_hardware::WAMStub)
