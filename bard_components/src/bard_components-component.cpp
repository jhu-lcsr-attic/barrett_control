#include "bard_components-component.hpp"
#include <rtt/Component.hpp>

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use 
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Bard_components)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Bard_components)
