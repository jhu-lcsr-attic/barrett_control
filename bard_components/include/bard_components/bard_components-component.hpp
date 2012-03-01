#ifndef OROCOS_BARD_COMPONENTS_COMPONENT_HPP
#define OROCOS_BARD_COMPONENTS_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>

class Bard_components
    : public RTT::TaskContext
{
 public:
    Bard_components(string const& name)
        : TaskContext(name)
    {
        std::cout << "Bard_components constructed !" <<std::endl;
    }

    bool configureHook() {
        std::cout << "Bard_components configured !" <<std::endl;
        return true;
    }

    bool startHook() {
        std::cout << "Bard_components started !" <<std::endl;
        return true;
    }

    void updateHook() {
        std::cout << "Bard_components executes updateHook !" <<std::endl;
    }

    void stopHook() {
        std::cout << "Bard_components executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "Bard_components cleaning up !" <<std::endl;
    }
};

#endif
