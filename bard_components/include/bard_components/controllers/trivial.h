#ifndef __BARD_COMPONENTS_CONTROLLERS_TRIVIAL_H
#define __BARD_COMPONENTS_CONTROLLERS_TRIVIAL_H

#include <iostream>

#include <kdl/jntarrayvel.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

namespace bard_components {
  namespace controllers {
    class Trivial : public RTT::TaskContext
    {
      // RTT Ports
      RTT::InputPort<KDL::JntArrayVel> positions_in_port_;
      RTT::OutputPort<KDL::JntArray> torques_out_port_;

    public:
      Trivial(string const& name);
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

    private:
      // Working variables
      unsigned int n_dof_;
      KDL::JntArrayVel positions_;
      KDL::JntArray torques_;
    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_TRIVIAL_H
