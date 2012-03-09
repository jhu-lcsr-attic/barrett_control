#ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
#define __BARD_COMPONENTS_CONTROLLER_MUX_H

// This class can be connected to multiple controllers, and performs
// automatic validation to ensure that large impulses do not arise from
// switching controllers at runtime.

namespace bard_components {
  class ControllerMux : public RTT::TaskContext {

  };
}

#endif // ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
