#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

using namespace RTT;
using namespace std;

/**
 * An example service which can be loaded in a component.
 */
class MyService : public RTT::Service {
public:
    MyService(TaskContext* owner) 
        : Service("myservice", owner) 
    {
        this->addOperation("getOwnerName", &MyService::getOwnerName, this).doc("Returns the name of the owner of this object.");
    }

    string getOwnerName() {
        // getOwner() returns the TaskContext pointer we got in
        // the constructor:
        return getOwner()->getName();
    }
};

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(MyService, "myservice")
