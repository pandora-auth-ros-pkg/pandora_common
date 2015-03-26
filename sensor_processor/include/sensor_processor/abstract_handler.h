#ifndef SENSOR_PROCESSOR_ABSTRACT_HANDLER_H
#define SENSOR_PROCESSOR_ABSTRACT_HANDLER_H

#include <ros/forwards.h>
#include "state_manager/state_client.h"

namespace sensor_processor
{
  /**
   * @class AbstractHandler TODO
   */
  class AbstractHandler : public StateClient
  {
    public:
      // AbstractHandler () {}
      // virtual
        // ~AbstractHandler() {}
      virtual ros::NodeHandlePtr
        shareNodeHandle() = 0;

    protected:
      virtual void
        startTransition(int newState) {}
      virtual void
        completeTransition() {}
  };
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_ABSTRACT_HANDLER_H

