#ifndef SENSOR_PROCESSOR_DUMMY_HANDLER_H
#define SENSOR_PROCESSOR_DUMMY_HANDLER_H

#include "sensor_processor/handler.h"
#include "std_msgs/Int32.h"

namespace sensor_processor
{
  typedef std_msgs::Int32 Int32;
  class DummyHandler : public Handler<Int32, Int32, Int32, Int32>
  {
  public:
    DummyHandler(const std::string& ns);

  protected:
    virtual void
      startTransition(int newState);
    virtual void
      completeTransition();
  };
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_DUMMY_HANDLER_H
