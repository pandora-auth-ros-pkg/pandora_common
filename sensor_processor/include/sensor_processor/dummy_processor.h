#ifndef SENSOR_PROCESSOR_DUMMY_PROCESSOR_H
#define SENSOR_PROCESSOR_DUMMY_PROCESSOR_H

#include "std_msgs/Int32.h"
#include "sensor_processor/processor.h"
#include "sensor_processor/abstract_handler.h"

namespace sensor_processor
{
  /**
   * @class DummyProcessor TODO
   */
  class DummyProcessor : public Processor<std_msgs::Int32, std_msgs::Int32>
  {
  public:
    DummyProcessor (const std::string& ns, AbstractHandler* handler);

    virtual bool
      process(const std_msgs::Int32ConstPtr& input, const std_msgs::Int32Ptr& output);

  private:
    int data_;
  };
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_DUMMY_PROCESSOR_H
