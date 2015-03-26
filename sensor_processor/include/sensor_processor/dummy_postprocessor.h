#ifndef SENSOR_PROCESSOR_DUMMY_POSTPROCESSOR_H
#define SENSOR_PROCESSOR_DUMMY_POSTPROCESSOR_H

#include "std_msgs/Int32.h"
#include "sensor_processor/postprocessor.h"
#include "sensor_processor/abstract_handler.h"

namespace sensor_processor
{
  /**
   * @class DummyPostProcessor TODO
   */
  class DummyPostProcessor : public PostProcessor<std_msgs::Int32, std_msgs::Int32>
  {
  public:
    DummyPostProcessor (const std::string& ns, AbstractHandler* handler);

    virtual bool
      postProcess(const std_msgs::Int32ConstPtr& input, const std_msgs::Int32Ptr& output);

  private:
    int data_;
  };
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_DUMMY_POSTPROCESSOR_H
