#ifndef SENSOR_PREPROCESSOR_DUMMY_PREPROCESSOR_H
#define SENSOR_PREPROCESSOR_DUMMY_PREPROCESSOR_H

#include "std_msgs/Int32.h"
#include "sensor_processor/preprocessor.h"
#include "sensor_processor/abstract_handler.h"

namespace sensor_processor
{
  /**
   * @class DummyPreProcessor TODO
   */
  class DummyPreProcessor : public PreProcessor<std_msgs::Int32, std_msgs::Int32>
  {
  public:
    DummyPreProcessor (const std::string& ns, AbstractHandler* handler);

    virtual bool
      preProcess(const std_msgs::Int32ConstPtr& input, const std_msgs::Int32Ptr& output);

  private:
    int data_;
  };
}  // namespace sensor_processor

#endif  // SENSOR_PREPROCESSOR_DUMMY_PREPROCESSOR_H
