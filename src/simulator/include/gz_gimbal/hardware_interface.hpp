#ifndef RMOSS_GZ_BASE__HARDWARE_INTERFACE_HPP_
#define RMOSS_GZ_BASE__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace rmoss_gz_base
{

template<class DataT>
class Actuator
{
public:
  using SharedPtr = std::shared_ptr<Actuator<DataT>>;
  virtual void set(const DataT & data) = 0;
};

template<class DataT>
using SensorCallback = std::function<void (const DataT & data, const rclcpp::Time & stamp)>;

template<class DataT>
class Sensor
{
public:
  using SharedPtr = std::shared_ptr<Sensor<DataT>>;
  virtual void add_callback(SensorCallback<DataT> callback) = 0;
};

template<class DataT>
class DataSensor : public Sensor<DataT>
{
public:
  DataSensor() {}
  void add_callback(SensorCallback<DataT> callback) override
  {
    callbacks_.push_back(callback);
  }
  void update(const DataT & data, const rclcpp::Time & stamp)
  {
    if (callbacks_.size() > 0) {
      for (auto & cb : callbacks_) {
        cb(data, stamp);
      }
    }
  }
  std::vector<SensorCallback<DataT>> callbacks_;
};

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__HARDWARE_INTERFACE_HPP_
