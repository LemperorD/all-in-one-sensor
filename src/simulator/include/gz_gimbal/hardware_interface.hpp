#ifndef HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

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

#endif  // HARDWARE_INTERFACE_HPP_
