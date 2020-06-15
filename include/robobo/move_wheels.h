#ifndef MOVE_WHEELS_PLUGIN_H_
#define MOVE_WHEELS_PLUGIN_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <cstdint>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>

#include "robobo_msgs/MoveWheels.h"
#include "robobo_msgs/MoveWheelsCommand.h"

namespace gazebo
{
class MoveWheels : public ModelPlugin
{
public:
  physics::JointControllerPtr jointController;

  /// \brief Constructor
  MoveWheels();

  /// \brief Destructor
  ~MoveWheels();

  /// \brief Load the controller
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  bool ServiceCallback(robobo_msgs::MoveWheels::Request &req, robobo_msgs::MoveWheels::Response &res);
  void TopicCallback(const boost::shared_ptr<robobo_msgs::MoveWheelsCommand const> &msg);
  void QueueThread();

private:

  void HandleMoveWheels(int8_t lwp, int8_t rwp, float time, int16_t blockid);

  /// \brief Pointer to the model.
  physics::ModelPtr model;

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS service server
  ros::ServiceServer moveWheelsService;

  /// \brief A ROS Subscriber
  ros::Subscriber moveWheelsSubscriber;

  /// \brief BlockId publisher
  ros::Publisher blockPub;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  /// \brief A thread the keeps running velocity setting
  std::thread moveWheelsThread;
};

}  // namespace gazebo

#endif
