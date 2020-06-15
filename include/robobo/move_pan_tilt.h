#ifndef MOVE_PAN_TILT_PLUGIN_H_
#define MOVE_PAN_TILT_PLUGIN_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <cstdint>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>

#include "robobo_msgs/MovePanTilt.h"
#include "robobo_msgs/MovePanTiltCommand.h"

namespace gazebo
{
class MovePanTilt : public ModelPlugin
{
public:
  physics::JointControllerPtr jointController;

  /// \brief Constructor
  MovePanTilt();

  /// \brief Destructor
  ~MovePanTilt();

  /// \brief Load the controller
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  bool ServiceCallback(robobo_msgs::MovePanTilt::Request &req, robobo_msgs::MovePanTilt::Response &res);
  void TopicCallback(const boost::shared_ptr<robobo_msgs::MovePanTiltCommand const> &msg);
  void QueueThread();

private:
  void HandleMovePanTilt(int16_t panPos, int8_t panSpeed, int16_t panBlockId, int16_t tiltPos,
                                    int8_t tiltSpeed, int16_t tiltBlockId);

  /// \brief Pointer to the model.
  physics::ModelPtr model;

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A thread the keeps running velocity setting
  std::thread movePanTiltThread;

  /// \brief A ROS service server
  ros::ServiceServer movePanTiltService;

  /// \brief A ROS Subscriber
  ros::Subscriber movePanTiltSubscriber;

  /// \brief BlockId publisher
  ros::Publisher blockPub;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;
};

}  // namespace gazebo

#endif
