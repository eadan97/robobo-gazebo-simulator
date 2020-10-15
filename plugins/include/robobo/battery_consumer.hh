#ifndef GAZEBO_BATTERY_CONSUMER_H
#define GAZEBO_BATTERY_CONSUMER_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include <boost/thread/mutex.hpp>
#include "robobo_msgs/SetLoad.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

namespace gazebo
{
    class GAZEBO_VISIBLE BatteryConsumerPlugin : public ModelPlugin
    {
    // Constructor
    public: BatteryConsumerPlugin();

    public: ~BatteryConsumerPlugin();

    // Inherited from ModelPlugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: virtual void Init();

    public: virtual void Reset();

    public: bool SetConsumerPowerLoad(robobo_msgs::SetLoad::Request& req,
                                      robobo_msgs::SetLoad::Response& res);

    // Connection to the World Update events.
    protected: event::ConnectionPtr updateConnection;

    protected: physics::WorldPtr world;

    protected: physics::PhysicsEnginePtr physics;

    protected: physics::ModelPtr model;

    protected: physics::LinkPtr link;

    protected: sdf::ElementPtr sdf;

    // Battery
    private: common::BatteryPtr battery;

    // Consumer identifier
    private: int32_t consumerId;

    protected: double powerLoad;

    // This node is for ros communications
    protected: std::unique_ptr<ros::NodeHandle> rosNode;

    protected: ros::ServiceServer set_power_load;

    protected: boost::mutex lock;

    };

}

#endif //GAZEBO_BATTERY_CONSUMER_H
