#ifndef GAZEBO_BATTERY_DISCHARGE_H
#define GAZEBO_BATTERY_DISCHARGE_H

#include <map>
#include <string>
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"
#include <boost/thread/mutex.hpp>
#include "std_msgs/Bool.h"

namespace gazebo
{
    /// \brief A plugin that simulate BRASS CP1 battery model: discharge and charge according to power models
    class GAZEBO_VISIBLE BatteryPlugin : public ModelPlugin
    {
    /// \brief Constructor
    public: BatteryPlugin();

    public: ~BatteryPlugin();

    // Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: virtual void Init();

    public: virtual void Reset();

    private: double OnUpdateVoltage(const common::BatteryPtr &_battery);

    // Connection to the World Update events.
    protected: event::ConnectionPtr updateConnection;

    protected: physics::WorldPtr world;

    protected: physics::PhysicsEnginePtr physics;

    protected: physics::ModelPtr model;

    protected: physics::LinkPtr link;

    protected: common::BatteryPtr battery;

    protected: sdf::ElementPtr sdf;


    // E(t) = e0 + e1* Q(t)/c
    protected: double et;
    protected: double e0;
    protected: double e1;

    // Initial battery charge in Ah.
    protected: double q0;

    // Charge rate in A
    // More description about charge rate: http://batteriesbyfisher.com/determining-charge-time
    protected: double qt;

    // Battery capacity in Ah.
    protected: double c;

    // Battery inner resistance in Ohm
    protected: double r;

    // Current low-pass filter characteristic time in seconds.
    protected: double tau;

    // Raw battery current in A.
    protected: double iraw;

    // Smoothed battery current in A.
    protected: double ismooth;

    // Instantaneous battery charge in Ah.
    protected: double q;

    // This node is for ros communications
    protected: std::unique_ptr<ros::NodeHandle> rosNode;

    protected: ros::Publisher charge_state;

    protected: boost::mutex lock;

    protected: double sim_time_now;

    };
}

#endif //GAZEBO_BATTERY_DISCHARGE_H
