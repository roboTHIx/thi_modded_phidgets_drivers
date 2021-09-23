/*
 * Copyright (c) 2019, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "phidgets_api/motors.h"
#include "phidgets_motors/motors_ros_i.h"

namespace phidgets {

MotorsRosI::MotorsRosI(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    int serial_num;
    if (!nh_private_.getParam("serial", serial_num))
    {
        serial_num = -1;  // default open any device
    }

    int hub_port;
    if (!nh_private.getParam("hub_port", hub_port))
    {
        hub_port = 0;  // only used if the device is on a VINT hub_port
    }
    
    std::string topicname_prefix;
    nh_private.getParam("topicname_prefix", topicname_prefix);
    if(topicname_prefix == "")
    {
        topicname_prefix = "motor00";
    }

    int data_interval_ms;
    if (!nh_private.getParam("data_interval_ms", data_interval_ms))
    {
        data_interval_ms = 256;
    }

    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 0;
    }

    std::string fan_mode;
    nh_private.getParam("fan_mode", fan_mode);
    if(fan_mode == "")
    {
        fan_mode = "AUTO";
    }

    double acceleration;
    if (!nh_private.getParam("acceleration", acceleration))
    {
        acceleration = 1.0;
    }

    double braking_strength;
    if (!nh_private.getParam("braking_strength", braking_strength))
    {
        braking_strength = 0.0;
    }

    double current_limit;
    if (!nh_private.getParam("current_limit", current_limit))
    {
        current_limit = 2.0;
    }

    double current_regulator_gain;
    if (!nh_private.getParam("current_regulator_gain", current_regulator_gain))
    {
        current_regulator_gain = 10.0;
    }


    ROS_INFO("Starting Phidgets Motor Topic: %s, connecting to Serial %d, port %d",
        topicname_prefix.c_str(), serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(motor_mutex_);

    int n_motors;
    try
    {
        motors_ = std::make_unique<Motors>(
            serial_num, hub_port, false,
            std::bind(&MotorsRosI::dutyCycleChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotorsRosI::backEMFChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        ROS_INFO("Connected, found %d motors", motors_->getMotorCount());

        n_motors = motors_->getMotorCount();
        motor_vals_.resize(n_motors);
        for (int i = 0; i < n_motors; i++)
        {
            std::string topicname;
            std::string pubtopic;
            std::string backemftopic;

            if(n_motors == 1)
            {
                topicname    = topicname_prefix + "/set_duty_cycle";
                pubtopic     = topicname_prefix + "/get_duty_cycle";
                backemftopic = topicname_prefix + "/get_back_emf";
            }
            else
            {
                // This need some time to make it less shitty...
                topicname    = topicname_prefix + "/set_duty_cycle" + "00";
                pubtopic     = topicname_prefix + "/get_duty_cycle" + "00";
                backemftopic = topicname_prefix + "/get_back_emf" + "00";
            }

            motor_vals_[i].duty_cycle_sub       = std::make_unique<DutyCycleSetter>(motors_.get(), i, nh, topicname);
            motor_vals_[i].duty_cycle_pub       = nh_.advertise<std_msgs::Float64>(pubtopic, 1);
            motor_vals_[i].back_emf_pub         = nh_.advertise<std_msgs::Float64>(backemftopic, 1);
            motor_vals_[i].last_duty_cycle_val  = motors_->getDutyCycle(i);
            // motor_vals_[i].last_back_emf_val    = motors_->getBackEMF(i);

            motors_->setDataInterval(i, data_interval_ms);

            if(fan_mode == "ON")
                motors_->setFanMode(i, FAN_MODE_ON);
            else if(fan_mode == "OFF")
                motors_->setFanMode(i, FAN_MODE_OFF);
            else if(fan_mode == "AUTO")
                motors_->setFanMode(i, FAN_MODE_AUTO);
            else
                motors_->setFanMode(i, FAN_MODE_AUTO);

            motors_->setAcceleration(i, acceleration);

            motors_->setBraking(i, braking_strength);

            motors_->setCurrentLimit(i, current_limit);

            motors_->setCurrentRegulatorGain(i, current_regulator_gain);
        }
    } catch (const Phidget22Error& err)
    {
        ROS_ERROR("Motors: %s", err.what());
        throw;
    }

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &MotorsRosI::timerCallback, this);
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        for (int i = 0; i < n_motors; ++i)
        {
            publishLatestDutyCycle(i);
            publishLatestBackEMF(i);
        }
    }
}

void MotorsRosI::publishLatestDutyCycle(int index)
{
    std_msgs::Float64 msg;
    msg.data = motor_vals_[index].last_duty_cycle_val;
    motor_vals_[index].duty_cycle_pub.publish(msg);
}

void MotorsRosI::publishLatestBackEMF(int index)
{
    std_msgs::Float64 backemf_msg;
    backemf_msg.data = motor_vals_[index].last_back_emf_val;
    motor_vals_[index].back_emf_pub.publish(backemf_msg);
}

void MotorsRosI::timerCallback(const ros::TimerEvent& /* event */)
{
    std::lock_guard<std::mutex> lock(motor_mutex_);
    for (int i = 0; i < static_cast<int>(motor_vals_.size()); ++i)
    {
        publishLatestDutyCycle(i);
        publishLatestBackEMF(i);
    }
}

DutyCycleSetter::DutyCycleSetter(Motors* motors, int index, ros::NodeHandle nh,
                                 const std::string& topicname)
    : motors_(motors), index_(index)
{
    subscription_ =
        nh.subscribe(topicname, 1, &DutyCycleSetter::setMsgCallback, this);
}

void DutyCycleSetter::setMsgCallback(const std_msgs::Float64::ConstPtr& msg)
{
    try
    {
        motors_->setDutyCycle(index_, msg->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        // If the data was wrong, the lower layers will throw an exception; just
        // catch and ignore here so we don't crash the node.
    }
}

void MotorsRosI::dutyCycleChangeCallback(int channel, double duty_cycle)
{
    if (static_cast<int>(motor_vals_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        motor_vals_[channel].last_duty_cycle_val = duty_cycle;

        if (publish_rate_ <= 0)
        {
            publishLatestDutyCycle(channel);
        }
    }
}

void MotorsRosI::backEMFChangeCallback(int channel, double back_emf)
{
    if (static_cast<int>(motor_vals_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        motor_vals_[channel].last_back_emf_val = back_emf;

        if (publish_rate_ <= 0)
        {
            publishLatestBackEMF(channel);
        }
    }
}

}  // namespace phidgets
