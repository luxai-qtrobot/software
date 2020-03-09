/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef QT_MOTORS_CONTROLLER_H
#define QT_MOTORS_CONTROLLER_H

#include <realtime_tools/realtime_publisher.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <controller_interface/controller.h>
#include <qt_motor/qt_motor.h>

#include <qt_motors_controller/set_control_mode.h>
#include <qt_motors_controller/set_velocity.h>
#include <qt_motors_controller/home.h>
#include <qt_motors_controller/MotorState.h>

namespace qt_motors_controller
{
  class QTMotorsController : public controller_interface::Controller<QTMotorInterface>
  {
  public:
    //bool init(QTMotorInterface* hw, ros::NodeHandle &n);
    bool init(QTMotorInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

  private:
    bool setControlModeCB(qt_motors_controller::set_control_mode::Request  &req,
                          qt_motors_controller::set_control_mode::Response &res);
    bool setVelocityCB(qt_motors_controller::set_velocity::Request  &req,
                          qt_motors_controller::set_velocity::Response &res);
    bool homeCB(qt_motors_controller::home::Request  &req,
                          qt_motors_controller::home::Response &res);
  private:
    QTMotorInterface *hw;
    ros::ServiceServer serviceSetControlMode;
    ros::ServiceServer serviceSetVelocity;
    ros::ServiceServer serviceHome;
    boost::shared_ptr<realtime_tools::RealtimePublisher<qt_motors_controller::MotorState> > realtime_pub_;
    ros::Time last_publish_time_;
    double ms_publish_rate_;
    unsigned int num_hw_joints_;
  };

}

#endif
