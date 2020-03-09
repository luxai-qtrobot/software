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

#ifndef qt_gesture_controller_H
#define qt_gesture_controller_H

#include <map>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <std_msgs/String.h>
#include <controller_interface/controller.h>
#include <qt_motor/qt_motor.h>

#include <qt_gesture_controller/gesture_record.h>
#include <qt_gesture_controller/gesture_stop.h>
#include <qt_gesture_controller/gesture_save.h>
#include <qt_gesture_controller/gesture_play.h>
#include <qt_gesture_controller/gesture_list.h>

namespace qt_gesture_controller
{
  enum State { waiting, recording, playing, stopped, timeout, record_requested, stop_requested };

  typedef std::map<std::string, std::vector<double> > JointsPosition;
  typedef std::map<std::string, std::vector<double> >::iterator JointsPosition_itr;

  struct RecordData {
      std::vector<std::string> parts;
      std::vector<ros::Time> stamps;
      JointsPosition positions;
  };

  class QTGestureController : public controller_interface::Controller<QTMotorInterface>
  {
  public:
      /**
       * @brief QTGestureController
       */
      QTGestureController();

      /**
       * @brief ~QTGestureController
       */
      ~QTGestureController();

    //bool init(QTMotorInterface* hw, ros::NodeHandle &n);
    bool init(QTMotorInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time) { ROS_INFO("QTGestureController: starting"); }
    void stopping(const ros::Time& time) { ROS_INFO("QTGestureController: stoping");}

  private:
    bool recordCB(qt_gesture_controller::gesture_record::Request  &req,
                          qt_gesture_controller::gesture_record::Response &res);
    bool stopCB(qt_gesture_controller::gesture_stop::Request  &req,
                          qt_gesture_controller::gesture_stop::Response &res);
    bool saveCB(qt_gesture_controller::gesture_save::Request  &req,
                          qt_gesture_controller::gesture_save::Response &res);
    bool playCB(qt_gesture_controller::gesture_play::Request  &req,
                          qt_gesture_controller::gesture_play::Response &res);
    bool listCB(qt_gesture_controller::gesture_list::Request  &req,
                          qt_gesture_controller::gesture_list::Response &res);

    void playSubCallback(const std_msgs::String::ConstPtr& msg);
    bool play(std::string name, float speed, bool blocking=true);
    bool playGesture(std::string name, float speed);


  private:
    std::string gesturePath;
    double initial_delay;
    QTMotorInterface *hw;
    std::vector<hardware_interface::JointStateHandle> jointStateLarm;
    std::vector<hardware_interface::JointStateHandle> jointStateRarm;
    std::vector<hardware_interface::JointStateHandle> jointStateHead;

    RecordData data;
    State state;
    //qt_gesture_controller::record::Request recordRequest;


    ros::ServiceServer serviceRecord;
    ros::ServiceServer serviceStop;
    ros::ServiceServer serviceSave;
    ros::ServiceServer servicePlay;
    ros::ServiceServer serviceList;
    ros::Subscriber subPlay;
    boost::mutex mutexRecord;
    boost::thread* threadPlay;

  };

}

#endif
