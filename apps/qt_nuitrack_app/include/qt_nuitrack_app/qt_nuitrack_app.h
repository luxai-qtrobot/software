/*
 * Copyright (C) 2016 Lux Future Robotic
 * Author:  Ali Paikan
 * email:   ali.paikan@luxai.com
 * website: www.luxai.com
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#ifndef _qt_nuitrack_app_
#define _qt_nuitrack_app_


#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/Image.h>


#include "qt_nuitrack_app/suspend.h"
#include "qt_nuitrack_app/Faces.h"
#include "qt_nuitrack_app/Gestures.h"
#include "qt_nuitrack_app/Hands.h"
#include "qt_nuitrack_app/Skeletons.h"


#include <nuitrack/Nuitrack.h>
#include "parser.h"

namespace qt_nuitrack_app {

class QTNuitrackApp {
public:
    QTNuitrackApp(ros::NodeHandle& nh);

    virtual ~QTNuitrackApp();

    bool suspendCB(qt_nuitrack_app::suspend::Request  &req,
                   qt_nuitrack_app::suspend::Response &res) {
                        res.status = suspend(req.flag);
                        return true;
    }

protected:
    virtual bool suspend(bool flag);
    void nuitrackTimerCallback(const ros::TimerEvent& event);
    void onNewGestures(const tdv::nuitrack::GestureData::Ptr gesture_data);
    void onNewColorFrame(tdv::nuitrack::RGBFrame::Ptr frame);
    void onHandUpdate(tdv::nuitrack::HandTrackerData::Ptr handData);
    void onSkeletonUpdate(tdv::nuitrack::SkeletonData::Ptr skeletonData);
    void onNewFace();

private:
    std::string type2string( const tdv::nuitrack::GestureType gesture_type );
private:
    // Nuitracker
    //sensor_msgs::PointCloud2 cloud_msg_; // color and depth point cloud
    tdv::nuitrack::ColorSensor::Ptr colorSensor;
    tdv::nuitrack::HandTracker::Ptr handTracker;
    tdv::nuitrack::HandTrackerData::Ptr handData;
    tdv::nuitrack::GestureRecognizer::Ptr gestureRecognizer;
    tdv::nuitrack::GestureData::Ptr gestureData;
    tdv::nuitrack::SkeletonTracker::Ptr skeletonTracker;
    ros::Timer nuitrackTimer;
    parser::JSON json;

    // nuitracker services and publishers
    ros::ServiceServer serviceSuspend;
    ros::Publisher colorImagePub;
    ros::Publisher facePub;
    ros::Publisher gesturePub;
    ros::Publisher handPub;
    ros::Publisher skeletonPub;

    // params
    double main_frame_rate;
    double face_frame_rate;
};

}

#endif //_qt_nuitrack_app_
