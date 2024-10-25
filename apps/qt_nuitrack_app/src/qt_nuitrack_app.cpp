#include <ctime>
#include <cstdlib>
#include <sstream>

#include "qt_nuitrack_app/qt_nuitrack_app.h"

using namespace tdv::nuitrack;

using namespace qt_nuitrack_app;

QTNuitrackApp::QTNuitrackApp(ros::NodeHandle& nh) {


    // Initialize Nuitrack
    ROS_INFO_STREAM("Initializing Nuitrack...");
    try {
        Nuitrack::init("");
    }
    catch (const Exception& e) {
        ROS_ERROR_STREAM("Can not initialize Nuitrack (ExceptionType: " << e.type() << ")");
        ros::shutdown();
    }
    
    if(!nh.getParam("/qt_nuitrack_app/image_width", image_width))
        image_width = 640;
        
    if(!nh.getParam("/qt_nuitrack_app/image_height", image_height))
        image_height = 480;


    enable_face = false;
    if(!nh.getParam("/qt_nuitrack_app/enable_face", enable_face))
        enable_face = false;

    enable_depth = false;
    if(!nh.getParam("/qt_nuitrack_app/enable_depth", enable_depth))
        enable_depth = false;
    
    enable_landmarks = false;
    if(!nh.getParam("/qt_nuitrack_app/enable_landmarks", enable_landmarks))
        enable_landmarks = false;

    tdv::nuitrack::Nuitrack::setConfigValue( "Realsense2Module.RGB.ProcessWidth", std::to_string(image_width));
    tdv::nuitrack::Nuitrack::setConfigValue( "Realsense2Module.RGB.ProcessHeight", std::to_string(image_height));
    tdv::nuitrack::Nuitrack::setConfigValue( "Faces.ToUse", "true" );    
    if(enable_depth) {     
        tdv::nuitrack::Nuitrack::setConfigValue( "DepthProvider.Depth2ColorRegistration", "true" );   
    }

    
        
    // create Gesture
    gestureRecognizer = tdv::nuitrack::GestureRecognizer::create();
    gestureRecognizer->connectOnNewGestures(std::bind( &QTNuitrackApp::onNewGestures, this, std::placeholders::_1));

    // Create Skeleton
    skeletonTracker = tdv::nuitrack::SkeletonTracker::create();
    skeletonTracker->connectOnUpdate(std::bind(&QTNuitrackApp::onSkeletonUpdate, this, std::placeholders::_1));

    // Create Hand
    handTracker = HandTracker::create();
    handTracker->connectOnUpdate(std::bind(&QTNuitrackApp::onHandUpdate, this, std::placeholders::_1));


    // Create color sensor
    colorSensor = tdv::nuitrack::ColorSensor::create();
    colorSensor->connectOnNewFrame(std::bind( &QTNuitrackApp::onNewColorFrame, this, std::placeholders::_1));


    // Create depth sensor
    if(enable_depth) {     
        depthSensor = tdv::nuitrack::DepthSensor::create();
        tdv::nuitrack::OutputMode outputMode = depthSensor->getOutputMode();
        tdv::nuitrack::OutputMode colorOutputMode = colorSensor->getOutputMode();
        if ((colorOutputMode.xres != outputMode.xres) || (colorOutputMode.yres != outputMode.yres)) {
            ROS_WARN("WARNING! DEPTH AND COLOR SIZE NOT THE SAME!");
        }

        // Use depth as the frame size
        int frame_width = outputMode.xres;
        int frame_height = outputMode.yres;
        
        // Point Cloud message (includes depth and color)
        int numpoints = frame_width * frame_height;
        cloud_msg.header.frame_id = "camera_depth_frame";
        cloud_msg.width  = numpoints;
        cloud_msg.height = 1;
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false; // there may be invalid points

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2,"xyz","rgb");
        modifier.resize(numpoints);

        depthSensor->connectOnNewFrame(std::bind(&QTNuitrackApp::onNewDepthFrame, this, std::placeholders::_1));
    }

    // Start Nuitrack
    ROS_INFO_STREAM("Starting Nuitrack...");
    try {
        Nuitrack::run();
    }
    catch (const Exception& e) {
        ROS_ERROR_STREAM("Can not start Nuitrack (ExceptionType: " << e.type() << ")");
        ros::shutdown();
    }

    serviceSuspend = nh.advertiseService("qt_nuitrack_app/suspend", &QTNuitrackApp::suspendCB, this);
    colorImagePub = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 10);
    gesturePub = nh.advertise<qt_nuitrack_app::Gestures>("/qt_nuitrack_app/gestures", 1);
    handPub = nh.advertise<qt_nuitrack_app::Hands>("/qt_nuitrack_app/hands", 1);
    skeletonPub = nh.advertise<qt_nuitrack_app::Skeletons>("/qt_nuitrack_app/skeletons", 10);

    if(enable_face) {
        facePub = nh.advertise<qt_nuitrack_app::Faces>("/qt_nuitrack_app/faces", 1);
    }

    if(enable_depth) {
        depthImagePub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 10);
        depthCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/cloud", 10);
    }

    if(!nh.getParam("/qt_nuitrack_app/main_frame_rate", main_frame_rate))
        main_frame_rate  = 30;
    if(!nh.getParam("/qt_nuitrack_app/face_frame_rate", face_frame_rate))
        face_frame_rate  = 2;
    ROS_INFO_STREAM("Satrting main loop:");
    ROS_INFO_STREAM("\tMain frame rate: "<<main_frame_rate);
    std::string str_bool = (enable_face) ? "yes" : "no";
    ROS_INFO_STREAM("\tFacial features enabled: " << str_bool);
    if(enable_face){
        ROS_INFO_STREAM("\tFace frame rate: "<<face_frame_rate);
    }
    ROS_INFO_STREAM("\tCamera image size "<<image_width<<"x"<<image_height);
    str_bool = (enable_depth) ? "yes" : "no";
    ROS_INFO_STREAM("\tDepth camera enabled: " << str_bool);

    nuitrackTimer = nh.createTimer(ros::Duration(1.0/main_frame_rate),
                                   &QTNuitrackApp::nuitrackTimerCallback, this);

}

QTNuitrackApp::~QTNuitrackApp() {
    // Release Nuitrack
    try {
        ROS_INFO_STREAM("Releasing Nuitrack...");
        Nuitrack::release();
    }
    catch (const Exception& e) {
        ROS_ERROR_STREAM("Nuitrack release failed (ExceptionType: " << e.type() << ")");
    }
}



void QTNuitrackApp::nuitrackTimerCallback(const ros::TimerEvent& event) {
    //ROS_INFO("running ....");
    // Update Tracker
    try{
        tdv::nuitrack::Nuitrack::update();
        // Update Tracker        
        //tdv::nuitrack::Nuitrack::waitUpdate(handTracker);
    }
    catch( const tdv::nuitrack::LicenseNotAcquiredException& ex ){
        ROS_ERROR_STREAM("Nuitrack License timeout");
        ros::shutdown();
    }

    if(enable_face) {
        static ros::Time prev_update_face = ros::Time::now();
        ros::Duration diff = ros::Time::now() - prev_update_face;
        if(diff >= ros::Duration(1.0/face_frame_rate)) {
            onNewFace();
            prev_update_face = ros::Time::now();
        }
    }
}

void QTNuitrackApp::onNewColorFrame(tdv::nuitrack::RGBFrame::Ptr frame) {
    int width = frame->getCols();
    int height = frame->getRows();
    //ROS_INFO_STREAM(width << ", " << height);

    sensor_msgs::Image color_msg;
    color_msg.header.stamp = ros::Time::now();
    color_msg.header.frame_id = "camera_color_frame";        
    color_msg.height = height;
    color_msg.width = width;
    color_msg.encoding = "bgr8";    //sensor_msgs::image_encodings::TYPE_16UC1;
    color_msg.is_bigendian = false;
    color_msg.step = 3 * width;     // sensor_msgs::ImagePtr row step size
    size_t st0 = (color_msg.step * height);
    color_msg.data.resize(st0);
    memcpy(&color_msg.data[0], frame->getData(), st0);

    // Publish color frame
    colorImagePub.publish(color_msg);
}


void QTNuitrackApp::onNewDepthFrame(tdv::nuitrack::DepthFrame::Ptr frame) {
    int width = frame->getCols();
    int height = frame->getRows();
    //ROS_INFO_STREAM(width << ", " << height);
    const uint16_t* depthPtr = frame->getData();

    sensor_msgs::Image depth_msg;
    depth_msg.header.stamp = ros::Time::now();
    depth_msg.header.frame_id = "camera_depth_frame";        
    depth_msg.height = height;
    depth_msg.width = width;
    depth_msg.encoding = "bgr8";    //sensor_msgs::image_encodings::TYPE_16UC1;
    depth_msg.is_bigendian = false;
    depth_msg.step = 3 * width;     // sensor_msgs::ImagePtr row step size
    
    // Point Cloud message
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");

    for (size_t row = 0; row < height; ++row) {
        for (size_t col = 0; col < width; ++col ) {
            
            uint16_t fulldepthValue = *(depthPtr + col);
            uint16_t depthValue = *(depthPtr + col) >> 5;
            // RGB are all the same for depth (monochrome)
            depth_msg.data.push_back(depthValue); 
            depth_msg.data.push_back(depthValue);
            depth_msg.data.push_back(depthValue);

            //store xyz in point cloud, transforming from image coordinates, (Z Forward to X Forward)
            Vector3 cloud_point = depthSensor->convertProjToRealCoords(col, row, fulldepthValue );            
            float X_World = cloud_point.x / 1000.0; // mm to meters
            float Y_World = cloud_point.y / 1000.0;
            float Z_World = cloud_point.z / 1000.0; 
            
            *out_x = Z_World;
            *out_y = -X_World;
            *out_z = Y_World; 
            ++out_x;
            ++out_y;
            ++out_z;
        }
        depthPtr += width; // Next row
    }

    // Publish color frame
    depthImagePub.publish(depth_msg);

    // Publish depth cloud
    cloud_msg.header.stamp = ros::Time::now();
    depthCloudPub.publish(cloud_msg);
}


void QTNuitrackApp::onNewFace() {
    json = parser::parse( tdv::nuitrack::Nuitrack::getInstancesJson());
    
    qt_nuitrack_app::Faces msgFace;
    for( const parser::Human& human : json.humans ){
        if( !human.face ){
            continue;
        }
        uint32_t id = 0;
        const parser::Face& face = human.face.get();
        qt_nuitrack_app::FaceInfo faceInfo;
        qt_nuitrack_app::LandmarkInfo landmarkInfo;
        faceInfo.id = human.id;
        faceInfo.gender = face.gender;
        faceInfo.age_type = face.age.type;
        faceInfo.age_years = face.age.years;
        faceInfo.emotion_neutral = face.emotions.neutral;
        faceInfo.emotion_angry = face.emotions.angry;
        faceInfo.emotion_happy = face.emotions.happy;
        faceInfo.emotion_surprise = face.emotions.surprise;
        faceInfo.rectangle.push_back(face.rectangle.x);
        faceInfo.rectangle.push_back(face.rectangle.y);
        faceInfo.rectangle.push_back(face.rectangle.width);
        faceInfo.rectangle.push_back(face.rectangle.height);
        faceInfo.left_eye.push_back(face.eyes.left_eye.x);
        faceInfo.left_eye.push_back(face.eyes.left_eye.y);
        faceInfo.right_eye.push_back(face.eyes.right_eye.x);
        faceInfo.right_eye.push_back(face.eyes.right_eye.y);
        faceInfo.angles.push_back(face.angles.yaw);
        faceInfo.angles.push_back(face.angles.pitch);
        faceInfo.angles.push_back(face.angles.roll);
        if(enable_landmarks){
            for( const parser::Vec& landmark : face.landmarks ){
                landmarkInfo.id = id++;
                landmarkInfo.x = landmark.x;
                landmarkInfo.y = landmark.y;
                faceInfo.landmarks.push_back(landmarkInfo);
                // ROS_INFO_STREAM("\tlandmark["<< index++ <<"]:("<<landmark.x<<", "<<landmark.y <<")");
            }
        }    

        
        msgFace.faces.push_back(faceInfo);
        
        //ROS_INFO_STREAM("Human "<<human.id);
        //ROS_INFO_STREAM("\tGender: "<<face.gender);
        //ROS_INFO_STREAM("\tAge: "<<face.age.years);
        //ROS_INFO_STREAM("\tAge type: "<<face.age.type);
        //ROS_INFO_STREAM("\tNeutral: "<<face.emotions.neutral);
        //ROS_INFO_STREAM("\tAngry: "<<face.emotions.angry);
        //ROS_INFO_STREAM("\tHappy: "<<face.emotions.happy);
        //ROS_INFO_STREAM("\tSurprise: "<<face.emotions.surprise);
    }
    if(msgFace.faces.size())
        facePub.publish(msgFace);
}

void QTNuitrackApp::onSkeletonUpdate(tdv::nuitrack::SkeletonData::Ptr skeletonData) {
    qt_nuitrack_app::Skeletons msgSkeleton;
    const std::vector<tdv::nuitrack::Skeleton> skeletons = skeletonData->getSkeletons();
    for( const tdv::nuitrack::Skeleton& skeleton : skeletons ){

        //const tdv::nuitrack::Joint& joint = joints[JOINT_HEAD];
        //if( joint.confidence < 0.2 ){
        //    ROS_INFO_STREAM("JOINT_HEAD:");
        //    ROS_INFO_STREAM("\treal: ("<<joint.real.x <<", "<<joint.real.y <<", "<<joint.real.z <<")");
        //}

        qt_nuitrack_app::SkeletonInfo skeletonInfo;
        skeletonInfo.id = skeleton.id;
        const std::vector<tdv::nuitrack::Joint> joints = skeleton.joints;
        for( const tdv::nuitrack::Joint& joint : joints ){

            qt_nuitrack_app::JointInfo jointInfo;
            jointInfo.type = joint.type;
            jointInfo.confidence = joint.confidence;
            jointInfo.real.push_back(joint.real.x);
            jointInfo.real.push_back(joint.real.y);
            jointInfo.real.push_back(joint.real.z);
            jointInfo.projection.push_back(joint.proj.x);
            jointInfo.projection.push_back(joint.proj.y);
            jointInfo.projection.push_back(joint.proj.z);
            jointInfo.orientation.resize(9);
            std::copy(joint.orient.matrix, joint.orient.matrix+9, jointInfo.orientation.begin());
            skeletonInfo.joints.push_back(jointInfo);
        }
        msgSkeleton.skeletons.push_back(skeletonInfo);
    }
    if(msgSkeleton.skeletons.size())
        skeletonPub.publish(msgSkeleton);
}

void QTNuitrackApp::onHandUpdate(tdv::nuitrack::HandTrackerData::Ptr handData) {
    qt_nuitrack_app::Hands msgHands;
    const std::vector<UserHands> users_hands = handData->getUsersHands();
    for(const UserHands& user_hands : users_hands ){
        qt_nuitrack_app::HandInfo handInfo;        
        const tdv::nuitrack::Hand::Ptr left_hand = user_hands.leftHand;
        const tdv::nuitrack::Hand::Ptr right_hand = user_hands.rightHand;
        handInfo.id = user_hands.userId;
        if(right_hand) {
            handInfo.right_projection.push_back(right_hand->x);
            handInfo.right_projection.push_back(right_hand->y);            
            handInfo.right_real.push_back(right_hand->xReal);
            handInfo.right_real.push_back(right_hand->yReal);
            handInfo.right_real.push_back(right_hand->zReal);
            handInfo.right_click = right_hand->click;
            handInfo.right_pressure = right_hand->pressure;
            //ROS_INFO_STREAM("UID:"<< id << "\tR: "<<right_hand->xReal <<", "
           //             <<right_hand->yReal <<", "<<right_hand->zReal);
        }
        if(left_hand) {
            handInfo.left_projection.push_back(left_hand->x);
            handInfo.left_projection.push_back(left_hand->y);
            handInfo.left_real.push_back(left_hand->xReal);
            handInfo.left_real.push_back(left_hand->yReal);
            handInfo.left_real.push_back(left_hand->zReal);
            handInfo.left_click = left_hand->click;
            handInfo.left_pressure = left_hand->pressure;
        }
        if(right_hand || left_hand)
            msgHands.hands.push_back(handInfo);
    }
    if(msgHands.hands.size())
        handPub.publish(msgHands);
}

void QTNuitrackApp::onNewGestures(const tdv::nuitrack::GestureData::Ptr gesture_data) {
    qt_nuitrack_app::Gestures msgGestures;
    const std::vector<tdv::nuitrack::Gesture> gestures = gesture_data->getGestures();
    for( const tdv::nuitrack::Gesture& gesture : gestures ){
        //ROS_INFO_STREAM(gesture.userId << ": " << type2string(gesture.type));
        qt_nuitrack_app::GestureInfo gestureInfo;
        gestureInfo.name = type2string(gesture.type);
        if(gestureInfo.name.size()) {
            gestureInfo.id = gesture.userId;
            msgGestures.gestures.push_back(gestureInfo);
        }
    }
    if(msgGestures.gestures.size())
        gesturePub.publish(msgGestures);
}

// Convert Gesture Type to String
std::string QTNuitrackApp::type2string( const tdv::nuitrack::GestureType gesture_type ) {
    switch( gesture_type ){
        case tdv::nuitrack::GestureType::GESTURE_WAVING:
            return "WAVING";
        case tdv::nuitrack::GestureType::GESTURE_SWIPE_LEFT:
            return "SWIPE LEFT";
        case tdv::nuitrack::GestureType::GESTURE_SWIPE_RIGHT:
            return "SWIPE RIGHT";
        case tdv::nuitrack::GestureType::GESTURE_SWIPE_UP:
            return "SWIPE UP";
        case tdv::nuitrack::GestureType::GESTURE_SWIPE_DOWN:
            return "SWIPE DOWN";
        case tdv::nuitrack::GestureType::GESTURE_PUSH:
            return "PUSH";
        default:
            throw std::runtime_error( "failed can't convert gesture type to string" );
            return "";
    }
}

bool QTNuitrackApp::suspend(bool flag) {
  if(flag)
      nuitrackTimer.stop();
  else
      nuitrackTimer.start();
  return true;
}

