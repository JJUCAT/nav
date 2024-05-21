#include <dock_visual_perception/VisualPerception.h>
#include <image_transport/image_transport.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>

// Inorder to load dynamically.
PLUGINLIB_EXPORT_CLASS(dock_visual_perception::VisualPerception,
                       navit_auto_dock::plugins::ApproachDockPerception)

namespace dock_visual_perception
{
    //Initialize some parameters
    void VisualPerception::initialize(const std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf)
    {
        ros::NodeHandle nh,pnh("~/" + name);
        //set marker values.
        pnh.param("marker_size",marker_size_,9.0);
        pnh.param("max_new_marker_error",max_new_marker_error_,0.08);
        pnh.param("max_track_error",max_track_error_,0.2);
        pnh.param("max_frequency",max_frequency_,20.0);
        pnh.param("marker_resolution",marker_resolution_,5);
        pnh.param("marker_margin",marker_margin_,2);
        pnh.param("input_id",input_id_, 1);
        confidence_ = START_;
        cam_info_topic_ = "/camera/camera_info";
        cam_image_topic_ = "/camera/image";
        pnh.param("cam_info_topic", cam_info_topic_, cam_info_topic_);
        pnh.param("cam_image_topic",cam_image_topic_,cam_image_topic_);
        pnh.param("roll",roll_, 0.0);
        pnh.param("pitch",pitch_, 0.0);
        pnh.param("yaw",yaw_, 0.0);

        //set frame value.
        output_frame_ = "/odom";
        //generate the subscibers.
        show_msg_ = false;
        image_transport::ImageTransport it_(nh);
        image_sub_ = it_.subscribe(cam_image_topic_,1,&VisualPerception::process,this);
        qr_code_pub_= pnh.advertise<geometry_msgs::PoseStamped>("qr_code_detected",1);
        cam_ = new Camera(nh,cam_info_topic_);
        //In the first time, the camera_info and image topic need more time to configure the message queue.
        //we set the default value : 0.5
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        //Get the transforms between two frames.
        tf_buffer_ = tf;
    }

    bool VisualPerception::start(const geometry_msgs::PoseStamped &target_pose){
        return true;
    }

    bool VisualPerception::stop(){
        return true;
    }
    
    bool VisualPerception::getPose(geometry_msgs::PoseStamped &pose) 
    {   
        bool res = false;
        if(cam_->getCamInfo_)
        {
            ros::Duration(1 / max_frequency_).sleep();
            ros::spinOnce();
            switch(confidence_){
                case INVALID_: 
                    ROS_WARN_STREAM("The result is not confident.");
                    res = false;
                    break;
                case ERROR_: 
                    ROS_WARN_STREAM("Error.");
                    res = false;
                    break;
                case VALID_: 
                    ROS_INFO_STREAM("Successful.");
                    //TODO: visualization in rviz
                    pose = toRobotFrame(tagPoseMsg_);
                    res = true;
                    break;
                default:
                    res = false;
                    break;
            }
        }
        else{
            ROS_WARN_STREAM("Couldn't get the camera's information.");
            res = false;
        }
        return res;
    }

    void VisualPerception::process(const sensor_msgs::ImageConstPtr& image_msg)
    {
        // std::cout<<"process"<<std::endl;
        marker_detector_.SetMarkerSize(marker_size_, marker_resolution_, marker_margin_);
        try
        {
            // Convert the image
            cv_bridge::CvImagePtr cv_ptr_ =
                cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

            marker_detector_.Detect(cv_ptr_->image, cam_, true, true,
                                    max_new_marker_error_, max_track_error_, CVSEQ,
                                    true);
                                    
            for (size_t i = 0; i < marker_detector_.markers->size(); i++)
            {
                //Get the id.If the id does not match what we set, then we skip.
                int id = (*(marker_detector_.markers))[i].GetId();
                if(id != input_id_) continue;
                //Get the pose relative to the camera.
                Pose p = (*(marker_detector_.markers))[i].pose;
                double px = p.translation[0] / 100.0;
                double py = p.translation[1] / 100.0;
                double pz = p.translation[2] / 100.0;
                cv::Mat quat =cv::Mat(4, 1, CV_64F);
                p.GetQuaternion(quat);
                double qx = quat.at<double>(1,0); //p.quaternion[1];
                double qy = quat.at<double>(2,0); //p.quaternion[2];
                double qz = quat.at<double>(3,0); //p.quaternion[3];
                double qw = quat.at<double>(0,0); //p.quaternion[0];
                tf::Quaternion rotation(qx, qy, qz, qw);
                tf::Vector3 origin(px, py, pz);
                tf::Transform t(rotation, origin);
                tf::Vector3 markerOrigin(0, 0, 0);
                tf::Transform m(tf::Quaternion::getIdentity(), markerOrigin);
                tf::Transform markerPose = t * m;  // marker pose in the camera frame
                // as we can't see through markers, this one is false positive detection
                tf::Vector3 z_axis_cam = tf::Transform(rotation, tf::Vector3(0, 0, 0)) * tf::Vector3(0, 0, 1);
                if (z_axis_cam.z() > 0) {
                    confidence_ = INVALID_;
                    continue;
                }
                else confidence_ = VALID_;
                // Show the information of id and pose.
                if(show_msg_)
                    ROS_INFO("Id: %02i In camera frame: qx %f qy %f qz %f qw %f x %f y %f z %f",
                                id, qx, qy, qz, qw, px, py, pz);
                else show_msg_ = true;
                // Get the pose of the tag in the camera frame, then the output frame.
                tf::poseTFToMsg(markerPose,markerPoseMsg_.pose);
                markerPoseMsg_.header.frame_id = image_msg->header.frame_id;
                markerPoseMsg_.header.stamp = image_msg->header.stamp;
                qr_code_pub_.publish(markerPoseMsg_);
                // Transform from camera frame to odom frame.
                try{
                    CamToOutputMsg_ = tf_buffer_->lookupTransform(output_frame_, image_msg->header.frame_id, ros::Time(0));
                    tf2::doTransform(markerPoseMsg_, tagPoseMsg_, CamToOutputMsg_);
                } 
                catch (tf2::TransformException const &ex) {
                    confidence_ = ERROR_;
                    ROS_WARN_STREAM("No transform data from camera frame to output frame ");
                }
                break;
            }
        }
        catch (const std::exception& e){
            ROS_ERROR("Error in callback: %s", e.what());
            confidence_ = ERROR_;
        }
    }

    geometry_msgs::PoseStamped VisualPerception::toRobotFrame(geometry_msgs::PoseStamped &pose){
        geometry_msgs::PoseStamped goal_pose;

        tf2::Transform odom_to_ori_tf, odom_to_goal_tf;
        tf2::fromMsg(pose.pose,odom_to_ori_tf);


        tf2::Transform ori_to_goal_tf;
        ori_to_goal_tf.setOrigin(tf2::Vector3(0,0,0));
        //ori_to_goal_tf.setRotation(tf2::Quaternion(pitch_,roll_,yaw_));//ROS_DEPRECATED Quaternion(const tf2Scalar& yaw, const tf2Scalar& pitch, const tf2Scalar& roll)
        tf2::Quaternion q;  
        q.setRPY(roll_, pitch_, yaw_); 
        ori_to_goal_tf.setRotation(q);

        odom_to_goal_tf=odom_to_ori_tf*ori_to_goal_tf;
        tf2::toMsg(odom_to_goal_tf, goal_pose.pose);
        goal_pose.header=pose.header;

        return goal_pose;

    }

};