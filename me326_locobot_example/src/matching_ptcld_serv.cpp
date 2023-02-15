#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

/*
//This block of Eigen functions aren't required in this script, 
but I personally include this on most applications so I have easy access 
to matrix functionatliy when needed (similar to python numpy). 
*/
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <numeric>

#include <map> //dictionary equivalent
#include<std_msgs/Header.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

//include the service for this package
#include <me326_locobot_example/PixtoPoint.h>


//Setup the class:
class Matching_Pix_to_Ptcld
{
public:
	Matching_Pix_to_Ptcld();

	// Make callback functions for subscribers
	void info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg);
	void depth_callback(const sensor_msgs::Image::ConstPtr& msg);
	void color_image_callback(const sensor_msgs::Image::ConstPtr& msg);
	bool service_callback(me326_locobot_example::PixtoPoint::Request &req, me326_locobot_example::PixtoPoint::Response &res);
	void camera_cube_locator_marker_gen();


  private:
	ros::NodeHandle nh;

  // Publisher declarations
	ros::Publisher image_color_filt_pub_;
	ros::Publisher camera_cube_locator_marker_;
	// Subscriber declarations
	ros::Subscriber cam_info_sub_;
	ros::Subscriber depth_sub_;
	ros::Subscriber rgb_image_sub_;
	// Rosservice
	ros::ServiceServer service_;
	//Variables
	std::vector<geometry_msgs::Point> point_3d_cloud_; //Point in pointcloud corresponding to desired pixel
	std::vector<std_msgs::ColorRGBA> point_color; // Point color
	std::vector<std_msgs::ColorRGBA> marker_color;
	std_msgs::Header header_;
	std::vector<geometry_msgs::Point> uv_pix_; //pixel index
	std::string color_image_topic_; // this string is over-written by the service request
	std::string depth_image_topic_; // this string is over-written by the service request
	std::string depth_img_camera_info_; // this string is over-written by the service request
	std::string registered_pt_cld_topic_; // this string is over-written by the service request
	image_geometry::PinholeCameraModel camera_model_; //Camera model, will help us with projecting the ray through the depth image
	bool depth_cam_info_ready_; //This will help us ensure we don't ask for a variable before its ready
	bool point_3d_ready_;
	bool p2d;
	bool p3d;
	// TF Listener
	tf2_ros::Buffer tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

Matching_Pix_to_Ptcld::Matching_Pix_to_Ptcld() 
{
	//Class constructor
	nh = ros::NodeHandle(); //This argument makes all topics internal to this node namespace. //takes the node name (global node handle), If you use ~ then its private (under the node handle name) /armcontroller/ *param*
	//this is how to setup the TF buffer in a class:
	tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
	//ROSparam set variables
	nh.param<std::string>("pt_srv_color_img_topic", color_image_topic_, "/locobot/camera/color/image_raw");
	nh.param<std::string>("pt_srv_depth_img_topic", depth_image_topic_, "/locobot/camera/aligned_depth_to_color/image_raw");
	nh.param<std::string>("pt_srv_depth_img_cam_info_topic", depth_img_camera_info_, "/locobot/camera/aligned_depth_to_color/camera_info");
	nh.param<std::string>("pt_srv_reg_pt_cld_topic", registered_pt_cld_topic_, "/locobot/camera/depth_registered/points");

  // Publisher declarations
	image_color_filt_pub_ = nh.advertise<sensor_msgs::Image>("/locobot/camera/block_color_filt_img",1);
	camera_cube_locator_marker_ = nh.advertise<visualization_msgs::Marker>("/locobot/camera_cube_locator",1);
	
	// Subscriber declarations
	cam_info_sub_ = nh.subscribe(depth_img_camera_info_,1,&Matching_Pix_to_Ptcld::info_callback,this);
	depth_sub_ = nh.subscribe(depth_image_topic_,1,&Matching_Pix_to_Ptcld::depth_callback,this);
	rgb_image_sub_ = nh.subscribe(color_image_topic_,1,&Matching_Pix_to_Ptcld::color_image_callback,this);
	depth_cam_info_ready_ = false; //set this to false so that depth doesn't ask for camera_model_ until its been set
	point_3d_ready_ = false;
	p2d = true;
	p3d = false;
	//Service
	service_ = nh.advertiseService("pix_to_point_cpp", &Matching_Pix_to_Ptcld::service_callback, this);
}

void Matching_Pix_to_Ptcld::camera_cube_locator_marker_gen(){
	if (point_3d_ready_) {
		ROS_DEBUG("size %d", point_3d_cloud_.size());
		visualization_msgs::Marker marker;
		if (point_3d_cloud_.size() > 0){
			//marker.header.frame_id = point_3d_cloud_.header.frame_id;
			marker.header.frame_id = header_.frame_id;
			marker.header.stamp = ros::Time::now();
			marker.type = visualization_msgs::Marker::POINTS;
			marker.action = visualization_msgs::Marker::ADD;
			// Set the marker scale
			marker.scale.x = 0.1;  //radius of the sphere
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			for (int ii = 0; ii < point_3d_cloud_.size(); ++ii){
				marker.points.push_back(point_3d_cloud_[ii]);
				marker.colors.push_back(marker_color[ii]);
				ROS_DEBUG("xyz %f %f %f", point_3d_cloud_[ii], point_3d_cloud_[ii], point_3d_cloud_[ii]);
				ROS_DEBUG("color %f %f %f %f", marker_color[ii].r, marker_color[ii].g, marker_color[ii].b,marker_color[ii].a);
			}
			camera_cube_locator_marker_.publish(marker);
		}
		
		
	}
	point_3d_cloud_.clear();
	point_color.clear();
	marker_color.clear();
	uv_pix_.clear();
	
	//mtx_2d.lock();
	p2d = true;
	ROS_DEBUG("changing 2d %d", p2d);
	//mtx_2d.unlock();
	point_3d_ready_ = false;
}

void Matching_Pix_to_Ptcld::info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
	//create a camera model from the camera info
	camera_model_.fromCameraInfo(msg);
	depth_cam_info_ready_ = true;	
}

void Matching_Pix_to_Ptcld::depth_callback(const sensor_msgs::Image::ConstPtr& msg){
	//Take the depth message, using teh 32FC1 encoding and define the depth pointer
	 cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (p3d == false) return;
  //mtx_3d.lock();
  p3d = false;
  ROS_DEBUG("changing p3d %d", p3d);
  //mtx_3d.unlock();
  //Access the pixel of interest
  header_.frame_id = msg->header.frame_id;
  for (int jj = 0; jj < uv_pix_.size(); jj++) {
  	geometry_msgs::Point p_aux = uv_pix_[jj];
	  cv::Mat depth_image = cv_ptr->image;
	  //float depth_value = depth_image.at<float>(uv_pix_.x,uv_pix_.y);  // access the depth value of the desired pixel
	  float depth_value = depth_image.at<float>(p_aux.x,p_aux.y);
	  //If the pixel that was chosen has non-zero depth, then find the point projected along the ray at that depth value
		if (depth_value == 0)
		{
			ROS_WARN("Skipping cause pixel had no depth");
			return;
		}else{
			if (depth_cam_info_ready_)
			{
				
				//Pixel has depth, now we need to find the corresponding point in the pointcloud
				//Use the camera model to get the 3D ray for the current pixel
				//cv::Point2d pixel(uv_pix_.y, uv_pix_.x);
				cv::Point2d pixel(p_aux.y, p_aux.x);
				cv::Point3d ray = camera_model_.projectPixelTo3dRay(pixel);
				//Calculate the 3D point on the ray using the depth value
				cv::Point3d point_3d = ray*depth_value;		
				//geometry_msgs::PointStamped point_3d_geom_msg;
				geometry_msgs::Point point_3d_geom_msg; 
				//point_3d_geom_msg.header = msg->header;
				point_3d_geom_msg.x = point_3d.x;
				point_3d_geom_msg.y = point_3d.y;
				point_3d_geom_msg.z = point_3d.z;
				
				ROS_DEBUG("%f %f %f %f %f",p_aux.x, p_aux.y, point_3d.x, point_3d.y, point_3d.z);
				
				if (point_3d.x != point_3d.x) continue;
				
				ROS_DEBUG("in xy %f %f, aux out %f %f %f", p_aux.x, p_aux.y, point_3d.x, point_3d.y, point_3d.z);
				
				//Transform the point to the pointcloud frame using tf
				std::string point_cloud_frame = camera_model_.tfFrame();
				// Get the camera pose in the desired reference frame
				geometry_msgs::TransformStamped transform;
				try {
				    transform = tf_buffer_.lookupTransform(point_cloud_frame, msg->header.frame_id, ros::Time(0));
				} catch (tf2::TransformException &ex) {
				    ROS_ERROR("%s", ex.what());
				}
				// Transform a point cloud point
				//geometry_msgs::PointStamped point_3d_aux;
				geometry_msgs::Point point_3d_aux;
				//tf2::doTransform(point_3d_geom_msg, point_3d_cloud_, transform); // syntax: (points_in, points_out, transform)
				tf2::doTransform(point_3d_geom_msg, point_3d_aux, transform); // syntax: (points_in, points_out, transform)
				ROS_DEBUG("in xy %f %f, out %f %f %f", p_aux.x, p_aux.y, point_3d_aux.x, point_3d_aux.y, point_3d_aux.z);
				if (point_3d_aux.x != point_3d_aux.x) continue;
				point_3d_cloud_.push_back(point_3d_aux);
				marker_color.push_back(point_color[jj]);
			} else{
				p3d = true;
			}
		}
	}
	ROS_DEBUG("calling marker");
	point_3d_ready_ = true;
	//Now show the cube location spherical marker:
	Matching_Pix_to_Ptcld::camera_cube_locator_marker_gen();
}

void Matching_Pix_to_Ptcld::color_image_callback(const sensor_msgs::Image::ConstPtr& msg){
	//convert sensor_msgs image to opencv image : http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
	cv_bridge::CvImagePtr color_img_ptr;
	try
	{
	  color_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //accesses image through color_img_ptr->image
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	
	ROS_DEBUG("2d %d", p2d);
	if (p2d == false) return;
	//mtx_2d.lock();
	p2d = false;
	ROS_DEBUG("changing p2d %d", p2d);
	//mtx_2d.unlock();
	
	//Convert the color image to HSV
	cv::Mat hsv_img;
    cv::cvtColor(color_img_ptr->image, hsv_img, cv::COLOR_RGB2HSV);

    // Gray scale image
    cv::Mat gray;
    cv::cvtColor(hsv_img, gray, cv::COLOR_BGR2GRAY);

    // Threshold image
    cv::Mat mask;
    
    cv::adaptiveThreshold(gray, mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 3, 1);
    

    // Find countors
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    ROS_DEBUG("num countors %lu", contours.size());

    // Filter out countours that are too small
    std::vector<std::vector<cv::Point>> contours_filtered;
    for (int i = 0; i < contours.size(); i++) {
    	ROS_DEBUG("area: %f",cv::contourArea(contours[i]));
    	int area = cv::contourArea(contours[i]);
	if (area > 5 && area < 1000) {
	    contours_filtered.push_back(contours[i]);
	}
    }
    
    ROS_DEBUG("countors added");

    // Get countour mean color
    std::vector<cv::Mat> contour_masks;
    std::vector<cv::Vec3b> contours_mean_color;
    for (int i = 0; i < contours_filtered.size(); i++) {
		cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
		cv::drawContours(mask, contours_filtered, i, cv::Scalar(255), -1);
		contour_masks.push_back(mask);
		cv::Scalar mean = cv::mean(hsv_img, mask);
		contours_mean_color.push_back(cv::Vec3b(mean[0], mean[1], mean[2]));
		
		for (int j = 0; j < 3; j++) {
			ROS_DEBUG("mean %d %f", j, mean[j]);
		}
    }

    // Get distance to desired color. We give more weight to hue, than saturation and value
	for (int i = 1; i < contours_mean_color.size(); i++) {
		std::vector<float> distance;
		cv::Vec3b color = contours_mean_color[i];
		
		// 0 Red
		distance.push_back(cv::norm(cv::Vec3f(color[0], color[1]*0.4, color[2]*0.4) - cv::Vec3f(0, 200*0.4, 120*0.4)));
		// 1 Green
		distance.push_back(cv::norm(cv::Vec3f(color[0], color[1]*0.4, color[2]*0.4) - cv::Vec3f(50, 200*0.4, 120*0.4)));
		// 2 Blue
		distance.push_back(cv::norm(cv::Vec3f(color[0], color[1]*0.4, color[2]*0.4) - cv::Vec3f(110, 200*0.4, 120*0.4)));
		// 3 Yellow
		distance.push_back(cv::norm(cv::Vec3f(color[0], color[1]*0.4, color[2]*0.4) - cv::Vec3f(25, 200*0.4, 150*0.4)));
		for (int ii = 0; ii < distance.size(); ii++){
			ROS_DEBUG("dist %d %f", ii, distance[ii]);
		}
		
		cv::Scalar mean, stddev;
		cv::meanStdDev(distance, mean, stddev);
		ROS_DEBUG("mean std %f %f", mean[0], stddev[0]);
		if (stddev[0] < 20) continue;

		// Countour color
		for (int jj = 0; jj < distance.size(); jj++){
			ROS_DEBUG("d %d %f", jj, distance[jj]);
		}
		int idx = std::min_element(distance.begin(), distance.end()) - distance.begin();
		std_msgs::ColorRGBA pc;
		pc.a = 1.0;
		ROS_DEBUG("idx %d", idx);
		if (idx == 0){
			pc.r = 1.0; pc.g = 0.0; pc.b = 0.0;
		} else if (idx == 1){
			pc.r = 0.0; pc.g = 1.0; pc.b = 0.0;
		} else if (idx == 2){
			pc.r = 0.0; pc.g = 0.0; pc.b = 1.0;
		} else if (idx == 3){
			pc.r = 1.0; pc.g = 1.0; pc.b = 0.0;
		}
		
		
		point_color.push_back(pc);
		ROS_DEBUG(" color %f %f %f %f", pc.r, pc.g, pc.b, pc.a);

		// Contour center
		cv::Moments M = cv::moments(contours_filtered[i], true);
		cv::Point center(M.m10/M.m00, M.m01/M.m00);
		center.x = (int) center.x;
		center.y = (int) center.y;
  
		cv::Mat mask_img;
		bitwise_and(color_img_ptr->image, color_img_ptr->image, mask_img, contour_masks[i]);
		
		// Turn the average pixel location white; Make the center point pixel bright so it shows up in this image
		mask_img.at<cv::Vec3b>(center.y, center.x) = cv::Vec3b(255, 255, 255);

		geometry_msgs::Point p_aux;
		p_aux.x = center.y;
		p_aux.y = center.x;
		uv_pix_.push_back(p_aux);

		//Publish the image (color img with mask applied)
		/*cv_bridge::CvImage cv_bridge_mask_image;
		cv_bridge_mask_image.header.stamp = ros::Time::now();
		cv_bridge_mask_image.header.frame_id = msg->header.frame_id;
		cv_bridge_mask_image.encoding = sensor_msgs::image_encodings::RGB8;//::MONO8;
		cv_bridge_mask_image.image = mask_img;
		sensor_msgs::Image ros_mask_image; //now convert from cv::Mat image back to ROS sensor_msgs image
		cv_bridge_mask_image.toImageMsg(ros_mask_image);
		image_color_filt_pub_.publish(ros_mask_image);*/
		//break;
	}
	//mtx_3d.lock();
	p3d = true;
	ROS_DEBUG("changed 3d %d", p3d);
	//mtx_3d.unlock();
	
}


bool Matching_Pix_to_Ptcld::service_callback(me326_locobot_example::PixtoPoint::Request &req, me326_locobot_example::PixtoPoint::Response &res){
	// the topic for the rgb_img should be set as a rosparam when the file is launched (this can be done in the launch file, it is not done here since the subscriber is started with the class object instantiation)
	//res.ptCld_point = point_3d_cloud_; //send the point back as a response	
	return true;
}


int main(int argc, char **argv)
{
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc,argv,"matching_ptcld_serv");
  ros::NodeHandle nh("~");
  Matching_Pix_to_Ptcld ctd_obj;
  ros::spin();
  return 0;
}
