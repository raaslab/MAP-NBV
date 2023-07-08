/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/thread.hpp>
#include <depth_image_proc/depth_traits.h>
#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace depth_image_proc {

	using namespace message_filters::sync_policies;
    namespace enc = sensor_msgs::image_encodings;

    class PointCloudXyzRadialNodelet : public nodelet::Nodelet
    {

	    ros::NodeHandlePtr rgb_nh_;

	    // Subscriptions

		image_transport::SubscriberFilter sub_depth_;
		message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;

		typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
		typedef ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ExactSyncPolicy;
		typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
		typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
		boost::shared_ptr<Synchronizer> sync_;
		boost::shared_ptr<ExactSynchronizer> exact_sync_;

		// Subscriptions
		boost::shared_ptr<image_transport::ImageTransport> it_;

		// Publications
		boost::mutex connect_mutex_;
		typedef sensor_msgs::PointCloud2 PointCloud;
		ros::Publisher pub_point_cloud_;

		std::string depth_camera_topic;
		std::string camera_info_topic;
		std::string pointcloud_topic;

		std::vector<double> D_;
		boost::array<double, 9> K_;
	  
		int width_;
		int height_;

		cv::Mat binned;
	  
		virtual void onInit();

		void connectCb();

		void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
			     const sensor_msgs::CameraInfoConstPtr& info_msg);

		// Handles float or uint16 depths
		template<typename T>
		void convert(const sensor_msgs::ImageConstPtr& depth_msg, PointCloud::Ptr& cloud_msg);
    };

    cv::Mat initMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs, int width, int height, bool radial)
    {
		int i,j;
		int totalsize = width*height;
		cv::Mat pixelVectors(1,totalsize,CV_32FC3);
		cv::Mat dst(1,totalsize,CV_32FC3);

		cv::Mat sensorPoints(cv::Size(height,width), CV_32FC2);
		cv::Mat undistortedSensorPoints(1,totalsize, CV_32FC2);

		std::vector<cv::Mat> ch;
		for(j = 0; j < height; j++)
		{
		    for(i = 0; i < width; i++)
		    {
			cv::Vec2f &p = sensorPoints.at<cv::Vec2f>(i,j);
			p[0] = i;
			p[1] = j;
		    }
		}

		sensorPoints = sensorPoints.reshape(2,1);

		cv::undistortPoints(sensorPoints, undistortedSensorPoints, cameraMatrix, distCoeffs);

		ch.push_back(undistortedSensorPoints);
		ch.push_back(cv::Mat::ones(1,totalsize,CV_32FC1));
		cv::merge(ch,pixelVectors);

		if(radial)
		{
		    for(i = 0; i < totalsize; i++)
		    {
			normalize(pixelVectors.at<cv::Vec3f>(i),
				  dst.at<cv::Vec3f>(i));
		    }
		    pixelVectors = dst;
		}
		return pixelVectors.reshape(3,width);
    }
  

    void PointCloudXyzRadialNodelet::onInit()
    {
		ros::NodeHandle& nh         = getNodeHandle();
		ros::NodeHandle& private_nh = getPrivateNodeHandle();
		rgb_nh_.reset( new ros::NodeHandle(nh, "rgb") );
  		ros::NodeHandle depth_nh(nh, "depth_registered");
		it_.reset(new image_transport::ImageTransport(depth_nh));

		// Read parameters
		int queue_size;
		private_nh.param("queue_size", queue_size, 5);
		bool use_exact_sync;
  		private_nh.param("exact_sync", use_exact_sync, false);
  		
  		private_nh.param<std::string>("robot/depth", depth_camera_topic, "/Image");
  		private_nh.param<std::string>("robot/info", camera_info_topic, "/Info");
  		private_nh.param<std::string>("robot/pointcloud", pointcloud_topic, "/Points");

  		// Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
		if (use_exact_sync)
		{
			exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_depth_, sub_info_) );
			exact_sync_->registerCallback(boost::bind(&PointCloudXyzRadialNodelet::depthCb, this, _1, _2));
		}
		else
		{
			sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_info_) );
			ROS_INFO("Registering callback");
			sync_->registerCallback(boost::bind(&PointCloudXyzRadialNodelet::depthCb, this, _1, _2));
		}

		// Monitor whether anyone is subscribed to the output
		ros::SubscriberStatusCallback connect_cb = 
		    boost::bind(&PointCloudXyzRadialNodelet::connectCb, this);
		// Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
		boost::lock_guard<boost::mutex> lock(connect_mutex_);
		pub_point_cloud_ = nh.advertise<PointCloud>(pointcloud_topic, 1, connect_cb, connect_cb);
    }

    // Handles (un)subscribing when clients (un)subscribe
    void PointCloudXyzRadialNodelet::connectCb()
    {
		boost::lock_guard<boost::mutex> lock(connect_mutex_);
		if (pub_point_cloud_.getNumSubscribers() == 0)
		{
		    sub_depth_.unsubscribe();
    		sub_info_ .unsubscribe();
		}
		else if (!sub_depth_.getSubscriber())
		{
			ros::NodeHandle& private_nh = getPrivateNodeHandle();
		    // parameter for depth_image_transport hint
		    std::string depth_image_transport_param = "depth_image_transport";

		    // depth image can use different transport.(e.g. compressedDepth)
		    image_transport::TransportHints depth_hints("raw",ros::TransportHints(), private_nh, depth_image_transport_param);
		    sub_depth_.subscribe(*it_, depth_camera_topic,       1, depth_hints);
		    ROS_INFO("Subscribing to depth image topic: %s", depth_camera_topic.c_str());

		    // rgb uses normal ros transport hints.
		    image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
		    sub_info_ .subscribe(*rgb_nh_,   camera_info_topic,      1);
		    ROS_INFO("Subscribing to camera info topic: %s", camera_info_topic.c_str());
		    //image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
		    //sub_depth_ = it_->subscribeCamera("image_raw",
			//			      queue_size,
			//			      &PointCloudXyzRadialNodelet::depthCb,
			//			      this, hints);
		}
    }

    void PointCloudXyzRadialNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
					     const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
		PointCloud::Ptr cloud_msg(new PointCloud);
		cloud_msg->header = depth_msg->header;
		cloud_msg->height = depth_msg->height;
		cloud_msg->width  = depth_msg->width;
		cloud_msg->is_dense = false;
		cloud_msg->is_bigendian = false;

		sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
		pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

		if(info_msg->D != D_ || info_msg->K != K_ || width_ != info_msg->width ||
		   height_ != info_msg->height)
		{
		    D_ = info_msg->D;
		    K_ = info_msg->K;
		    width_ = info_msg->width;
		    height_ = info_msg->height;
		    binned = initMatrix(cv::Mat_<double>(3, 3, &K_[0]),cv::Mat(D_),width_,height_,true);
		}

		if (depth_msg->encoding == enc::TYPE_16UC1)
		{
		    convert<uint16_t>(depth_msg, cloud_msg);
		}
		else if (depth_msg->encoding == enc::TYPE_32FC1)
		{
		    convert<float>(depth_msg, cloud_msg);
		}
		else
		{
		    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
		    return;
		}

		pub_point_cloud_.publish (cloud_msg);
    }

    template<typename T>
    void PointCloudXyzRadialNodelet::convert(const sensor_msgs::ImageConstPtr& depth_msg, PointCloud::Ptr& cloud_msg)
    {
		// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
		double unit_scaling = DepthTraits<T>::toMeters( T(1) );
		float bad_point = std::numeric_limits<float>::quiet_NaN();

		sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
		const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
		int row_step = depth_msg->step / sizeof(T);
		for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
		{
		    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
		    {
			T depth = depth_row[u];

			// Missing points denoted by NaNs
			if (!DepthTraits<T>::valid(depth))
			{
			    *iter_x = *iter_y = *iter_z = bad_point;
			    continue;
			}
			const cv::Vec3f &cvPoint = binned.at<cv::Vec3f>(u,v) * DepthTraits<T>::toMeters(depth);
			// Fill in XYZ
			*iter_x = cvPoint(0);
			*iter_y = cvPoint(1);
			*iter_z = cvPoint(2);
		    }
		}
    }

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::PointCloudXyzRadialNodelet,nodelet::Nodelet);
