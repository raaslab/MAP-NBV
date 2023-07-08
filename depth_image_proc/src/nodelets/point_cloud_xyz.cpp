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
#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace depth_image_proc {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class PointCloudXyzNodelet : public nodelet::Nodelet
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
  //image_transport::CameraSubscriber sub_depth_;

  // Publications
  boost::mutex connect_mutex_;
  typedef sensor_msgs::PointCloud2 PointCloud;
  ros::Publisher pub_point_cloud_;

  image_geometry::PinholeCameraModel model_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void PointCloudXyzNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  rgb_nh_.reset( new ros::NodeHandle(nh, "rgb") );
  ros::NodeHandle depth_nh(nh, "depth_registered");
  it_.reset( new image_transport::ImageTransport(depth_nh) );
  //it_.reset(new image_transport::ImageTransport(nh));

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool use_exact_sync;
  private_nh.param("exact_sync", use_exact_sync, false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync)
  {
    exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_depth_, sub_info_) );
    exact_sync_->registerCallback(boost::bind(&PointCloudXyzNodelet::depthCb, this, _1, _2));
  }
  else
  {
    sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_info_) );
    ROS_INFO("Registering callback");
    sync_->registerCallback(boost::bind(&PointCloudXyzNodelet::depthCb, this, _1, _2));
  }

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_point_cloud_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyzNodelet::connectCb()
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
    sub_depth_.subscribe(*it_, "/airsim_node/drone_1/front_center_custom_depth/DepthPerspective",       1, depth_hints);
    ROS_INFO("Subscribing to depth image.");

    // rgb uses normal ros transport hints.
    image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
    sub_info_ .subscribe(*rgb_nh_,   "/airsim_node/drone_1/front_center_custom_color/Scene/camera_info",      1);
    ROS_INFO("Subscribing to camera info.");
    //image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    //sub_depth_ = it_->subscribeCamera("/airsim_node/drone_1/front_center_custom_depth/DepthPerspective/", queue_size_, &PointCloudXyzNodelet::depthCb, this, hints);
    //ROS_INFO("Subscribing to depth image.");
  }
}

void PointCloudXyzNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  ROS_INFO("Inside callback");
  PointCloud::Ptr cloud_msg(new PointCloud);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Update camera model
  model_.fromCameraInfo(info_msg);

  if (depth_msg->encoding == enc::TYPE_16UC1)
  {
    convert<uint16_t>(depth_msg, cloud_msg, model_);
  }
  else if (depth_msg->encoding == enc::TYPE_32FC1)
  {
    convert<float>(depth_msg, cloud_msg, model_);
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_.publish (cloud_msg);
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::PointCloudXyzNodelet,nodelet::Nodelet);
