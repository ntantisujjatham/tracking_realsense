#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tracking_realsense/pcl_object_segmenter.h>
#include <tracking_realsense/pcl_object_tracker.h>

namespace pcl_object_track {
  class TrackerClient {
    public:
      typedef pcl_object_track::PclObjectTracker::Cloud Cloud;
      typedef typename pcl_object_track::PclObjectTracker::CloudPtr CloudPtr;
      typedef typename pcl_object_track::PclObjectTracker::CloudConstPtr CloudConstPtr;
  
      TrackerClient(int thread_nr, double downsampling_grid_size, bool use_fixed);
  
    private:
      int counter_;
      ros::Publisher ref_cloud_pub_;
      ros::Publisher tracked_cloud_pub_;
      ros::Publisher tracked_pos_pub_;

      sensor_msgs::PointCloud2 ref_cloud_msg_;
      sensor_msgs::PointCloud2 tracked_cloud_msg_;
      geometry_msgs::PointStamped tracked_pos_msg_;
 
      pcl_object_segment::PclObjectSegmenter::PclObjectSegmenterPtr itsSegmenter_; 
      pcl_object_track::PclObjectTracker::PclObjectTrackerPtr itsTracker_;

      void convertToPclPC(const sensor_msgs::PointCloud2ConstPtr &msg, Cloud &cloud); 
      void convertToRosMsg(const CloudConstPtr &pclCloud, sensor_msgs::PointCloud2 &PC2msg);
  
    public:
      void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
      void run();
  };
  
  TrackerClient::TrackerClient(int thread_nr, double downsampling_grid_size, bool use_fixed) {
    counter_ = 0;
    pcl_object_segment::PclObjectSegmenter::PclObjectSegmenterPtr itsSegmenter(new pcl_object_segment::PclObjectSegmenter(downsampling_grid_size));
    pcl_object_track::PclObjectTracker::PclObjectTrackerPtr itsTracker(new pcl_object_track::PclObjectTracker(thread_nr, downsampling_grid_size, use_fixed));
    itsSegmenter_ = itsSegmenter;
    itsTracker_ = itsTracker;
  }

  void TrackerClient::convertToPclPC(const sensor_msgs::PointCloud2ConstPtr &msg, Cloud &cloud) {
    sensor_msgs::PointCloud2 msg0 = *msg;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg0, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);
  }
  
  void TrackerClient::convertToRosMsg(const CloudConstPtr &pclCloud, sensor_msgs::PointCloud2 &PC2msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*pclCloud,pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, PC2msg);
    PC2msg.header.frame_id = "camera_depth_optical_frame";
  }
  
  void TrackerClient::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    CloudPtr rawcloud(new Cloud);
    convertToPclPC(msg, *rawcloud);
  
    if ((counter_ == 10) /*|| ((counter_ > 10) && !(itsTracker_->modelAvailable()))*/) {
      CloudPtr refCloud;

      itsSegmenter_->segmentObject(rawcloud);
      refCloud = itsSegmenter_->getReferenceDownSampledCloud();
      convertToRosMsg(refCloud, ref_cloud_msg_);

      itsTracker_->setReference(refCloud);
    }
  
    if (counter_ >= 10) {
      CloudPtr resultCloudPcl;
      itsTracker_->track(rawcloud);
      itsTracker_->getResultCloud(resultCloudPcl);

      convertToRosMsg(resultCloudPcl, tracked_cloud_msg_);

      double x;
      double y;
      double z;

      itsTracker_->getObjectLocation(x,y,z);
      tracked_pos_msg_.point.x = x;
      tracked_pos_msg_.point.y = y;
      tracked_pos_msg_.point.z = z;
      tracked_pos_msg_.header.frame_id = "camera_depth_optical_frame";

      tracked_cloud_pub_.publish(tracked_cloud_msg_);
      ref_cloud_pub_.publish(ref_cloud_msg_);
      tracked_pos_pub_.publish(tracked_pos_msg_);
    }

    counter_++;
  }

  void TrackerClient::run() {
    ros::NodeHandle n;

    tracked_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("/realsense_tracker/tracked_cloud", 1000);
    ref_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("/realsense_tracker/ref_cloud", 1000);
    tracked_pos_pub_ = n.advertise<geometry_msgs::PointStamped>("/realsense_tracker/tracked_pos", 1000);

    ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, &TrackerClient::cloud_cb, this);
    
    ros::spin();
  }
}

int main (int argc, char** argv) {
  double downsampling_grid_size = 0.01;
  pcl_object_track::TrackerClient tracker_client(8, downsampling_grid_size, false);

  ros::init(argc, argv, "realsense_tracker");
  tracker_client.run();
}
