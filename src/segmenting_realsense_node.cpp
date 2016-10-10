#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tracking_realsense/pcl_object_segmenter.h>

namespace pcl_object_segment {
  class SegmenterClient {
    public:
      typedef pcl_object_segment::PclObjectSegmenter::Cloud Cloud;
      typedef typename pcl_object_segment::PclObjectSegmenter::CloudPtr CloudPtr;
      typedef typename pcl_object_segment::PclObjectSegmenter::CloudConstPtr CloudConstPtr;
  
      SegmenterClient(double downsampling_grid_size);
  
    private:
      int counter_;
      double downsampling_grid_size_;
      ros::Publisher pass_pub_;
      ros::Publisher proj_pub_;
      ros::Publisher nonplane_pub_;
      ros::Publisher hull_pub_;
      ros::Publisher seg_pub_;
      ros::Publisher ref_pub_;
      ros::Publisher ref_down_pub_;

      sensor_msgs::PointCloud2 pass_msg_;
      sensor_msgs::PointCloud2 proj_msg_;
      sensor_msgs::PointCloud2 nonplane_msg_;
      sensor_msgs::PointCloud2 hull_msg_;
      sensor_msgs::PointCloud2 seg_msg_;
      sensor_msgs::PointCloud2 ref_msg_;
      sensor_msgs::PointCloud2 ref_down_msg_;
  
      pcl_object_segment::PclObjectSegmenter::PclObjectSegmenterPtr itsSegmenter_;
  
      void convertToRosMsg(const CloudConstPtr &pclCloud, sensor_msgs::PointCloud2 &PC2msg);
  
    public:
      void cloud_cb(const sensor_msgs::PointCloud2ConstPtr & msg);
      void run();
  };
  
  SegmenterClient::SegmenterClient(double downsampling_grid_size) {
    counter_ = 0;
    pcl_object_segment::PclObjectSegmenter::PclObjectSegmenterPtr itsSegmenter(new PclObjectSegmenter(downsampling_grid_size));
    itsSegmenter_ = itsSegmenter;
  }
  
  void SegmenterClient::convertToRosMsg(const CloudConstPtr &pclCloud, sensor_msgs::PointCloud2 &PC2msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*pclCloud,pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, PC2msg);
    PC2msg.header.frame_id = "camera_depth_optical_frame";
  }
  
  void SegmenterClient::cloud_cb(const sensor_msgs::PointCloud2ConstPtr & msg) {
    counter_++;
  
    if ((counter_ == 10) /*|| ((counter_ > 10) && !(itsSegmenter_->modelAvailable()))*/) {
      CloudPtr tempPclCloud;
      sensor_msgs::PointCloud2 msg0 = *msg;
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(msg0, pcl_pc2);
      CloudPtr cloud(new Cloud);
      pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
  
      itsSegmenter_->segmentObject(cloud);
  
      tempPclCloud = itsSegmenter_->getCloudPass();
      convertToRosMsg(tempPclCloud, pass_msg_);
  
      tempPclCloud = itsSegmenter_->getCloudProjected();
      convertToRosMsg(tempPclCloud, proj_msg_);
  
      tempPclCloud = itsSegmenter_->getNonPlaneCloud();
      convertToRosMsg(tempPclCloud, nonplane_msg_);
  
      tempPclCloud = itsSegmenter_->getCloudHull();
      convertToRosMsg(tempPclCloud, hull_msg_);
  
      tempPclCloud = itsSegmenter_->getSegmentedCloud();
      convertToRosMsg(tempPclCloud, seg_msg_);
  
      tempPclCloud = itsSegmenter_->getReferenceCloud();
      convertToRosMsg(tempPclCloud, ref_msg_);
  
      tempPclCloud = itsSegmenter_->getReferenceDownSampledCloud();
      convertToRosMsg(tempPclCloud, ref_down_msg_);
    }
  
    if (counter_ >= 10) {
      pass_pub_.publish(pass_msg_);
      proj_pub_.publish(proj_msg_);
      nonplane_pub_.publish(nonplane_msg_);
      hull_pub_.publish(hull_msg_);
      seg_pub_.publish(seg_msg_);
      ref_pub_.publish(ref_msg_);
      ref_down_pub_.publish(ref_down_msg_);
    }
  }

  void SegmenterClient::run() {
    ros::NodeHandle n;

    pass_pub_ = n.advertise<sensor_msgs::PointCloud2>("/segmenter/pass", 1000);
    proj_pub_ = n.advertise<sensor_msgs::PointCloud2>("/segmenter/proj", 1000);
    nonplane_pub_ = n.advertise<sensor_msgs::PointCloud2>("/segmenter/nonplane", 1000);
    hull_pub_ = n.advertise<sensor_msgs::PointCloud2>("/segmenter/hull", 1000);
    seg_pub_ = n.advertise<sensor_msgs::PointCloud2>("/segmenter/seg", 1000);
    ref_pub_ = n.advertise<sensor_msgs::PointCloud2>("/segmenter/ref", 1000);
    ref_down_pub_ = n.advertise<sensor_msgs::PointCloud2>("/segmenter/ref_down", 1000);

    ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, &SegmenterClient::cloud_cb, this);
    
    ros::spin();
  }
}

int main (int argc, char** argv) {
  double downsampling_grid_size = 0.01;
  pcl_object_segment::SegmenterClient segmenter_client(downsampling_grid_size);

  ros::init(argc, argv, "realsense_segmenter");
  segmenter_client.run();
}
