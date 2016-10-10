/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <boost/format.hpp>

#include <tracking_realsense/pcl_object_segmenter.h>

namespace pcl_object_segment {

  PclObjectSegmenter::PclObjectSegmenter (double downsampling_grid_size) {
    object_model_available_ = false;
    downsampling_grid_size_ = downsampling_grid_size;
  }
  
  void PclObjectSegmenter::filterPassThrough(CloudPtr &cloud, Cloud &result) {
    //TODO: play with these parameters
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z"); //TODO: this might be causing points to be filtered unnecessarily
    pass.setFilterLimits (0.0, 10.0);
    //pass.setFilterLimits (0.0, 1.5);
    //pass.setFilterLimits (0.0, 0.6);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
  }
  
  void PclObjectSegmenter::euclideanSegment (const CloudConstPtr &cloud, std::vector<pcl::PointIndices> &cluster_indices) {
    pcl::EuclideanClusterExtraction<PointType> ec;
    KdTreePtr tree (new KdTree ());
    
    // TODO: play with these parameters
    ec.setClusterTolerance (0.003); // in meters
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (100000);
    //ec.setMaxClusterSize (400);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
  }
    
  void PclObjectSegmenter::gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size) {
    double start = pcl::getTime ();
    pcl::VoxelGrid<PointType> grid;
    //pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
    //result = *cloud;
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
  }
  
  void PclObjectSegmenter::planeSegmentation (const CloudConstPtr &cloud, pcl::ModelCoefficients &coefficients, pcl::PointIndices &inliers) {
    // TODO: play with these parameters, display segmented plane
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud);
    seg.segment (inliers, coefficients);
  }
  
  void PclObjectSegmenter::planeProjection (const CloudConstPtr &cloud, Cloud &result, const pcl::ModelCoefficients::ConstPtr &coefficients) {
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (result);
  }
  
  void PclObjectSegmenter::convexHull (const CloudConstPtr &cloud, Cloud &, std::vector<pcl::Vertices> &hull_vertices) {
    pcl::ConvexHull<PointType> chull;
    chull.setInputCloud (cloud);
    chull.reconstruct (*cloud_hull_, hull_vertices);
  }
  
  void PclObjectSegmenter::extractNonPlanePoints (const CloudConstPtr &cloud, const CloudConstPtr &cloud_hull, Cloud &result) {
    pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
    pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
    polygon_extract.setHeightLimits (0.01, 10.0);
    polygon_extract.setInputPlanarHull (cloud_hull);
    polygon_extract.setInputCloud (cloud);
    polygon_extract.segment (*inliers_polygon);
    {
      pcl::ExtractIndices<PointType> extract_positive;
      extract_positive.setNegative (false);
      extract_positive.setInputCloud (cloud);
      extract_positive.setIndices (inliers_polygon);
      extract_positive.filter (result);
    }
  }
  
  void PclObjectSegmenter::removeZeroPoints (const CloudConstPtr &cloud, Cloud &result) {
    for (size_t i = 0; i < cloud->points.size (); i++) {
      PointType point = cloud->points[i];
      if (/*!(fabs(point.x) < 0.01 &&
            fabs(point.y) < 0.01 &&
            fabs(point.z) < 0.01) &&*/
          !pcl_isnan(point.x) &&
          !pcl_isnan(point.y) &&
          !pcl_isnan(point.z))
        result.points.push_back(point);
    }
  
    result.width = static_cast<pcl::uint32_t> (result.points.size ());
    result.height = 1;
    result.is_dense = true;
  }
  
  void PclObjectSegmenter::extractSegmentCluster (const CloudConstPtr &cloud, const std::vector<pcl::PointIndices> cluster_indices, const int segment_index, Cloud &result) {
    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
    // add all points corresponding to a specific cluster in cloud to the resultant cloud
    for (size_t i = 0; i < segmented_indices.indices.size(); i++) {
      PointType point = cloud->points[segmented_indices.indices[i]];
      result.points.push_back(point);
    }
    result.width = pcl::uint32_t (result.points.size ());
    result.height = 1;
    result.is_dense = true;
  }
  
  void PclObjectSegmenter::segmentObject(CloudPtr &cloud) {
    //boost::mutex::scoped_lock lock(mtx_);
    double start = pcl::getTime();

    cloud_pass_.reset(new Cloud); 		// reset pass-through filtered cloud with a new cloud object
    filterPassThrough(cloud, *cloud_pass_);    // create a pass-through filtered version of the original cloud
    //cloud_pass_ = cloud;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    planeSegmentation(cloud_pass_, *coefficients, *inliers); // any part of the point cloud that conforms to the model of a plane is identified from the filtered point cloud
    CloudPtr target_cloud;
    if (inliers->indices.size () > 3) {
      cloud_projected_.reset(new Cloud);					// reset projected cloud with a new cloud object
      planeProjection(cloud_pass_, *cloud_projected_, coefficients);		// the filtered point cloud is projected onto the segmented plane according to the plane model coefficients
      PCL_INFO("pass through filter done\n");

      cloud_hull_.reset(new Cloud);	    					// reset convex hull cloud with a new cloud object
      convexHull(cloud_projected_, *cloud_hull_, hull_vertices_);		// a convex hull is found for the cloud projected onto the plane
      PCL_INFO("convex hull done\n");

      nonplane_cloud_.reset(new Cloud);    					// reset cloud with nonplane points with a new cloud object
      extractNonPlanePoints(cloud_pass_, cloud_hull_, *nonplane_cloud_);	// get nonplane points from the filtered point cloud using the convex hull
      PCL_INFO("nonplane points done\n");

      target_cloud = nonplane_cloud_;
    }
    else {
      PCL_WARN("cannot segment plane\n");
    }
   
    if (target_cloud != NULL) {
      PCL_INFO("segmentation, please wait...\n");
      std::vector<pcl::PointIndices> cluster_indices;
      euclideanSegment(target_cloud, cluster_indices); // find clusters in the non-plane point cloud
      PCL_INFO("euclidean segmentation done\n");

      if (cluster_indices.size() > 0) {
        PCL_INFO("Number of Clusters: %u \n", cluster_indices.size());
        CloudPtr temp_cloud(new Cloud);
        extractSegmentCluster(target_cloud, cluster_indices, 0, *temp_cloud);
        Eigen::Vector4f c;
        pcl::compute3DCentroid<PointType>(*temp_cloud, c);
        int segment_index = 0;
        unsigned long int segment_num_points = temp_cloud->width;
        PCL_INFO("Cluster 1: %u \n", segment_num_points);

        for (size_t i = 1; i < cluster_indices.size(); i++) {
          temp_cloud.reset(new Cloud);
          extractSegmentCluster(target_cloud, cluster_indices, int (i), *temp_cloud);
          //pcl::compute3DCentroid<PointType>(*temp_cloud, c);
          double distance = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
          unsigned long int num_points = temp_cloud->width;
          PCL_INFO("Cluster %u: %u \n", (i+1), num_points);
          if (distance < 1) { // filter out clusters that are more than a meter away
            if (num_points > segment_num_points) {
              segment_index = int (i);
              segment_num_points = num_points; // take the cluster with the highest number of points
            }
          }
        }
       
        segmented_cloud_.reset(new Cloud);
        extractSegmentCluster(target_cloud, cluster_indices, segment_index, *segmented_cloud_);

        CloudPtr ref_cloud (new Cloud);
        ref_cloud = segmented_cloud_;

        CloudPtr nonzero_ref(new Cloud);
        removeZeroPoints(ref_cloud, *nonzero_ref);
        
        PCL_INFO("calculating cog\n");
        
        CloudPtr transed_ref (new Cloud);
        pcl::compute3DCentroid<PointType>(*nonzero_ref, c);
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
        pcl::transformPointCloud<PointType>(*nonzero_ref, *transed_ref, trans.inverse());
        CloudPtr transed_ref_downsampled(new Cloud);
        gridSample(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
        reference_ = transed_ref; // reference point cloud, not downsampled
        reference_downsampled_ = transed_ref_downsampled;
        object_model_available_ = true;

        PCL_INFO("Done!\n");
      }
      else {
        PCL_WARN("euclidean segmentation failed\n");
      }
    }
    double end = pcl::getTime();
    computation_time_ = end - start;
  }
  
  PclObjectSegmenter::CloudPtr PclObjectSegmenter::getCloudPass() {
    CloudPtr nonzero(new Cloud);
    removeZeroPoints(cloud_pass_,*nonzero);
    return nonzero;
  }
  
  PclObjectSegmenter::CloudPtr PclObjectSegmenter::getCloudProjected() {
    CloudPtr nonzero(new Cloud);
    removeZeroPoints(cloud_projected_, *nonzero);
    return nonzero;
  }
  
  PclObjectSegmenter::CloudPtr PclObjectSegmenter::getNonPlaneCloud() {
    CloudPtr nonzero(new Cloud);
    removeZeroPoints(nonplane_cloud_, *nonzero);
    return nonzero;
  }
  
  PclObjectSegmenter::CloudPtr PclObjectSegmenter::getCloudHull() {
    CloudPtr nonzero(new Cloud);
    removeZeroPoints(cloud_hull_, *nonzero);
    return nonzero;
  }
  
  PclObjectSegmenter::CloudPtr PclObjectSegmenter::getSegmentedCloud() {
    CloudPtr nonzero(new Cloud);
    removeZeroPoints(segmented_cloud_, *nonzero);
    return nonzero;
  }
  
  PclObjectSegmenter::CloudPtr PclObjectSegmenter::getReferenceCloud() {
    CloudPtr nonzero(new Cloud);
    removeZeroPoints(reference_, *nonzero);
    return nonzero;
  }
  
  PclObjectSegmenter::CloudPtr PclObjectSegmenter::getReferenceDownSampledCloud() {
    CloudPtr nonzero(new Cloud);
    removeZeroPoints(reference_downsampled_, *nonzero);
    return nonzero;
  }
  
  bool PclObjectSegmenter::modelAvailable() {
    return object_model_available_;
  }
}
