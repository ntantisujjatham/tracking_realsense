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
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
//#include <pcl/tracking/hsv_color_coherence.h>
//#include <pcl/tracking/normal_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

//#include <pcl/features/normal_3d.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <tracking_realsense/pcl_object_tracker.h>

namespace pcl_object_track {

  PclObjectTracker::PclObjectTracker(int thread_nr, double downsampling_grid_size, bool use_fixed) {
    downsampling_grid_size_ = downsampling_grid_size;

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
    if (use_fixed) {
      boost::shared_ptr<pcl::tracking::ParticleFilterOMPTracker<PointType, ParticleT> > tracker
        (new pcl::tracking::ParticleFilterOMPTracker<PointType, ParticleT>(thread_nr));
      tracker_ = tracker;
    }
    else {
      boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointType, ParticleT> > tracker(new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointType, ParticleT> (thread_nr));
      tracker->setMaximumParticleNum (500);
      tracker->setDelta (0.99);
      tracker->setEpsilon (0.2);
      ParticleT bin_size;
      bin_size.x = 0.1f;
      bin_size.y = 0.1f;
      bin_size.z = 0.1f;
      bin_size.roll = 0.1f;
      bin_size.pitch = 0.1f;
      bin_size.yaw = 0.1f;
      tracker->setBinSize (bin_size);
      tracker_ = tracker;
    }
    
    tracker_->setTrans(Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance(default_step_covariance);
    tracker_->setInitialNoiseCovariance(initial_noise_covariance);
    tracker_->setInitialNoiseMean(default_initial_mean);
    tracker_->setIterationNum(1);
    
    tracker_->setParticleNum(400);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal(false);
    // setup coherences
    pcl::tracking::ApproxNearestPairPointCloudCoherence<PointType>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<PointType>::Ptr(new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointType>());
    // NearestPairPointCloudCoherence<PointType>::Ptr coherence = NearestPairPointCloudCoherence<PointType>::Ptr
    //   (new NearestPairPointCloudCoherence<PointType> ());
    
    boost::shared_ptr<pcl::tracking::DistanceCoherence<PointType> > distance_coherence = boost::shared_ptr<pcl::tracking::DistanceCoherence<PointType> >(new pcl::tracking::DistanceCoherence<PointType>());
    coherence->addPointCoherence(distance_coherence);
   
    //NOTE: turned off color coherence 
    //boost::shared_ptr<HSVColorCoherence<PointType> > color_coherence
    //  = boost::shared_ptr<HSVColorCoherence<PointType> > (new HSVColorCoherence<PointType> ());
    //color_coherence->setWeight (0.1);
    //coherence->addPointCoherence (color_coherence);
    
    //boost::shared_ptr<pcl::search::KdTree<PointType> > search (new pcl::search::KdTree<PointType> (false));
    boost::shared_ptr<pcl::search::Octree<PointType> > search(new pcl::search::Octree<PointType>(0.01));
    //boost::shared_ptr<pcl::search::OrganizedNeighbor<PointType> > search (new pcl::search::OrganizedNeighbor<PointType>);
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(0.01);
    tracker_->setCloudCoherence(coherence);
  }

  void PclObjectTracker::setReference(CloudPtr &refCloud) {
    tracker_->setReferenceCloud(refCloud);
  }

  void PclObjectTracker::filterPassThrough(const CloudPtr &cloud, Cloud &result) {
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

  void PclObjectTracker::gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size)
  {
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud(cloud);
    grid.filter(result);
  }
    
  void PclObjectTracker::track(const CloudPtr &cloud) {
    boost::mutex::scoped_lock lock (mtx_);
    double start = pcl::getTime ();

    CloudPtr cloud_pass;
    cloud_pass.reset(new Cloud);
    filterPassThrough(cloud, *cloud_pass);

    cloud_pass_downsampled_.reset(new Cloud);
    gridSampleApprox(cloud_pass, *cloud_pass_downsampled_, downsampling_grid_size_);

    tracker_->setInputCloud (cloud_pass_downsampled_);
    tracker_->compute();
  
    ParticleT result = tracker_->getResult();
    Eigen::Affine3f transformation;
    transformation = tracker_->toEigenMatrix(result);
    Eigen::Vector3f transl = transformation.translation();

    x_ = transl[0];
    y_ = transl[1];
    z_ = transl[2];
  
    result_cloud_.reset(new Cloud);
    pcl::transformPointCloud<PointType>(*(tracker_->getReferenceCloud()), *result_cloud_, transformation);
  
    double end = pcl::getTime();
    computation_time_ = end - start;
  }

  void PclObjectTracker::getObjectLocation(double &x, double &y, double &z) {
    x = x_;
    y = y_;
    z = z_;
  }

  void PclObjectTracker::getResultCloud(CloudPtr &result_cloud) {
    result_cloud = result_cloud_;
  }  
}
