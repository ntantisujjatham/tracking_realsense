#ifndef PCL_OBJECT_TRACKER
#define PCL_OBJECT_TRACKER

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

namespace pcl_object_track {
  class PclObjectTracker {
    public:
      typedef pcl::PointXYZ PointType;
      typedef pcl::tracking::ParticleXYZRPY ParticleT;
      typedef pcl::PointCloud<PointType> Cloud;
      typedef typename Cloud::Ptr CloudPtr;
      typedef typename Cloud::ConstPtr CloudConstPtr;
      //typedef KLDAdaptiveParticleFilterTracker<PointType, ParticleT> ParticleFilter;
      //typedef KLDAdaptiveParticleFilterOMPTracker<PointType, ParticleT> ParticleFilter;
      //typedef ParticleFilterOMPTracker<PointType, ParticleT> ParticleFilter;
      typedef pcl::tracking::ParticleFilterTracker<PointType, ParticleT> ParticleFilter;
      typedef typename ParticleFilter::CoherencePtr CoherencePtr;

      typedef boost::shared_ptr<PclObjectTracker> PclObjectTrackerPtr;

      PclObjectTracker(int thread_nr, double downsampling_grid_size, bool use_fixed);
      void setReference(CloudPtr &refCloud);
      void track(const CloudPtr &cloud);
      void getObjectLocation(double &x, double &y, double &z);
      void getResultCloud(CloudPtr &result_cloud);

      CloudPtr cloud_pass_downsampled_;
      CloudPtr result_cloud_;

      double x_;
      double y_;
      double z_;

      boost::mutex mtx_;
      boost::shared_ptr<ParticleFilter> tracker_;
      double tracking_time_;
      double computation_time_;
      double downsampling_time_;
      double downsampling_grid_size_;
    private:
      void filterPassThrough(const CloudPtr &cloud, Cloud &result);
      void gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01);
  };
}

#endif // PCL_OBJECT_TRACKER
