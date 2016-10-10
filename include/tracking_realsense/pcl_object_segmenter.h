#ifndef PCL_OBJECT_SEGMENTER
#define PCL_OBJECT_SEGMENTER

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

namespace pcl_object_segment {
  class PclObjectSegmenter {
    public:
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> Cloud;
      typedef typename Cloud::Ptr CloudPtr;
      typedef typename Cloud::ConstPtr CloudConstPtr;
    
      typedef typename pcl::search::KdTree<PointType> KdTree;
      typedef typename KdTree::Ptr KdTreePtr;

      typedef boost::shared_ptr<PclObjectSegmenter> PclObjectSegmenterPtr;
    
      PclObjectSegmenter(double downsampling_grid_size);
    
    private:
      CloudPtr cloud_pass_; 		// pass-through filtered cloud
      CloudPtr cloud_projected_;	// cloud projected onto segmented plane
      CloudPtr nonplane_cloud_;		// part of cloud that is not part of the segmented plane
      CloudPtr cloud_hull_;		// convex hull of projected cloud
      CloudPtr segmented_cloud_;	// segmented cloud representing object before removing zeros
      CloudPtr reference_;
      CloudPtr reference_downsampled_;
      std::vector<pcl::Vertices> hull_vertices_;
    
      //boost::mutex mtx_;
      bool object_model_available_;
      double computation_time_;
      double downsampling_time_;
      double downsampling_grid_size_;
    
      void filterPassThrough(CloudPtr &cloud, Cloud &result);
      void euclideanSegment (const CloudConstPtr &cloud, std::vector<pcl::PointIndices> &cluster_indices);
      void gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01);
      void planeSegmentation (const CloudConstPtr &cloud, pcl::ModelCoefficients &coefficients, pcl::PointIndices &inliers);
      void planeProjection (const CloudConstPtr &cloud, Cloud &result, const pcl::ModelCoefficients::ConstPtr &coefficients);
      void convexHull (const CloudConstPtr &cloud, Cloud &, std::vector<pcl::Vertices> &hull_vertices);
      void extractNonPlanePoints (const CloudConstPtr &cloud, const CloudConstPtr &cloud_hull, Cloud &result);
      void removeZeroPoints (const CloudConstPtr &cloud, Cloud &result);
      void extractSegmentCluster (const CloudConstPtr &cloud, const std::vector<pcl::PointIndices> cluster_indices, const int segment_index, Cloud &result);
    
    public:
      void segmentObject(CloudPtr &cloud);
      CloudPtr getCloudPass();
      CloudPtr getCloudProjected();
      CloudPtr getNonPlaneCloud();
      CloudPtr getCloudHull();
      CloudPtr getSegmentedCloud();
      CloudPtr getReferenceCloud();
      CloudPtr getReferenceDownSampledCloud();
      bool modelAvailable();
  };
}

#endif // PCL_OBJECT_SEGMENTER
