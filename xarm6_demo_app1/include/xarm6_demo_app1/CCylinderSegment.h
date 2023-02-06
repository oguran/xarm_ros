#if !defined(XARM6_DEMO_APP1_CCYLINDERSEGMENT_H)
#define XARM6_DEMO_APP1_CCYLINDERSEGMENT_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>


class CCylinderSegment
{
public:
  CCylinderSegment(ros::NodeHandle& node_handle);
  /** \brief Given the parameters of the cylinder add the cylinder to the planning scene.
      @param cylinder_params - Pointer to the struct AddCylinderParams. */
  void addCylinder();

  /** \brief Given the pointcloud containing just the cylinder, compute its center point and its height and store in
     cylinder_params.
      @param cloud - Pointcloud containing just the cylinder.
      @param cylinder_params - Pointer to the struct AddCylinderParams. */
  void extractLocationHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
      @param cloud - Pointcloud.
      @param cloud_normals - The point normals once computer will be stored in this. */
  void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

  /** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers_plane - Indices whose normals need to be extracted. */
  void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane);

  /** \brief Given the pointcloud and indices of the plane, remove the plannar region from the pointcloud.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. */
  void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers_plane);

  /** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point normals extract the cylinder from the
     pointcloud and store the cylinder parameters in coefficients_cylinder.
      @param cloud - Pointcloud whose plane is removed.
      @param coefficients_cylinder - Cylinder parameters used to define an infinite cylinder will be stored here.
      @param cloud_normals - Point normals corresponding to the plane on which cylinder is kept */
  void extractCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder,
                       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);

private:
  // BEGIN_SUB_TUTORIAL param_struct
  // There are 4 fields and a total of 7 parameters used to define this.
  struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector towards the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };
  // Declare a variable of type AddCylinderParams and store relevant values from ModelCoefficients.
  AddCylinderParams* cylinder_params;
  // END_SUB_TUTORIAL

  bool points_not_found = true;
};


#endif // XARM6_DEMO_APP1_CCYLINDERSEGMENT_H
