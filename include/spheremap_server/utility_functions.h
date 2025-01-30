#pragma once
#ifndef SP_UTILS_H
#define SP_UTILS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <iostream>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
/* #include <geometry_msgs/PoseStamped.h> */
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <spheremap_server/mapping_structures.h>
#include <spheremap_server/spheremap.h>
#include <spheremap_server/map_types.h>

namespace spheremap_server
{
class StopWatch {
public:
  void tick() {
    measuring          = true;
    measure_start_time = ros::WallTime::now();
  }
  void tock() {
    last_val  = (ros::WallTime::now() - measure_start_time).toNSec() / 1000.0;
    measuring = false;
    sum_val += last_val;
    if (last_val > max_val) {
      max_val = last_val;
    }
  }
  void clear() {
    sum_val   = 0;
    last_val  = 0;
    measuring = false;
    max_val   = 0;
  }
  float valSec() {
    return sum_val / 1000.0;
  }
  float maxValSec() {
    return max_val / 1000.0;
  }

  float         max_val   = 0;
  float         sum_val   = 0;
  float         last_val  = 0;
  bool          measuring = false;
  ros::WallTime measure_start_time;
};

std::vector<Point3DType> getPCA3D(std::vector<Point3DType> points);
tf2::Quaternion               rotationMatrixToQuaternion(std::vector<Point3DType> vectors);
Point3DType    castRayWithNullNodes(std::shared_ptr<MapType> occupancy_octree_, Point3DType point, Point3DType end_point, bool* hit);
bool                isPointSafe(Point3DType center_point, float safe_dist, std::shared_ptr<MapType> occupancy_octree_);
bool                makePointSafe(Point3DType* point_ptr, float safe_dist, int max_iters, std::shared_ptr<MapType> occupancy_octree_,
                                  std::shared_ptr<octomap::SurfaceOcTree> surface_octree_);
tf2::Vector3        getRandomFOVRayEndpoint(Point3DType top_left_corner, bool is_optic_frame = true);
float               getHeadingFromVector(Point3DType normal);
bool                hasNodeUnknownChildren(NodeType node, std::shared_ptr<MapType> octree, uint depth);
float               getExplorednessRelativeToCamera(float dist);  // Because the distance might sometimes be calculated before calling this function
std_msgs::ColorRGBA heatMapColor(double h, double a);
Point3DType    getRandomPointInSphere(float r);
Point3DType    getRandomPointInSphere(float min_r, float delta_r);
BoundingBox         getPointsBoundingBox(std::vector<Point3DType> points);
Point3DType    getSegmentColor(int segment_id);
float               getSegmentPathEulerDistance(octomap::SegmentPath path);
std::vector<pcl::PointXYZ> octomapToPointcloud(std::shared_ptr<MapType> occupancy_octree_, BoundingBox bbx);
std::vector<pcl::PointXYZ> octomapToPointcloud(std::shared_ptr<MapType> occupancy_octree_);
float                      calculatePointFrontierExplorationValue(Point3DType point, int num_raycasts, std::shared_ptr<MapType> occupancy_octree_);

void publishPointsMarkers(std::vector<Point3DType> points, std::vector<int> colors, std::string markers_name, float size,
                          ros::Publisher* pub_octomap_markers_, std::string map_frame_);
void publishMarkers(std::vector<visualization_msgs::Marker> markers, ros::Publisher* publisher, std::string markers_name, std::string map_frame,
                    int start_id = 0, float lifetime = 2);
void publishMarkers(std::shared_ptr<std::vector<visualization_msgs::Marker>> markers, ros::Publisher* publisher, std::string markers_name,
                    std::string map_frame, int start_id = 0, float lifetime = 2);

/* BASIC MARKERS */
visualization_msgs::Marker getMarkerArrow(Point3DType pos, Point3DType dir, Point3DType color = Point3DType(0, 1, 0),
                                          float shaft_width = 0.2, float head_width = 0.4, float head_length = 0.1);
visualization_msgs::Marker getMarkerSphere(Point3DType pos, float d, Point3DType color = Point3DType(0, 1, 0), float alpha = 1);
visualization_msgs::Marker getMarkerCube(Point3DType pos, float d, Point3DType color = Point3DType(0, 1, 0));
visualization_msgs::Marker getMarkerLine(Point3DType pos1, Point3DType pos2, Point3DType color = Point3DType(0, 1, 0),
                                         float linewidth = 0.1);
visualization_msgs::Marker getMarkerText(Point3DType pos1, std::string text, Point3DType color = Point3DType(0, 1, 0), float size = 5);
std::vector<visualization_msgs::Marker> getPointsMarkers(std::vector<pcl::PointXYZ> points, Point3DType color = Point3DType(1, 0, 0),
                                                         float size = 0.2);
std::vector<visualization_msgs::Marker> getSphereMapPathsMarkers(std::vector<SphereMapPath> paths, Point3DType path_color = Point3DType(1, 0, 0),
                                                         float path_line_width = 0.2);

/* SEGMAP MARKERS */
std::vector<visualization_msgs::Marker>                  getSegmapDetailedMarkers(std::shared_ptr<SegMap> segmap_, bool is_received_segmap = false);
std::vector<visualization_msgs::Marker>                  getSegmapCoverageMarkers(std::shared_ptr<SegMap> segmap_);
std::vector<visualization_msgs::Marker>                  getSegmapBlockMarkers(std::shared_ptr<SegMap> segmap_, bool is_received_segmap = false);
std::vector<visualization_msgs::Marker>                  getSegmapReceivedMarkers(std::shared_ptr<SegMap> segmap_);
std::shared_ptr<std::vector<visualization_msgs::Marker>> getSegmentOctreeMarkers(std::shared_ptr<octomap::SegmentOcTree> segment_octree_);
std::shared_ptr<std::vector<visualization_msgs::Marker>> getSegmentOctreeMarkersInside(std::shared_ptr<octomap::SegmentOcTree> segment_octree_);

/* SPHEREMAP MARKERS */
std::vector<visualization_msgs::Marker> getSpheremapDebugMarkers(std::shared_ptr<SphereMap> spheremap_);
std::vector<visualization_msgs::Marker> getSpheremapMarkers(Point3DType center, float box_halfsize, std::shared_ptr<SphereMap> spheremap_);
std::vector<visualization_msgs::Marker> getSpheremapPointMarkers(Point3DType center, float box_halfsize, std::shared_ptr<SphereMap> spheremap_,
                                                                 bool is_visited_positions_map = false, float node_maxval = 0, float val_decrease_dist = 0,
                                                                 float blocking_dist = 0);
std::vector<visualization_msgs::Marker> getSpheremapSegmentMarkers(std::shared_ptr<SphereMap> spheremap_);
std::vector<visualization_msgs::Marker> getSpheremapNavigationMarkers(std::shared_ptr<SphereMap> spheremap_);

/* FACETMAP MARKERS */
std::vector<visualization_msgs::Marker> getFacetMapMarkersFull(Point3DType center, float box_halflength,
                                                               std::shared_ptr<octomap::SurfaceOcTree> surface_octree_);
std::vector<visualization_msgs::Marker> getFacetMapMarkersPoints(Point3DType center, float box_halflength,
                                                                 std::shared_ptr<octomap::SurfaceOcTree> surface_octree_);

std::optional<FrontierExplorationPoint> getFrontierExplorationData(Point3DType pos, int num_rays, float max_ray_dist,
                                                                   std::shared_ptr<MapType> occupancy_octree_, bool only_look_down = false);
int  getNearestSegmentRaycasting(Point3DType pos, int num_rays, float max_dist, std::shared_ptr<MapType> occupancy_octree_,
                                 std::shared_ptr<octomap::SegmentOcTree> seg_octree_);
int  getNearestSegmentRaycasting2(Point3DType pos, int num_rays, float max_dist, std::shared_ptr<MapType> occupancy_octree_,
                                  std::shared_ptr<octomap::SegmentOcTree> seg_octree_);
bool arePointsMutuallyVisible(Point3DType p1, Point3DType p2, std::shared_ptr<MapType> occupancy_octree_);
void filterPoints(std::vector<Point3DType>* in, std::vector<Point3DType>* out, float filter_dist);
std::vector<Point3DType> getCylinderSamplingPoints(int num_points_circle, float delta_r, float delta_z, int num_circles, int num_layers);

std::vector<Point3DType> blockAnglesToDirections(float alpha, float beta);
std::pair<float, float>       directionsToBlockAngles(std::vector<Point3DType> p);

std::pair<float, float> calculateBestFitAlphaBeta(std::vector<Point3DType>& pts, float startalpha = 0, float startbeta = 0);
std::pair<float, float> calculateBestFitAlphaBeta(std::vector<Point3DType>& pts, std::vector<float>& radii, float startalpha = 0, float startbeta = 0);

void calculateBlockParamsForSegment(octomap::Segment* seg_ptr, std::vector<Point3DType>& deltapoints);
void calculateBlockParamsForSegment(std::map<uint, SphereMapSegment>::iterator seg_ptr, std::vector<Point3DType>& pts, std::vector<float>& radii);

std::vector<visualization_msgs::Marker> getSegmapNavGraphMarkers(std::shared_ptr<SegMap> segmap_);
float                                   getObstacleDist(Point3DType& test_point, std::shared_ptr<PCLMap>& pclmap);

bool                 arePointsMutuallyVisible2(Point3DType p1, Point3DType p2, std::shared_ptr<MapType> occupancy_octree);
bool                 isNear(std::vector<Point3DType> points, Point3DType, float maxdist);
std::optional<float> getNearestPointDist(std::vector<Point3DType> points, Point3DType, float maxdist);
geometry_msgs::Point octopoint2geometry(Point3DType);
}  // namespace spheremap_server

#endif
