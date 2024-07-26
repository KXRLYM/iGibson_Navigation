#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/CollisionMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/conversions.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

namespace octomap_server {

class TrackingOctomapServer: public OctomapServer {
public:
  TrackingOctomapServer(const std::string& filename = "");
  virtual ~TrackingOctomapServer();

  void trackCallback(sensor_msgs::PointCloud2Ptr cloud);
  void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

protected:
  void trackChanges();

  bool listen_changes;
  bool track_changes;
  int min_change_pub;
  std::string change_id_frame;
  ros::Publisher pubFreeChangeSet;
  ros::Publisher pubChangeSet;
  ros::Subscriber subChangeSet;
  ros::Subscriber subFreeChanges;
};

}


class NBVNode {
    public:
        NBVNode()
}