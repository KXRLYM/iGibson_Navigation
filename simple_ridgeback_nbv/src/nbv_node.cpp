#include"ros/ros.h"
#include"sensor_msgs/JointState.h"

#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <fstream>
#include <vector>

#include <octomap_msgs/GetOctomap.h>

#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

// #include "quadtree.h"


static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;

static constexpr unsigned char OBJECT_DETECTED = 255;
static constexpr unsigned char CANDIDATES = 125; 

static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

class NBVNode
{
    public:
        NBVNode():
            nh{},
            pub_pgrid(nh.advertise<nav_msgs::OccupancyGrid>("test_map", 5)),
            sub_ogrid(nh.subscribe("/projected_map", 1000, &NBVNode::ogrid_callback, this)),
            timer(nh.createTimer(ros::Duration(0.1), &NBVNode::main_loop, this))
            {
            int radius = 1;
            }

            void ogrid_callback(const nav_msgs::OccupancyGrid & ogrid)
            {

            // Firstly read in the metadata
            std_msgs::Header header = ogrid.header;
            nav_msgs::MapMetaData info = ogrid.info;
            nav_msgs::OccupancyGrid* pgrid = new nav_msgs::OccupancyGrid();
            ROS_INFO("Got map %d %d", info.width, info.height);

            pgrid->info = info;
            pgrid->header = header;

            std::vector<int8_t> pgrid_data(info.width*info.height, OCC_GRID_UNKNOWN);
            //char* pgrid_data = new char[info.width*info.height](OCC_GRID_UNKNOWN);

            int8_t data;
            int32_t DOWN, UP, RIGHT, LEFT, TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT;
            for (int i = 0; i < info.width*info.height; i++) {
                data = ogrid.data[i];

                DOWN = i + info.width;
                UP = i - info.width;
                RIGHT = i + 1;
                LEFT = i - 1;
                TOP_RIGHT = i - info.width + 1;
                TOP_LEFT = i - info.width - 1;
                BOTTOM_LEFT = i + info.width - 1;
                BOTTOM_RIGHT = i + info.width + 1;

                if (data == OCC_GRID_OCCUPIED) {
                    if (ogrid.data[DOWN] != OCC_GRID_FREE && ogrid.data[DOWN] != OCC_GRID_OCCUPIED && ogrid.data[DOWN] != CANDIDATES && bound_check(DOWN, info.width, info.height)) {
                        pgrid_data[DOWN] = CANDIDATES;
                    } 
                    
                    if (ogrid.data[UP] != OCC_GRID_FREE && ogrid.data[UP] != OCC_GRID_OCCUPIED && ogrid.data[UP] != CANDIDATES && bound_check(UP, info.width, info.height)) {
                        pgrid_data[UP] = CANDIDATES;
                    }
                    
                    if (ogrid.data[LEFT] != OCC_GRID_FREE && ogrid.data[LEFT] != OCC_GRID_OCCUPIED && ogrid.data[LEFT] != CANDIDATES && bound_check(LEFT, info.width, info.height)) {
                        pgrid_data[LEFT] = CANDIDATES;
                    } 
                    
                    if (ogrid.data[RIGHT] != OCC_GRID_FREE && ogrid.data[RIGHT] != OCC_GRID_OCCUPIED && ogrid.data[RIGHT] != CANDIDATES && bound_check(RIGHT, info.width, info.height)) {
                        pgrid_data[RIGHT] = CANDIDATES;
                    }

                    if (ogrid.data[BOTTOM_LEFT] != OCC_GRID_OCCUPIED && ogrid.data[BOTTOM_LEFT] != OCC_GRID_FREE && ogrid.data[BOTTOM_LEFT] != CANDIDATES && bound_check(BOTTOM_LEFT, info.width, info.height)) {
                        pgrid_data[BOTTOM_LEFT] = CANDIDATES;
                    }

                    if (ogrid.data[BOTTOM_RIGHT] != OCC_GRID_OCCUPIED && ogrid.data[BOTTOM_RIGHT] != OCC_GRID_FREE && ogrid.data[BOTTOM_RIGHT] != CANDIDATES && bound_check(BOTTOM_RIGHT, info.width, info.height)) {
                        pgrid_data[BOTTOM_RIGHT] = CANDIDATES;
                    }

                    if (ogrid.data[TOP_LEFT] != OCC_GRID_OCCUPIED && ogrid.data[TOP_LEFT] != OCC_GRID_FREE && ogrid.data[TOP_LEFT] != CANDIDATES && bound_check(TOP_LEFT, info.width, info.height)) {
                        pgrid_data[TOP_LEFT] = CANDIDATES;
                    }

                    
                    if (ogrid.data[TOP_RIGHT] != OCC_GRID_OCCUPIED && ogrid.data[TOP_RIGHT] != OCC_GRID_FREE && ogrid.data[TOP_RIGHT] != CANDIDATES && bound_check(TOP_RIGHT, info.width, info.height)) {
                        pgrid_data[TOP_RIGHT] = CANDIDATES;
                    }
                }
            }

            /** 
            for (unsigned int x = 0; x < info.width; x++){
                for (unsigned int y = 0; y < info.height; y++) {
                    if (ogrid.data[x+info.width*y] > 1 && ogrid.data[x+info.width*y] ) {
                        unexplored++;
                    }
                    data_set[x][y] << ogrid.data[x+ info.width * y];
                }
            }
            **/
            pgrid->data = pgrid_data;
            pub_pgrid.publish(*pgrid);
            }


            bool bound_check(int32_t location, int32_t width, int32_t height) {
                if (location < 0 || location > width * height - 1) {
                    //ROS_INFO("false %d", location);
                    return false;
                }
                return true;
            }

            void iterate_neighbours() {

            }

            void main_loop(const ros::TimerEvent &) const
            {
            }

         

    private:
        ros::NodeHandle nh;
        ros::Publisher pub_pgrid;
        ros::Subscriber sub_ogrid;
        ros::Timer timer;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nbv_node");
    NBVNode node;
    ros::spin();
    return 0;
}

