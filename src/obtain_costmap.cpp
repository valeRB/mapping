#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include "std_msgs/Bool.h"
#include <boost/foreach.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//std_msgs::Bool enableCostMap;
class createCostMap
{
public:
    ros::NodeHandle n;
    ros::Subscriber map_subscriber;
    ros::Subscriber costMapEnable_subscriber;
    ros::Publisher costMap_publisher;
    ros::Publisher occupGrid_Map_publisher;
    nav_msgs::OccupancyGrid::ConstPtr map_msg, costmap_msg;
    nav_msgs::OccupancyGrid occupGrid_Map, costMap;
    std::vector<signed char> costMap_vector;
    int width_robot, height_robot;
    int max, min;
    int flag;
    createCostMap()
    {
        n = ros::NodeHandle();
    }
    ~createCostMap()
    {}
    void init()
    {
     //   costMapEnable_subscriber = n.subscribe("/enableCostMap",1, &createCostMap::enableCostCallback, this);
        costMap_publisher = n.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
        occupGrid_Map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/gridmap", 1);

        getMap();
        //occupGrid_Map = map_msg;
        costMap_init();
        CostMapCreation();
        flag = 0;
    }

    void getMap()
    {
        rosbag::Bag bag;
        bag.open("/home/ras/catkin_ws/src/mapping/bagfiles/map_test_2top.bag", rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery("/gridmap"));
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            map_msg = m.instantiate<nav_msgs::OccupancyGrid>();
           // occupGrid_Map = map_msg;
            occupGrid_Map.data = map_msg->data;
            occupGrid_Map.header.stamp = ros::Time(0);
            occupGrid_Map.header.frame_id = "map";
            occupGrid_Map.info.resolution = 0.02;
            occupGrid_Map.info.height = 500;
            occupGrid_Map.info.width = 500;
            occupGrid_Map.info.origin.position.x = 0;
            occupGrid_Map.info.origin.position.y = 0;
            occupGrid_Map.info.origin.position.z = 0;
        }
        bag.close();
    }

 //   void enableCostCallback(const std_msgs::Bool &enable_msg)
 //   {
  //      enableCostMap = enable_msg;
  //  }

    void costMap_init()
    {
        //costMap.header.stamp = ros::Time(0);
        costMap.header.frame_id = "map";
        costMap.info.resolution = occupGrid_Map.info.resolution;
        costMap.info.height = occupGrid_Map.info.height;
        costMap.info.width = occupGrid_Map.info.width;
        costMap.info.origin.position.x = occupGrid_Map.info.origin.position.x;
        costMap.info.origin.position.y = occupGrid_Map.info.origin.position.y;
        costMap.info.origin.position.z = occupGrid_Map.info.origin.position.z;
        int cellNumber = costMap.info.height * costMap.info.width;
        costMap_vector = std::vector<signed char>(cellNumber,-1);
        width_robot = 10;
        height_robot = 10;
    }

    void CostMapCreation()
    {
        int width_map = costMap.info.width;
//        ROS_INFO("width_map: %d", width_map);
        for(int m = 0; m < width_map; m++)
        {
            for(int n = 0; n < width_map; n++)
            {
                if(occupGrid_Map.data[m+width_map*n] == 0 || occupGrid_Map.data[m+width_map*n] == 110)
                    if(costMap_vector[m+width_map*n]!=150)
                    {
                        costMap_vector[m+width_map*n] = 0;
                    }
            }

        }
        for(int m = 0; m < width_map; m++)
        {
            for(int n = 0; n < width_map; n++)
            {

                if(occupGrid_Map.data[m+width_map*n] == -106)
                {
                    costMapUpdate(m,n);
                }
            }
            costMap.data = costMap_vector;
        }
    }

       // costMap_publisher.publish(costMap);

    //    enableCostMap.data = false;

    void publishoccupGrid_Map()
    {
        occupGrid_Map_publisher.publish(occupGrid_Map);
    }
    void publishcostmap()
    {
        costMap_publisher.publish(costMap);
    }
    void costMapUpdate(int x_1, int y_1)
    {
        for(int i = x_1-(width_robot/2); i <= (x_1+(width_robot/2)); i++)
        {
            for(int j = y_1-(height_robot/2); j <= (y_1+(height_robot/2)); j++)
            {
                costMap_vector[i+costMap.info.width*j] = 150;
            }
        }
    }
private:

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obtain_costmap");
    createCostMap map;
    map.init();

    ros::Rate loop_rate(20.0);
    while(map.n.ok())
    {
        ros::spinOnce();
        map.publishcostmap();
        map.publishoccupGrid_Map();
        loop_rate.sleep();
    }
    return 0;
}
