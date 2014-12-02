#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include "std_msgs/Bool.h"


std_msgs::Bool enableCostMap;

class createCostMap
{
public:
    ros::NodeHandle n;
    ros::Subscriber map_subscriber;
    ros::Subscriber costMapEnable_subscriber;
    ros::Publisher costMap_publisher;
    nav_msgs::OccupancyGrid occupGrid_Map, costMap;
    std::vector<signed char> costMap_vector;
    int width_robot, height_robot;


    createCostMap()
    {
        n = ros::NodeHandle();
    }

    ~createCostMap()
    {}

    void init()
    {
        // Subscribers:
        costMapEnable_subscriber = n.subscribe("/enableCostMap",1, &createCostMap::enableCostCallback, this);
        map_subscriber = n.subscribe("/gridmap", 1, &createCostMap::mapCallback,this);

        // Publishers:
        costMap_publisher = n.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
    }

    void enableCostCallback(const std_msgs::Bool &enable_msg)
    {
        enableCostMap = enable_msg;
    }

    void mapCallback(const nav_msgs::OccupancyGrid &map_msg)
    {
            occupGrid_Map = map_msg;
            costMap_init();
            CostMapCreation();
    }

    void costMap_init()
    {        
        occupGrid_Map.header.frame_id;
        costMap.header.stamp = ros::Time(0);
        costMap.header.frame_id = "map";
        costMap.info.resolution = occupGrid_Map.info.resolution;
        costMap.info.height = occupGrid_Map.info.height;
        costMap.info.width = occupGrid_Map.info.width;
        costMap.info.origin.position.x = occupGrid_Map.info.origin.position.x;
        costMap.info.origin.position.y = occupGrid_Map.info.origin.position.y;
        costMap.info.origin.position.z = occupGrid_Map.info.origin.position.z;
        int cellNumber = costMap.info.height *  costMap.info.width;
        costMap_vector = std::vector<signed char>(cellNumber,-1);
        width_robot = 10;
        height_robot = 10;



    }

    void CostMapCreation()
    {             
            int width_map = costMap.info.width;
            ROS_INFO("%d", width_map);
            for(int m = 0; m < width_map; m++)
            {
                for(int n = 0; n < width_map; n++)
                {
                    if(occupGrid_Map.data[m+width_map*n] == 150)
                    {
                        costMapUpdate(m,n);
                    }

                    if(occupGrid_Map.data[m+width_map*n] == 0)
                    {
                        if(costMap_vector[m+width_map*n]!=100)
                            costMap_vector[m+width_map*n] = 0;
                    }
                }

            }
            costMap.data = costMap_vector;
            costMap_publisher.publish(costMap);
            enableCostMap.data = false;
    }


    void costMapUpdate(int x_1, int y_1)
    {
        for(int i = x_1-(width_robot/2); i <= (x_1+(width_robot/2)); i++)
        {
            for(int j = y_1-(height_robot/2); j <= (y_1+(height_robot/2)); j++)
            {
                if(costMap_vector[i+costMap.info.width*j]!=100)
                    costMap_vector[i+costMap.info.width*j] = 100;
            }
        }
    }


private:


};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "obtain_map");

    createCostMap map;
    map.init();

    ros::Rate loop_rate(20.0);

    while(map.n.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


