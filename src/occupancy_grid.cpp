#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "robot_msgs/IrTransformMsg.h"
#include "tf/transform_listener.h"
#include "std_msgs/Bool.h"
#include "visualization_msgs/Marker.h"
#include "signal.h"
#include "rosbag/bag.h"


class OccupancyGrid
{
public:
    ros::NodeHandle n;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber sensor_subscriber;
    ros::Publisher occupancy_publisher;
    ros::Subscriber object_subscriber;
    ros::Subscriber turn_sub;
    ros::Subscriber mappingDone_sub;
    ros::Publisher pose_publisher;
    ros::Publisher object_publisher;
    ros::Publisher costMap_publisher;
    double x_t, y_t, theta_t;
    int width_map, height_map;
    double resolution, center_x, center_y;
    int width_robot, height_robot; //[cell], preferably even number
    int x_pose_cell_map, y_pose_cell_map;
    int prev_x_pose_cell, prev_y_pose_cell;
    std_msgs::Bool turning, mapping_done;
    //geometry_msgs::PointStamped object_pose;
    robot_msgs::IrTransformMsg sensor_msg;
    tf::TransformListener listener;
    nav_msgs::OccupancyGrid grid_msg;

    std::vector<signed char> map_vector, final_map, cost_map;

    OccupancyGrid()
    {
        n = ros::NodeHandle();
        //occupancy_grid_ = NULL;
    }

    ~OccupancyGrid()
    {
        //delete occupancy_grid_;
    }

    void init()
    {
        //occupancy_grid_ = new OccupancyGrid();
        // Subscribers:
        odometry_subscriber = n.subscribe("/arduino/odometry", 1, &OccupancyGrid::odometryCallback,this);
        sensor_subscriber = n.subscribe("/transformed_ir_points",1, &OccupancyGrid::sensorCallback,this);
        object_subscriber = n.subscribe("/object_pose",1, &OccupancyGrid::objectCallback, this);
        //turn_sub = n.subscribe("/robot_turn", 1, &OccupancyGrid::turnCallback, this);
        mappingDone_sub = n.subscribe("/map_done", 1, &OccupancyGrid::mapDoneCallback, this);
        // Publishers:
        occupancy_publisher = n.advertise<nav_msgs::OccupancyGrid>("/gridmap", 1);
        costMap_publisher = n.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
        pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/map/pose", 1);
        object_publisher = n.advertise<visualization_msgs::Marker>("/object_marker",1);        
    }


    void odometryCallback(const ras_arduino_msgs::Odometry::ConstPtr &pose_msg)
    {
        x_t = pose_msg->x;
        y_t = pose_msg->y;
        theta_t = pose_msg->theta;
        // 1. Update map every time we move to another cell
        x_pose_cell_map = floor((center_x + x_t)/resolution);
        y_pose_cell_map = floor((center_y + y_t)/resolution);
        // Initial values of previous values != than zero

        if((prev_x_pose_cell != x_pose_cell_map) || (prev_y_pose_cell != y_pose_cell_map))
        {
            for(int i = x_pose_cell_map-(width_robot/2); i <= (x_pose_cell_map+(width_robot/2)); i++)
            {
                for(int j = y_pose_cell_map-floor(height_robot/2); j <= (y_pose_cell_map+floor(height_robot/2)); j++)
                {

                    if(cost_map[i + width_map*j] == 100)
                    {
                        costMapUpdate(i, j, 0);
                    }
                    map_vector[i+width_map*j] = 1;

                }
            }
            prev_x_pose_cell = x_pose_cell_map;
            prev_y_pose_cell = y_pose_cell_map;
        }

        if(sensor_msg.s1 == true)
        {
            mapUpdate(sensor_msg.p1.point.x,sensor_msg.p1.point.y,1);
        }

        if(sensor_msg.s2 == true)
        {
            mapUpdate(sensor_msg.p2.point.x,sensor_msg.p2.point.y,2);
        }

        if(sensor_msg.s3 == true)
        {
            mapUpdate(sensor_msg.p3.point.x,sensor_msg.p3.point.y,3);
        }

        if(sensor_msg.s4 == true)
        {
            mapUpdate(sensor_msg.p4.point.x,sensor_msg.p4.point.y,4);
        }



        //In case we want to view our direction
        //Publish pose for visualization
        geometry_msgs::PoseStamped poseStamp_msg;
        poseStamp_msg.header.frame_id = "robot_center";
        poseStamp_msg.header.stamp = ros::Time(0);
        poseStamp_msg.pose.position.x = 0;
        poseStamp_msg.pose.position.y = 0;
        poseStamp_msg.pose.position.z = 0;
        tf::Quaternion q;
        q.setEuler(0.0, 0.0, M_PI_2);
        tf::quaternionTFToMsg(q, poseStamp_msg.pose.orientation);
        pose_publisher.publish(poseStamp_msg);

        // Estimate cells to update here

    }

    void mapDoneCallback(const std_msgs::Bool &msg) {
        mapping_done = msg;
    }

    void objectCallback(const geometry_msgs::Pose &obj_msg)
    {
        //object_pose = obj_msg;

        visualization_msgs::Marker object;
        object.header.frame_id = "map";
        object.header.stamp = ros::Time(0);
        object.ns = "object0";
        object.id = 0;
        object.type = visualization_msgs::Marker::CUBE;
        object.action = visualization_msgs::Marker::ADD;
        object.pose.position.x = obj_msg.position.x;
        object.pose.position.y = obj_msg.position.y;
        object.pose.position.z = obj_msg.position.z;
        object.pose.orientation.w = obj_msg.orientation.w;
        object.pose.orientation.x = obj_msg.orientation.x;
        object.pose.orientation.y = obj_msg.orientation.y;
        object.pose.orientation.z = obj_msg.orientation.z;
        object.scale.x = 0.1;
        object.scale.y = 0.1;
        object.scale.z = 0.1;
        object.color.a = 1.0;
        object.color.r = 1.0;
        object.color.g = 1.0;
        object.color.b = 0;

        object_publisher.publish( object );



    }

    void mapInit()
    {
        // Map set to be 10mx10m
        width_map=500; //[cell]
        height_map=500; //[cell]
        int cellNumber=width_map*height_map;
        resolution = 0.02; //[m/cell]
        map_vector = std::vector<signed char>(cellNumber,-1);
        final_map = std::vector<signed char>(cellNumber,-1);
        cost_map = std::vector<signed char>(cellNumber,-1);
        center_x = width_map/2 * resolution; //[m]
        center_y = width_map/2 * resolution; //[m]
        prev_x_pose_cell=1000;
        prev_y_pose_cell=1000;
        width_robot=10; //[cell]
        height_robot=10; //[cell]
        turning.data=false;

        // The time at which the map was loaded
        grid_msg.header.stamp=ros::Time(0);
        grid_msg.header.frame_id = "map";
        //init_grid_msg.info.map_load_time = ros::Time();

        grid_msg.info.resolution = resolution;
        // Set width [cell] and height [cell] of grid
        grid_msg.info.height = height_map;
        grid_msg.info.width = width_map;
        // The time at which the map was loaded
        //grid_msg.info.map_load_time = ros::Time();
        // Set origin of map in [m,m,rad]
        grid_msg.info.origin.position.x = 0;
        grid_msg.info.origin.position.y = 0;
        grid_msg.info.origin.position.z = 0;
        mapping_done.data = false;

    }

    // Function needs working on
    void mapUpdate(double x_sens_dist, double y_sens_dist, int sensor)
    {

        //x_sens_dist and y_sens_dist are distance from robot to detected object.
        // Always start at center of grid: add/subtract to center_x to all distances
        // according to sign conventions.
        // First, convert sensor readings from robot frame to map frame then and then convert

        int x_sens_cell = floor(x_sens_dist/resolution);
        int y_sens_cell = floor(y_sens_dist/resolution);
        int weight_cell = 5;
        double corner_x, corner_y;
        int corner_x_cell, corner_y_cell;
        tf::StampedTransform transform;
        switch(sensor)
        {
            case 1:
            try
            {
                listener.waitForTransform("/sensor1", "/map", ros::Time(0), ros::Duration(1));
                listener.lookupTransform("/map", "/sensor1", ros::Time(0), transform);
                corner_x = transform.getOrigin().getX();
                corner_y = transform.getOrigin().getY();
                corner_x_cell = floor(corner_x/resolution);
                corner_y_cell = floor(corner_y/resolution);
                ros::Time::init();
            }
                catch(tf::TransformException ex)
                {
                    ROS_ERROR("%s",ex.what());
                }
            break;


            case 2:
            try
            {
                listener.waitForTransform("/sensor2", "/map", ros::Time(0), ros::Duration(1));
                listener.lookupTransform("/map", "/sensor2", ros::Time(0), transform);
                corner_x = transform.getOrigin().getX();
                corner_y = transform.getOrigin().getY();
                corner_x_cell = floor(corner_x/resolution);
                corner_y_cell = floor(corner_y/resolution);
            }
                catch(tf::TransformException ex)
            {
                    ROS_ERROR("%s",ex.what());
            }
            break;

            case 3:
            try
            {
                listener.waitForTransform("/sensor3", "/map", ros::Time(0), ros::Duration(1));
                listener.lookupTransform("/map", "/sensor3", ros::Time(0), transform);
                corner_x = transform.getOrigin().getX();
                corner_y = transform.getOrigin().getY();
                corner_x_cell = floor(corner_x/resolution);
                corner_y_cell = floor(corner_y/resolution);
            }
                catch(tf::TransformException ex)
            {
                    ROS_ERROR("%s",ex.what());
            }
            break;

            case 4:
            try
            {
                listener.waitForTransform("/sensor4", "/map", ros::Time(0), ros::Duration(1));
                listener.lookupTransform("/map", "/sensor4", ros::Time(0), transform);
                corner_x = transform.getOrigin().getX();
                corner_y = transform.getOrigin().getY();
                corner_x_cell = floor(corner_x/resolution);
                corner_y_cell = floor(corner_y/resolution);
            }
                catch(tf::TransformException ex)
            {
                    ROS_ERROR("%s",ex.what());
            }

        }


        //map_vector[x_sens_cell+width_map*y_sens_cell]=100;
        int x1, x2, y1, y2;
        if (corner_x_cell > x_sens_cell) {
            x1 = x_sens_cell;
            x2 = corner_x_cell;
        } else {
            x1 = corner_x_cell;
            x2 = x_sens_cell;
        }
        if (corner_y_cell > y_sens_cell) {
            y1 = y_sens_cell;
            y2 = corner_y_cell;
        } else {
            y1 = corner_y_cell;
            y2 = y_sens_cell;
        }
        for(int i = x2 ; i >= x1; i--)
        {
            for(int j = y2 ; j >= y1; j--)
            {
                if((i==x_sens_cell) && (j==y_sens_cell))
                {
                    if(map_vector[i+width_map*j] == -1)
                        map_vector[i+width_map*j] = 5;
                    else
                        map_vector[i+width_map*j] = map_vector[i+width_map*j] + weight_cell;
                }
                else
                {
                    if(map_vector[i+width_map*j]==-1)
                    {
                        map_vector[i+width_map*j] = 0;
                    }
                }
            }
        }


        /*
        for(int i = x1 ; i <= x2; i++)
        {
            for(int j = y1 ; j <= y2; j++)
            {
                if((i==x_sens_cell) && (j==y_sens_cell)) {
                   // if (map_vector[i+width_map*j] != 0)
                    if(turning.data == true)
                    {
                        if(map_vector[i+width_map*j] == -1)
                            map_vector[i+width_map*j] = 100;
                    }
                    else
                    {
                        map_vector[i+width_map*j] = 100;
                    }

                }
                else
                {
                    if(turning.data == true)
                    {
                        if(map_vector[i+width_map*j] == -1)
                            map_vector[i+width_map*j] = 0;
                    }
                    else
                    {
                        if(map_vector[i+width_map*j] == -1)
                            map_vector[i+width_map*j] = 0;
                    }

                }
            }
        }*/



        // Fill all cells between pose cell and sensed cell with 0 or +1;
        // needs working on


    }

    void sensorCallback(const robot_msgs::IrTransformMsg &msg)
    {
        //Only if the measurement is valid will we get the points

        sensor_msg = msg;

    }

    void gridVisualize()
    {

        int threshold = 10;
        // Create map
        for(int k = 0; k < width_map; k++)
        {
            for(int l=0; l < width_map; l++)
            {
                if((map_vector[k+width_map*l] >threshold) && ((map_vector[k+width_map*l] % 5) == 0))
                {
                    final_map[k+width_map*l] = 150;
                    costMapUpdate(k,l, 100);
                }

                if((map_vector[k+width_map*l] <= threshold) && (map_vector[k+width_map*l]>=0))
                {
                    final_map[k+width_map*l] = 0;
                    if(cost_map[k+width_map*l] != 100)
                        cost_map[k+width_map*l] = 0;
                }

            }
        }

        grid_msg.data = final_map;
        occupancy_publisher.publish(grid_msg);


        //costMapCreate();
        grid_msg.data = cost_map;
        costMap_publisher.publish(grid_msg);


    }

    void costMapUpdate(int x_1, int y_1, int value)
    {

        for(int i = x_1-(width_robot/2); i <= (x_1+(width_robot/2)); i++)
        {
            for(int j = y_1-(height_robot/2); j <= (y_1+(height_robot/2)); j++)
            {
                if(cost_map[i + width_map*j] != value){
                    cost_map[i + width_map*j] = value;

                }
            }
        }
    }

    nav_msgs::OccupancyGrid get_map()
    {
        grid_msg.data = final_map;
        return grid_msg;
    }

    nav_msgs::OccupancyGrid get_costMap()
    {
        grid_msg.data = cost_map;
        return grid_msg;
    }


private:

    //OccupancyGrid *occupancy_grid_;


};

OccupancyGrid* _map;

void save_maps(int sig)
{
    std::cout<<"Record Bag \n";
    rosbag::Bag bag;
    bag.open("map_test_2.bag", rosbag::bagmode::Write);
    nav_msgs::OccupancyGrid msg = _map->get_map();
    bag.write("/gridmap", ros::Time::now(), msg);
    bag.close();
    ros::shutdown();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancy_grid");

    OccupancyGrid map;
    _map = &map;

    map.init();
    map.mapInit();

    ros::Rate loop_rate(20.0);

    while(map.n.ok())
    {
        ros::spinOnce();
        map.gridVisualize();
        loop_rate.sleep();
    }

    save_maps(1);

    return 0;
}

