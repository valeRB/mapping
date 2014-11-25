#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include "geometry_msgs/PoseStamped.h"
#include "transforms/IrTransformMsg.h"
#include "tf/transform_listener.h"
#include "std_msgs/Bool.h"

class OccupancyGrid
{
public:
    ros::NodeHandle n;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber sensor_subscriber;
    ros::Publisher occupancy_publisher;
    ros::Subscriber turn_sub;
    //ros::Publisher pose_publisher;
    double x_t, y_t, theta_t;
    int width_map, height_map;
    double resolution, center_x, center_y;
    int width_robot, height_robot; //[cell], preferably even number
    int x_pose_cell_map, y_pose_cell_map;
    int prev_x_pose_cell, prev_y_pose_cell;
    std_msgs::Bool turning;
    transforms::IrTransformMsg sensor_msg;
    tf::TransformListener listener;

    std::vector<signed char> map_vector;


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
        // Publishers:
        occupancy_publisher = n.advertise<nav_msgs::OccupancyGrid>("/nav/grid", 1);
        //pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/map/pose", 1);
        turn_sub = n.subscribe("/robot_turn", 1, &OccupancyGrid::turnCallback, this);
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

        width_robot=10; //[cell]
        height_robot=10; //[cell]

        if((prev_x_pose_cell != x_pose_cell_map) || (prev_y_pose_cell != y_pose_cell_map))
        {
            for(int i = x_pose_cell_map-(width_robot/2); i <= (x_pose_cell_map+(width_robot/2)); i++)
            {
                for(int j = y_pose_cell_map-floor(height_robot/2); j <= (y_pose_cell_map+floor(height_robot/2)); j++)
                {
                    map_vector[i+width_map*j] = 0;

                }
            }
            prev_x_pose_cell = x_pose_cell_map;
            prev_y_pose_cell = y_pose_cell_map;
        }

        // 2. Check if sensors are giving measurements
        if (turning.data == false) {
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
        }

        // 3. Update cell that has been sensed and in between cells

        /*
        In case we want to view our direction
        //Publish pose for visualization
        geometry_msgs::PoseStamped poseStamp_msg;
        poseStamp_msg.header.frame_id = "map";
        poseStamp_msg.header.stamp = ros::Time();
        poseStamp_msg.pose.position.x = 0;
        poseStamp_msg.pose.position.y = 0;
        poseStamp_msg.pose.position.z = 0;
        poseStamp_msg.pose.orientation.x = 0;
        poseStamp_msg.pose.orientation.y = 0;
        poseStamp_msg.pose.orientation.z = 0;
        poseStamp_msg.pose.orientation.w = 1;

        pose_publisher.publish(poseStamp_msg);
        */
        // Estimate cells to update here

    }

    void turnCallback(const std_msgs::Bool &msg) {
        turning = msg;
    }

    void mapInit()
    {
        // Map set to be 10mx10m
        width_map=500; //[cell]
        height_map=500; //[cell]
        int cellNumber=width_map*height_map;
        resolution = 0.02; //[m/cell]
        map_vector = std::vector<signed char>(cellNumber,-1);
        center_x = width_map/2 * resolution; //[m]
        center_y = width_map/2 * resolution; //[m]
        prev_x_pose_cell=1000;
        prev_y_pose_cell=1000;
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
        ROS_INFO("Corner X %f", corner_x);
        ROS_INFO("Corner Y %f", corner_y);
        ROS_INFO("Wall X %f", x_sens_dist);
        ROS_INFO("Wall Y %f", y_sens_dist);

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

        for(int i = x1 ; i <= x2; i++)
        {
            for(int j = y1 ; j <= y2; j++)
            {
                if((i==x_sens_cell) && (j==y_sens_cell)) {
                   // if (map_vector[i+width_map*j] != 0)
                        map_vector[i+width_map*j] = 100;
                }
                else
                {
                    map_vector[i+width_map*j] = 0;

                }
            }
        }



        // Fill all cells between pose cell and sensed cell with 0 or +1;
        // needs working on


    }

    void sensorCallback(const transforms::IrTransformMsg &msg)
    {
        //Only if the measurement is valid will we get the points

        sensor_msg = msg;

    }

    void gridVisualize()
    {
        nav_msgs::OccupancyGrid grid_msg;

        // The time at which the map was loaded
        grid_msg.header.stamp=ros::Time();
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

        grid_msg.data = map_vector;


        occupancy_publisher.publish(grid_msg);

    }

    /*
    void sensorCallback(const something::something::ConstPtr &sensor_msg)
    {

    }
    */


private:

    //OccupancyGrid *occupancy_grid_;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancy_grid");

    OccupancyGrid map;
    map.init();
    map.mapInit();
    ros::Rate loop_rate(20.0);

    while(map.n.ok())
    {
        ros::spinOnce();
        map.gridVisualize();
        loop_rate.sleep();
    }

    return 0;
}

