#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include "geometry_msgs/PoseStamped.h"
#include "transforms/IrTransformMsg.h"

class OccupancyGrid
{
public:
    ros::NodeHandle n;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber sensor_subscriber;
    ros::Publisher occupancy_publisher;
    //ros::Publisher pose_publisher;
    double x_t, y_t, theta_t;
    int width_map, height_map;
    double resolution, center_x, center_y;
    int width_robot, height_robot; //[cell], preferably even number
    int x_pose_cell_map, y_pose_cell_map;
    int prev_x_pose_cell, prev_y_pose_cell;
    std::vector<signed char> map_vector;


    OccupancyGrid()
    {
        n = ros::NodeHandle("~");
        occupancy_grid_ = NULL;
    }

    ~OccupancyGrid()
    {
        delete occupancy_grid_;
    }

    void init()
    {
        occupancy_grid_ = new OccupancyGrid();
        // Subscribers:
        odometry_subscriber = n.subscribe("/arduino/odometry", 1, &OccupancyGrid::odometryCallback,this);
        sensor_subscriber = n.subscribe("/transformed_ir_points",1, &OccupancyGrid::sensorCallback,this);
        // Publishers:
        occupancy_publisher = n.advertise<nav_msgs::OccupancyGrid>("/nav/grid", 1);
        //pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/map/pose", 1);
    }


    void odometryCallback(const ras_arduino_msgs::Odometry::ConstPtr &pose_msg)
    {
        x_t = pose_msg->x;
        y_t = pose_msg->y;
        theta_t = pose_msg->theta;
        // 1. Update map every time we move to another cell
        x_pose_cell_map = floor((center_x + x_t)/resolution);
        y_pose_cell_map = floor((center_y - y_t)/resolution);
        // Initial values of previous values != than zero

        //width_robot=4;
        //height_robot=4;
        if((prev_x_pose_cell != x_pose_cell_map) || (prev_y_pose_cell != y_pose_cell_map))
        {
            for(int i = x_pose_cell_map-(width_robot/2); i <= (x_pose_cell_map+(width_robot/2)); i++)
            {
                for(int j = y_pose_cell_map-floor(height_robot/2); j <= (y_pose_cell_map+floor(height_robot/2)); j++)
                {
                    map_vector[i*width_map+j] = 0;
                }
            }
            prev_x_pose_cell = x_pose_cell_map;
            prev_y_pose_cell = y_pose_cell_map;
        }


        // 2. Check if sensors are giving measurements
        // 3. Update cell that has been sensed and in between cells
        /*
        if (s1==true)
        {
            mapUpdate(sensor_msg->p1.point.x, sensor_msg->p1.point.y,
                      x_t, y_t, theta_t, );
         }
        */
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

    void mapInit()
    {
        width_map=200; //[cell]
        height_map=200; //[cell]
        int cellNumber=width_map*height_map;
        resolution = 0.05; //[m/cell]
        map_vector = std::vector<signed char>(cellNumber,-1);
        center_x = width_map/2 * resolution; //[m]
        center_y = width_map/2 * resolution; //[m]
        prev_x_pose_cell=1000;
        prev_y_pose_cell=1000;
    }

    // Function needs working on
    /*void mapUpdate(double x_sens_dist, double y_sens_dist, double x_pose, double y_pose, double theta_pose)
    {
        //x_sens_dist and y_sens_dist are distance from robot to detected object.
        // Always start at center of grid: add/subtract to center_x to all distances
        // according to sign conventions.
        // First, convert sensor readings from robot frame to map frame then and then convert
        // --- obtain X coordenate of cells ---
        double x_sensed_dist_pose = x_sens_dist * cos(theta_pose);
        double x_pose_dist_map = center_x + x_pose; //add center for x
        double x_sensed_dist_map = x_pose_dist_map + x_sensed_dist_pose;
        int x_pose_cell = floor(x_pose_dist_map/resolution);
        int x_sensed_cell = floor(x_sensed_dist_map/resolution);

        // --- obtain Y coordenate of cells ---
        double y_sensed_dist_pose = y_sens_dist * sin(theta_pose);
        double y_pose_dist_map = center_y - y_pose; //add center for x
        double y_sensed_dist_map = y_pose_dist_map - y_sensed_dist_pose;
        int y_pose_cell = floor(y_pose_dist_map/resolution);
        int y_sensed_cell = floor(y_sensed_dist_map/resolution);

        // Fill all cells between pose cell and sensed cell with 0 or +1;
        // needs working on
        int j=0;
        for(int i = x_pose_cell; i <= x_sensed_cell; i++)
        {
            if((i!=x_sensed_cell))
            {
                //We will only set cells to 0 that are unseen
                if(map_vector[i*width_map+(j+y_pose_cell)]==-1)
                    map_vector[i*width_map+(j+y_pose_cell)]=0;
                j++;
            }
            else
            {
                map_vector[i*width_map+(j+y_pose_cell)] = map_vector[i*width_map+(j+y_pose_cell)] + 1;
            }
        }

    }*/

    void sensorCallback(const transforms::IrTransformMsg::ConstPtr &sensor_msg)
    {
        //Only if the measurement is valid will we get the points
        bool s1=sensor_msg->s1;

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

    OccupancyGrid *occupancy_grid_;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancy_grid");

    OccupancyGrid map;
    map.init();
    map.mapInit();
    ros::Rate loop_rate(10.0);

    while(map.n.ok())
    {
        ros::spinOnce();
        map.gridVisualize();
        loop_rate.sleep();
    }

    return 0;
}

