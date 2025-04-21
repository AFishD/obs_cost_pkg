#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>

class LaserObstacleAvoidance
{
    public:
    LaserObstacleAvoidance() :
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        costmap_ros_("custom_costmap", tf_buffer_),
        obstacle_range_(2.5), 
        max_obstacle_height_(0.6),
        raytrace_range_(3.0) 
    {
        laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LaserObstacleAvoidance::laserCallback, this);
        
        costmap_ = costmap_ros_.getCostmap();
        costmap_->setDefaultValue(costmap_2d::FREE_SPACE);
    }

    private:
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            // 1. 清除旧障碍物信息（raytracing）
            clearOldObstacles(scan_msg);

            // 2. 处理每个激光点
            for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
            {
                float range = scan_msg->ranges[i];
                if (range < scan_msg->range_min || range > scan_msg->range_max)
                {
                    continue; // 跳过无效数据
                }

                // 计算激光点角度
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                
                // 转换为笛卡尔坐标
                double x = range * cos(angle);
                double y = range * sin(angle);

                // 转换到地图坐标系
                geometry_msgs::PointStamped laser_point, map_point;
                laser_point.header = scan_msg->header;
                laser_point.point.x = x;
                laser_point.point.y = y;
                laser_point.point.z = 0;

                try
                {
                    // 使用tf2进行转换
                    map_point = tf_buffer_.transform
                    (
                        laser_point, 
                        costmap_ros_.getGlobalFrameID(), 
                        ros::Duration(1.0)
                    );
                } 
                catch (tf2::TransformException& ex)
                {
                    ROS_WARN("TF转换失败: %s", ex.what());
                    continue;
                }

                // 3. 更新代价值
                unsigned int mx, my;
                if (costmap_->worldToMap(map_point.point.x, map_point.point.y, mx, my))
                {
                    // 检查高度是否在有效范围内
                    if (fabs(map_point.point.z) <= max_obstacle_height_)
                    { 
                        costmap_->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
                    }
                }
            }
        }

        void clearOldObstacles(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            geometry_msgs::TransformStamped robot_transform;
            try
            {
                robot_transform = tf_buffer_.lookupTransform
                (
                    costmap_ros_.getGlobalFrameID(),
                    costmap_ros_.getBaseFrameID(),
                    ros::Time(0),
                    ros::Duration(1.0)
                );
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN("Can not get robot pos: %s", ex.what());
                return;
            }

            // 获取机器人世界坐标
            double robot_x = robot_transform.transform.translation.x;
            double robot_y = robot_transform.transform.translation.y;
            double clear_radius = raytrace_range_;
            
            unsigned int min_x, min_y, max_x, max_y;
            if (!costmap_->worldToMap(robot_x - clear_radius, robot_y - clear_radius, min_x, min_y)) return;
            if (!costmap_->worldToMap(robot_x + clear_radius, robot_y + clear_radius, max_x, max_y)) return;

            min_x = std::max(0, static_cast<int>(min_x));
            min_y = std::max(0, static_cast<int>(min_y));
            max_x = std::min(static_cast<int>(costmap_->getSizeInCellsX()), static_cast<int>(max_x));
            max_y = std::min(static_cast<int>(costmap_->getSizeInCellsY()), static_cast<int>(max_y));

            for (unsigned int y = min_y; y < max_y; ++y)
            {
                for (unsigned int x = min_x; x < max_x; ++x)
                {
                    if (hypot(x - robot_x, y - robot_y) <= clear_radius)
                    {
                        costmap_->setCost(x, y, costmap_2d::FREE_SPACE);
                    }
                }
            }
        }

        ros::NodeHandle nh_;
        ros::Subscriber laser_sub_;

        tf2_ros::Buffer             tf_buffer_;
        tf2_ros::TransformListener  tf_listener_;
        costmap_2d::Costmap2DROS    costmap_ros_;
        costmap_2d::Costmap2D*      costmap_;

        double obstacle_range_;
        double max_obstacle_height_;
        double raytrace_range_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_obstacle_avoidance");
    LaserObstacleAvoidance processor;
    ros::spin();
    return 0;
}