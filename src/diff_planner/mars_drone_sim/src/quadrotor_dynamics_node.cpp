
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_dynamics.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h> // 添加头文件
#include <random>

using namespace std;
using namespace Eigen;
// 定义风扰动结构体
struct WindDisturbance
{
    bool active;       // 风扰动是否激活
    Vector3d force;    // 风力向量 (N)
    Vector3d aabb_min; // AABB区域最小点
    Vector3d aabb_max; // AABB区域最大点
    Vector3d center;   // 风力中心点 (x,y,z)
    double strength;   // 风力强度 (N)
};
// 全局风扰动变量
WindDisturbance wind_effect = {
    .active = false,
    .force = Vector3d(0, 0, 0),          // 关注这个向量在哪里被赋值
    .aabb_min = Vector3d(5, -6, 0),      // 区域最小点 (x,y,z)
    .aabb_max = Vector3d(10, -1, 4),     // 区域最大点 (x,y,z)
    .center = Vector3d(-19.5, 4.0, 1.0), // 默认风力中心点
    .strength = 15.0                     // 默认风力强度
};
// bool have_odom_ = false;
std_msgs::Float32MultiArray RPM_msg;
Vector4d RPM_input;
void RPMCallbck(const std_msgs::Float32MultiArray &msg)
{
    // have_odom_ = true;
    RPM_msg = msg;
    RPM_input << RPM_msg.data[0], RPM_msg.data[1], RPM_msg.data[2], RPM_msg.data[3];
}

// 检查点是否在AABB区域内
bool isInWindZone(const Vector3d &position,
                  const Vector3d &aabb_min,
                  const Vector3d &aabb_max)
{
    return position.x() >= aabb_min.x() && position.x() <= aabb_max.x() &&
           position.y() >= aabb_min.y() && position.y() <= aabb_max.y() &&
           position.z() >= aabb_min.z() && position.z() <= aabb_max.z();
}

// 计算风力 (可自定义风场模型)
Vector3d calculateWindForce(const Vector3d &position, double strength, const Vector3d &center)
{
    // 简单恒定风模型
    // return Vector3d(5.0, 3.0, 0);  // 恒定风力 (5N, 3N, 0N)

    // 更真实的风场模型 (位置相关)
    // double strength = 15.0; // 最大风力强度 (N)

    // 计算到区域中心的距离
    // Vector3d center = (wind_effect.aabb_min + wind_effect.aabb_max) * 0.5;
    // Vector3d center;
    // center(0) = -19.5;
    // center(1) = 4.0;
    //  center(2) = 1.0;
    Vector3d dir = (position - center).normalized();

    // 湍流效果：添加随机扰动
    static default_random_engine generator;
    static normal_distribution<double> distribution(0.0, 0.3);
    double turb = distribution(generator);

    // 风力随远离中心减弱
    double dist_factor = 1.0 - min(1.0, (position - center).norm() / 5.0);

    return Vector3d(
        (dir.x() + turb) * strength * dist_factor,
        (dir.y() + turb) * strength * dist_factor,
        (dir.z() * 0.5) * strength * dist_factor);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quadrotor_dynamics");
    //  ros::NodeHandle nh;
    ros::NodeHandle n("~");
    double init_x, init_y, init_z, mass;
    double simulation_rate;
    std::string quad_name;
    n.param("mass", mass, 0.9);
    n.param("init_state_x", init_x, 0.0);
    n.param("init_state_y", init_y, 0.0);
    n.param("init_state_z", init_z, 1.0);
    n.param("simulation_rate", simulation_rate, 200.0);
    n.param("quadrotor_name", quad_name, std::string("quadrotor"));

    n.param("wind_effect/active", wind_effect.active, false);
    n.param("wind_effect/strength", wind_effect.strength, 15.0); // 风力强度
    n.param("wind_effect/min_x", wind_effect.aabb_min.x(), 5.0);
    n.param("wind_effect/min_y", wind_effect.aabb_min.y(), -6.0);
    n.param("wind_effect/min_z", wind_effect.aabb_min.z(), 0.0);
    n.param("wind_effect/max_x", wind_effect.aabb_max.x(), 10.0);
    n.param("wind_effect/max_y", wind_effect.aabb_max.y(), -1.0);
    n.param("wind_effect/max_z", wind_effect.aabb_max.z(), 4.0);

    // 读取风力中心点参数
    n.param("wind_effect/center_x", wind_effect.center.x(), -19.5);
    n.param("wind_effect/center_y", wind_effect.center.y(), 4.0);
    n.param("wind_effect/center_z", wind_effect.center.z(), 1.0);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("wind_zone_marker", 1);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
    // ros::Subscriber cmd_sub = n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber rpm_sub = n.subscribe("cmd_RPM", 100, RPMCallbck);

    Matrix3d Internal_mat;
    Internal_mat << 2.64e-3, 0, 0,
        0, 2.64e-3, 0,
        0, 0, 4.96e-3;//机体系转到世界系
    quadrotor_dynamics quadrotor(mass, Internal_mat);

    Vector3d init_pos;
    Vector4d init_q, actuator;
    init_pos << init_x, init_y, init_z;
    // init_q << 1, 0, 0.0, 0.0;
    // init_q << 0.0, 0.0, 0.0, 1.0;
    const double yaw = 0 * M_PI / 180.0;
    init_q << cos(yaw * 0.5), 0.0, 0.0, sin(yaw * 0.5);
    quadrotor.init(init_pos, init_q);
    actuator << 0.0, 0.0, 0.0, 0.0;
    // quadrotor.setActuatoroutput(actuator);

    // 风扰动状态跟踪
    bool was_in_wind_zone = false;
    double wind_activation_time = 0.0;
    double last_wind_update = 0.0;
    std::cout << "wind_effect.active = " << wind_effect.active << std::endl;
    ros::Rate rate(simulation_rate);
    rate.sleep();

    ros::Time last_time = ros::Time::now();

    while (n.ok())
    {

            ros::spinOnce();
            // ROS_INFO("RPM_input = %.2f,%.2f,%.2f,%.2f", RPM_input(0), RPM_input(1), RPM_input(2), RPM_input(3));
            quadrotor.setRPM(RPM_input);

            ros::Time now_time = ros::Time::now();
            // ROS_INFO("dt = %lf", (now_time-last_time).toSec());
            quadrotor.step_forward((now_time - last_time).toSec());
            last_time = now_time;
            // ROS_ERROR("wind_effect.active = 1%d", wind_effect.active);
            // 获取当前位置
            Vector3d pos = quadrotor.getPos();
            // 发布风区可视化标记
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world"; // 使用与odom相同的坐标系
            marker.header.stamp = ros::Time::now();
            marker.ns = "wind_zone";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            // 计算AABB中心点和尺寸
            Vector3d center = (wind_effect.aabb_min + wind_effect.aabb_max) * 0.5;
            Vector3d scale = wind_effect.aabb_max - wind_effect.aabb_min;

            marker.pose.position.x = center.x();
            marker.pose.position.y = center.y();
            marker.pose.position.z = center.z();
            marker.pose.orientation.w = 1.0; // 无旋转

            marker.scale.x = scale.x();
            marker.scale.y = scale.y();
            marker.scale.z = scale.z();

            // 设置颜色（红色半透明）
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.3; // 半透明

            marker.lifetime = ros::Duration(); // 一直存在
            marker_pub.publish(marker);
            // ROS_ERROR("wind_effect.active = 2%d", wind_effect.active);
            // std::cout<<"wind_effect.active = "<< wind_effect.active <<std::endl;
            // 检查是否在风区并更新风力
            if (wind_effect.active)
            {
                bool in_wind_zone = isInWindZone(pos, wind_effect.aabb_min, wind_effect.aabb_max);

                if (in_wind_zone)
                {
                    // 首次进入风区时初始化风力
                    if (!was_in_wind_zone)
                    {
                        wind_activation_time = now_time.toSec();
                        was_in_wind_zone = true;
                        ROS_WARN("Entering wind zone! Position: [%.2f, %.2f, %.2f]",
                                 pos.x(), pos.y(), pos.z());
                    }

                    // 每0.5秒更新一次风力 (模拟阵风)
                    if (now_time.toSec() - last_wind_update > 0.1)
                    {
                        wind_effect.force = calculateWindForce(pos, wind_effect.strength, wind_effect.center);
                        last_wind_update = now_time.toSec();
                        ROS_WARN("Applying wind disturbance: [%.2f N, %.2f N, %.2f N]",
                                 wind_effect.force.x(), wind_effect.force.y(), wind_effect.force.z());
                    }

                    // 应用风力
                    quadrotor.addExternalForce(wind_effect.force);
                    ros::Time now_time = ros::Time::now();
                    quadrotor.step_forward((now_time - last_time).toSec());
                    last_time = now_time;

                    ROS_WARN("Applyed wind disturbance.");
                }
                else if (was_in_wind_zone)
                {
                    // 离开风区
                    was_in_wind_zone = false;
                    quadrotor.clearExternalForce();
                    ROS_WARN("Exiting wind zone!");
                }
            }

            Vector3d vel, acc, angular_vel;
            // publish odometry
            nav_msgs::Odometry odom;
            odom.header.frame_id = "world";
            odom.header.stamp = now_time;
            Vector3d angular_vel_world;
            pos = quadrotor.getPos();
            vel = quadrotor.getVel();
            acc = quadrotor.getAcc();
            angular_vel = quadrotor.getAngularVel();
            Vector4d quat;
            quat = quadrotor.getQuat();
            Matrix3d R_body2world;
            R_body2world = quadrotor.getR();
            angular_vel_world = R_body2world * angular_vel;

            odom.pose.pose.position.x = pos(0);
            odom.pose.pose.position.y = pos(1);
            odom.pose.pose.position.z = pos(2);
            odom.pose.pose.orientation.w = quat(0);
            odom.pose.pose.orientation.x = quat(1);
            odom.pose.pose.orientation.y = quat(2);
            odom.pose.pose.orientation.z = quat(3);
            odom.twist.twist.linear.x = vel(0);
            odom.twist.twist.linear.y = vel(1);
            odom.twist.twist.linear.z = vel(2);
            odom.twist.twist.angular.x = angular_vel_world(0);
            odom.twist.twist.angular.y = angular_vel_world(1);
            odom.twist.twist.angular.z = angular_vel_world(2);
            odom_pub.publish(odom);

            // ROS_INFO("Odom = %f,%f,%f, %f,%f,%f,%f", pos(0), pos(1), pos(2), quat(0), quat(1), quat(2), quat(3));

            // imu generate
            sensor_msgs::Imu imu_msg;
            imu_msg.header.frame_id = "/" + quad_name;
            imu_msg.header.stamp = now_time;
            imu_msg.orientation.w = quat(0);
            imu_msg.orientation.x = quat(1);
            imu_msg.orientation.y = quat(2);
            imu_msg.orientation.z = quat(3);
            imu_msg.angular_velocity.x = angular_vel(0);
            imu_msg.angular_velocity.y = angular_vel(1);
            imu_msg.angular_velocity.z = angular_vel(2);
            acc = R_body2world.inverse() * (acc + Eigen::Vector3d(0, 0, -9.8));
            imu_msg.linear_acceleration.x = acc(0);
            imu_msg.linear_acceleration.y = acc(1);
            imu_msg.linear_acceleration.z = acc(2);
            imu_pub.publish(imu_msg);
            rate.sleep();

    }

    return 0;
}