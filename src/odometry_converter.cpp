#include "ros/ros.h"
#include "xbot_msgs/WheelTick.h"
#include "xbot_msgs/AbsolutePose.h"
#include <nav_msgs/Odometry.h>
#include "xbot_msgs/AbsolutePose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "robot_localization_om/GPSControlSrv.h"
#include "robot_localization_om/SetPoseSrv.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "robot_localization/SetPose.h"

ros::Publisher imu_pub, filtered_imu_pub, odometry_pub, pose_pub, open_mower_pose_pub, amcl_initial_publisher, amcl_filtered_publisher;
tf2_ros::Buffer tfBuffer;

sensor_msgs::Imu imu;
bool has_ticks;
xbot_msgs::WheelTick last_ticks;
nav_msgs::Odometry odometry;
geometry_msgs::PoseWithCovarianceStamped pose;
std::string odometry_frame_id = "odom";
double vx = 0.0;
bool is_reversing = false;

double cov_factor_pos = 10;
double cov_factor_ori = 100;
double cov_factor_float_pos = 500;
double cov_factor_float_ori = 5000;
double orientation_min_speed = 0.01;
double antenna_offset_x = 0.15;
double antenna_offset_y = 0.0;
double min_position_accuracy = 0.05;
double float_damping_factor = 10.0;
double max_covariance = 0.5;
double acceleration_covariance_factor = 100.0;
double yaw_covariance_factor = 100.0;
double amcl_covariance_factor = 10.0;
double fully_fixed_wait_time = 20.0; // seconds
bool useOdomFrame = false;
double fixed_yaw_covariance = 0.0001;
double fixed_acceleration_covariance = 0.0001;
double wheel_ticks_per_m = 285.0;

bool has_gyro;
sensor_msgs::Imu filtered_imu;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;
double accelerometer_offset;
bool positioning_initialized = false;
bool amcl_initialized = false;

ros::Time last_gps_fixed_time(0.0), last_gps_float_time(0.0), last_gps_fully_fixed_time(0.0);
nav_msgs::Odometry last_odometry;
ros::Time last_filter_init(0.0), last_good_odometry(0.0);
// Throttle publishing of pose when GPS is not RTK fixed (max ~2 Hz)
ros::Time last_unfixed_pose_publish(0.0);

std::recursive_mutex odom_mutex;

ros::ServiceClient positioningClientMap, positioningClientOdom; 

bool gps_enabled = true;

bool setGpsState(robot_localization_om::GPSControlSrvRequest &req, robot_localization_om::GPSControlSrvResponse &res) {
    gps_enabled = req.gps_enabled;
    last_gps_float_time = ros::Time::now();
    if (!gps_enabled) {
        amcl_initialized = false;
    }
    return true;
}

void setRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &new_pose) {
    ros::Rate retry_delay(1);
    robot_localization::SetPose loc_pose_srv;
    loc_pose_srv.request.pose.pose = new_pose->pose;
    //loc_pose_srv.request.pose.pose.covariance = new_pose.pose.covariance;
    loc_pose_srv.request.pose.header.frame_id = "map";
    for(int i = 0; i < 10; i++) {
        if(positioningClientMap.call(loc_pose_srv)) {
            break;
        }
        ROS_ERROR_STREAM("Error setting robot pose (map) to " << loc_pose_srv.request.pose << ". Retrying.");
        retry_delay.sleep();
    }
    if (useOdomFrame) {
        loc_pose_srv.request.pose.header.frame_id = "odom";
        for(int i = 0; i < 10; i++) {
            if(positioningClientOdom.call(loc_pose_srv)) {
                break;
            }
            ROS_ERROR_STREAM("Error setting robot pose (odom) to " << loc_pose_srv.request.pose << ". Retrying.");
            retry_delay.sleep();
        }
    }
    ROS_INFO_STREAM_THROTTLE(1, "odom_converter: setting amcl initial pose to " << new_pose->pose);
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose = new_pose->pose;
    amcl_initial_publisher.publish(pose);
}

bool setPoseFromService(robot_localization_om::SetPoseSrvRequest &req, robot_localization_om::SetPoseSrvResponse &res) {
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg_ptr = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(req.robot_pose);
    setRobotPose(msg_ptr);
    return true;
}

void onGPS(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    if (!gps_enabled && positioning_initialized) {
        return;
    }

    bool is_fixed = false;
    if ((msg->flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) == xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) {
        last_gps_float_time = ros::Time::now();
    } else {
        // when the state has not been updated for a while, the GPS was probably turned off and both float and fixed times are way off
        // in this case we should assume that gps has not been fixed for long enough and we init last_gps_float_time
        if ((ros::Time::now() - last_gps_fixed_time).toSec() > 10.0) {
            last_gps_float_time = ros::Time::now();
        }
        // if the fixed signal is recovered since more than 10 seconds, and the fixed signal time is recent, then we consider it fixed
        if (last_gps_float_time.isZero() || last_gps_float_time < last_gps_fixed_time) {
            auto now = ros::Time::now();
            auto seconds_since_float = (now - last_gps_float_time).toSec();
            auto seconds_since_fixed = (now - last_gps_fixed_time).toSec();
            if ((seconds_since_float > fully_fixed_wait_time) && (seconds_since_fixed < 1.0)) {
                is_fixed = true;
            }
        }
        last_gps_fixed_time = ros::Time::now();
    }
    if (is_fixed) {
        last_gps_fully_fixed_time = ros::Time::now();
    }
    // min_position_accuracy is the minimum accuracy of the GPS position in meters, if <= 0.0 it requires RTK_FIXED
    if (min_position_accuracy <= 0.0) {
        if (!is_fixed) {
            ROS_WARN_STREAM_THROTTLE(60, "odom_converter: GPS not RTK fixed");
            return;
        }
    } else {
        // damping factor should take care of gps missing for too long
        // if (msg->position_accuracy > min_position_accuracy || (((ros::Time::now() - last_gps_fixed_time).toSec() > 60.0) && !is_fixed)) {
        if (msg->position_accuracy > min_position_accuracy) {
            ROS_WARN_STREAM_THROTTLE(60, "odom_converter: GPS position accuracy is too low: " << msg->position_accuracy << "; senconds since last fixed:" << (ros::Time::now() - last_gps_fixed_time).toSec());
            return;
        }
    }

    // compute a damping factor based on the time since the last GPS fix (with a margin so fully fixed)
    double damping = 0.0;
    if (!last_gps_fully_fixed_time.isZero()) {
        damping = float_damping_factor * (ros::Time::now() - last_gps_fully_fixed_time).toSec();
    } else {
        damping = float_damping_factor * 60.0; // if the GPS is not fixed, use a high damping factor
    }

    if (!is_fixed) {
        // ROS_INFO_STREAM_THROTTLE(1, "odom_converter: GPS is float, last fixed " << (ros::Time::now() - last_gps_fixed_time).toSec() << " seconds ago, fully fixed " << (ros::Time::now() - last_gps_fully_fixed_time).toSec() << " seconds ago, damping: " << damping);
    }
    tf2::Quaternion q;
    // compute heading from motion vector
    double heading = msg->msg->motion_heading;
    // double heading = std::atan2(msg->motion_vector.y, msg->motion_vector.x);
    // if reversing, flip the heading
    if (is_reversing) {
        // ROS_INFO_STREAM_THROTTLE(1, "odom_converter: reversing: " << vx);
        heading += M_PI;
        if (heading > 2*M_PI) {
            heading -= 2*M_PI;
        }
    }
    q.setRPY(0, 0, heading);
    double orientation_covariance = msg->orientation_accuracy * msg->orientation_accuracy;
    if (orientation_covariance <= 0.0) {
        // take pose covariance instead
        orientation_covariance = msg->pose.covariance[0];
    }
    if (orientation_covariance <= 0.0) {
        // put some default value
        orientation_covariance = 0.1;
    }

    // convert pose from "gps" frame to "map" frame
    geometry_msgs::PoseStamped pose_map;
    try {
        // get current yaw from map to base_link
        geometry_msgs::TransformStamped transform_stamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);

        // Convert quaternion to RPY
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        pose_map.pose = msg->pose.pose;
        pose_map.pose.position.x -= antenna_offset_x * cos(yaw);
        pose_map.pose.position.y -= antenna_offset_x * sin(yaw);
        pose_map.pose.position.z = 0;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("om_mower_logic map/base_link: %s", ex.what());
        return;
    }

    bool is_moving = (std::sqrt(std::pow(msg->motion_vector.x, 2)+std::pow(msg->motion_vector.y, 2)) >= orientation_min_speed);

    pose.header.stamp = msg->header.stamp;
    pose.header.seq++;
    pose.header.frame_id = "map";
    //pose.pose = msg->pose;
    pose.pose.pose = pose_map.pose;
    pose.pose.covariance = msg->pose.covariance;
    pose.pose.covariance[21] = 0.0;
    pose.pose.covariance[28] = 0.0;
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation = tf2::toMsg(q);
    if (is_fixed) {
        pose.pose.covariance[0] = pose.pose.covariance[0] * cov_factor_pos;
        pose.pose.covariance[7] = pose.pose.covariance[7] * cov_factor_pos;
        pose.pose.covariance[14] = pose.pose.covariance[14] * cov_factor_pos;
        pose.pose.covariance[35] = orientation_covariance * cov_factor_ori;
    } else {
        pose.pose.covariance[0] = pose.pose.covariance[0] * (cov_factor_float_pos + damping);
        pose.pose.covariance[7] = pose.pose.covariance[7] * (cov_factor_float_pos + damping);
        pose.pose.covariance[14] = pose.pose.covariance[14] * (cov_factor_float_pos + damping);
        pose.pose.covariance[35] = orientation_covariance * cov_factor_float_ori;
    }
    if (!is_moving) {
        pose.pose.covariance[35] = 1000.0;
    }

    if (!positioning_initialized) {
        positioning_initialized = true;
        ROS_INFO_STREAM("odom_converter: GPS positioning initialized to " << pose.pose.pose.position.x << ", " << pose.pose.pose.position.y);
        // orientation covariance is not valid in this case so we set the covariance to a high value (probably not needed because not moving ... but just to be sure)
        pose.pose.covariance[35] = 1000.0;
        // set the robot pose to the current GPS pose
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg_ptr = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(pose);
        setRobotPose(msg_ptr);
    }
    // Publish pose:
    // If GPS is fixed, always publish.
    // If GPS is NOT fixed, publish at most at 2 Hz (every 0.5s) to reduce noisy updates.
    if (is_fixed) {
        pose_pub.publish(pose);
    } else {
        ros::Time now = ros::Time::now();
        if (last_unfixed_pose_publish.isZero() || (now - last_unfixed_pose_publish).toSec() >= 0.5) {
            pose_pub.publish(pose);
            last_unfixed_pose_publish = now;
        } else {
            ROS_DEBUG_STREAM_THROTTLE(2, "odom_converter: Skipping unfixed pose publish to throttle rate (<2Hz)");
        }
    }

    // publish odometry only if the speed is relevant
    if(is_moving) {
        imu.header.stamp = msg->header.stamp;
        imu.header.seq++;
        imu.header.frame_id = "gps";
        imu.orientation = pose.pose.pose.orientation;
        imu.orientation_covariance[8] = pose.pose.covariance[35];

        imu_pub.publish(imu);
    }

    // manage kidnapped robot problem: GPS is fixed but the filter's covariance is too high
    if (is_fixed && last_filter_init < ros::Time::now() - ros::Duration(5.0) && last_good_odometry < ros::Time::now() - ros::Duration(10.0)) {
        last_filter_init = ros::Time::now();
        ROS_WARN_STREAM("odom_converter: GPS is fixed but the filter is out of sync. Resetting the filter");
        // orientation covariance is not valid in this case so we set the covariance to a high value (probably not needed because not moving ... but just to be sure)
        pose.pose.covariance[35] = 1000.0;
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg_ptr = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(pose);
        setRobotPose(msg_ptr);
    }

    // initialize AMCL with the GPS pose
    // if (!amcl_initialized && is_fixed) {
    //     amcl_initialized = true;
    //     ROS_INFO_STREAM("odom_converter: AMCL initialized with GPS pose " << pose.pose.pose.position.x << ", " << pose.pose.pose.position.y);
    //     amcl_initial_publisher.publish(pose);
    // }
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if(!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();
    if (dt <= 0.0) {
        ROS_WARN_STREAM("odom_converter: got wheel ticks with dt <= 0.0 (" << dt << ") - dropping measurement");
        return;
    }

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (1/wheel_ticks_per_m);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (1/wheel_ticks_per_m);
    
    if(msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if(msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }
    is_reversing = msg->wheel_direction_rl && msg->wheel_direction_rr;

    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    double computed_vx = d_ticks / dt;

    last_ticks = *msg;

    // limit wheel angular speed
    double angular_speed = (d_wheel_r - d_wheel_l)/(dt * 0.33);

    // consider max possible robot speed of 0.50 (TODO get it from parameters)
    if(abs(computed_vx) > 1.0) {
        ROS_WARN_STREAM("got vx > 1.0 (" << computed_vx << ") - dropping measurement");
        return;
    }
    vx = computed_vx;

    // publish odometry
    odometry.header.stamp = msg->stamp;
    odometry.header.seq++;
    odometry.header.frame_id = odometry_frame_id;
    odometry.child_frame_id = "base_link";
    
    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.angular.z = angular_speed;
    odometry.twist.covariance[0] = 0.05;
    odometry.twist.covariance[7] = 0.05;
    odometry.twist.covariance[14] = 1000;
    odometry.twist.covariance[21] = 1000;
    odometry.twist.covariance[28] = 1000;
    odometry.twist.covariance[35] = 0.02;

    odometry_pub.publish(odometry);
}

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {

    filtered_imu = *msg;
    
    // covariance factors: adjust the covariance values based on the configuration
    // sometimes the IMU doens't provide the covariance values, so we use the fixed values
    if (fixed_acceleration_covariance > 0.0) {
        filtered_imu.linear_acceleration_covariance[0] = fixed_acceleration_covariance;
        filtered_imu.linear_acceleration_covariance[4] = fixed_acceleration_covariance;
        filtered_imu.linear_acceleration_covariance[8] = 1000.0;
    } else {
        filtered_imu.linear_acceleration_covariance[0] = filtered_imu.linear_acceleration_covariance[0] * acceleration_covariance_factor;
        filtered_imu.linear_acceleration_covariance[4] = filtered_imu.linear_acceleration_covariance[4] * acceleration_covariance_factor;
        filtered_imu.linear_acceleration_covariance[8] = filtered_imu.linear_acceleration_covariance[8] * acceleration_covariance_factor;
    }
    if (fixed_yaw_covariance > 0.0) {
        filtered_imu.angular_velocity_covariance[0] = 1000.0;
        filtered_imu.angular_velocity_covariance[4] = 1000.0;
        filtered_imu.angular_velocity_covariance[8] = fixed_yaw_covariance;
    } else {
        filtered_imu.angular_velocity_covariance[0] = filtered_imu.angular_velocity_covariance[0] * yaw_covariance_factor;
        filtered_imu.angular_velocity_covariance[4] = filtered_imu.angular_velocity_covariance[4] * yaw_covariance_factor;
        filtered_imu.angular_velocity_covariance[8] = filtered_imu.angular_velocity_covariance[8] * yaw_covariance_factor;
    }

    filtered_imu_pub.publish(filtered_imu);
}

void odomReceived(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::recursive_mutex> lk{odom_mutex};

    last_odometry = *msg;

    xbot_msgs::AbsolutePose open_mower_pose;
    open_mower_pose.header = msg->header;
    open_mower_pose.pose = msg->pose;
    open_mower_pose.position_accuracy = std::sqrt(msg->pose.covariance[0] + msg->pose.covariance[7]);
    open_mower_pose.orientation_accuracy = std::sqrt(msg->pose.covariance[35]);
    open_mower_pose.flags = 0;

    if (last_odometry.pose.covariance[0] < max_covariance && last_odometry.pose.covariance[7] < max_covariance) {
        last_good_odometry = ros::Time::now();

        open_mower_pose.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
        open_mower_pose.orientation_valid = true;
    }
    // open_mower_pose_pub.publish(open_mower_pose);
}

void onAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    std::lock_guard<std::recursive_mutex> lk{odom_mutex};

    // publish the pose to AMCL
    geometry_msgs::PoseWithCovarianceStamped amcl_pose;
    amcl_pose.header = msg->header;
    amcl_pose.pose = msg->pose;
    amcl_pose.pose.covariance[0] *= amcl_covariance_factor;
    amcl_pose.pose.covariance[7] *= amcl_covariance_factor;
    amcl_pose.pose.covariance[14] *= amcl_covariance_factor;
    amcl_pose.pose.covariance[21] *= amcl_covariance_factor;
    amcl_pose.pose.covariance[28] *= amcl_covariance_factor;
    amcl_pose.pose.covariance[35] *= amcl_covariance_factor;

    amcl_filtered_publisher.publish(amcl_pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_converter");

    has_gyro = false;
    positioning_initialized = false;

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::Subscriber wheel_tick_sub = paramNh.subscribe("/mower/wheel_ticks", 10, onWheelTicks);
    ros::Subscriber gps_sub = paramNh.subscribe("/xbot_driver_gps/xb_pose", 10, onGPS);
    ros::Subscriber raw_imu_sub = paramNh.subscribe("/imu/data_raw", 10, onImu);
    ros::Subscriber odom_sub = paramNh.subscribe("/odometry_map/filtered", 0, odomReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber initial_pose_sub = paramNh.subscribe("/initialpose", 10, setRobotPose);
    ros::Subscriber amcl_pose_sub = paramNh.subscribe("/amcl_pose", 1, onAmclPose);
    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom", 10);
    imu_pub = paramNh.advertise<sensor_msgs::Imu>("orientation", 10);
    filtered_imu_pub = paramNh.advertise<sensor_msgs::Imu>("filtered_imu", 10);
    pose_pub = paramNh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
    open_mower_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("open_mower_pose", 10);
    amcl_initial_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_initial", 1, true);
    amcl_filtered_publisher = paramNh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, true);

    paramNh.param("cov_factor_pos", cov_factor_pos, 10.0);
    paramNh.param("cov_factor_ori", cov_factor_ori, 100.0);
    paramNh.param("cov_factor_float_pos", cov_factor_float_pos, 500.0);
    paramNh.param("cov_factor_float_ori", cov_factor_float_ori, 5000.0);
    paramNh.param("orientation_min_speed", orientation_min_speed, 0.01);
    paramNh.param("antenna_offset_x", antenna_offset_x, 0.0);
    paramNh.param("antenna_offset_y", antenna_offset_y, 0.0);
    paramNh.param("min_position_accuracy", min_position_accuracy, 0.0);
    paramNh.param("float_damping_factor", float_damping_factor, 10.0);
    paramNh.param("max_covariance", max_covariance, 0.5);
    paramNh.param("acceleration_covariance_factor", acceleration_covariance_factor, 100.0);
    paramNh.param("yaw_covariance_factor", yaw_covariance_factor, 100.0);
    paramNh.param("amcl_covariance_factor", amcl_covariance_factor, 10.0);
    paramNh.param("use_odom_frame", useOdomFrame, false);
    paramNh.param("fully_fixed_wait_time", fully_fixed_wait_time, 20.0);
    paramNh.param("fixed_yaw_covariance", fixed_yaw_covariance, 0.0001);
    paramNh.param("fixed_acceleration_covariance", fixed_acceleration_covariance, 0.0001);
    paramNh.param("wheel_ticks_per_m", wheel_ticks_per_m, 280.0);

    positioningClientMap = n.serviceClient<robot_localization::SetPose>(
        "odometry_map/set_pose");
    if (useOdomFrame) {
        positioningClientOdom = n.serviceClient<robot_localization::SetPose>(
            "odometry_odom/set_pose");
    }

    ros::ServiceServer gps_service = n.advertiseService("odom_converter/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("odom_converter/set_robot_pose", setPoseFromService);

    ros::spin();

    return 0;
}