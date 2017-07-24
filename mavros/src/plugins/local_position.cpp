/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Local position plugin.
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class LocalPositionPlugin : public plugin::PluginBase {
public:
	LocalPositionPlugin() : PluginBase(),
		lp_nh("~local_position"),
		tf_send(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// header frame_id.
		// default to map (world-fixed,ENU as per REP-105).
		lp_nh.param<std::string>("frame_id", frame_id, "map");
		// Important tf subsection
		// Report the transform from world to base_link here.
		lp_nh.param("tf/send", tf_send, true);
		lp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		lp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");
		// Debug tf info
		// broadcast the following transform: (can be expanded to more if desired)
		// NED -> aircraft
		lp_nh.param("tf/send_fcu",tf_send_fcu,false);

		local_position = lp_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
		local_velocity = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);
		local_odom = lp_nh.advertise<nav_msgs::Odometry>("odom",10);
	}

	Subscriptions get_subscriptions() {
		return {
		       make_handler(&LocalPositionPlugin::handle_local_position_ned)
		};
	}

private:
	ros::NodeHandle lp_nh;

	ros::Publisher local_position;
	ros::Publisher local_velocity;
	ros::Publisher local_odom;

	std::string frame_id;		//!< frame for Pose
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF
	bool tf_send;
	bool tf_send_fcu;	//!< report NED->aircraft in tf tree

	void handle_local_position_ned(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED &pos_ned)
	{
		//--------------- Transform FCU position and Velocity Data ---------------//
		auto ned_position = Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z);
		auto ned_velocity = Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz);

		//--------------- Get Odom Information ---------------//
		auto ned_orientation_msg = m_uas->get_attitude_orientation();
		auto angular_msg = m_uas->get_attitude_angular_velocity();
		Eigen::Quaterniond ned_orientation;
		tf::quaternionMsgToEigen(ned_orientation_msg, ned_orientation);

		//--------------- Generate Message Pointers ---------------//
		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		auto twist = boost::make_shared<geometry_msgs::TwistStamped>();

		pose->header = m_uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
		twist->header = pose->header;

		tf::pointEigenToMsg(ned_position, pose->pose.position);
		pose->pose.orientation = ned_orientation_msg;

		tf::vectorEigenToMsg(ned_velocity, twist->twist.linear);
		twist->twist.angular = angular_msg;

		//--------------- Publish Data ---------------//
		local_position.publish(pose);
		local_velocity.publish(twist);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::LocalPositionPlugin, mavros::plugin::PluginBase)
