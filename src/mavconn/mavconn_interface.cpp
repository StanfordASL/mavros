/**
 * @file mavconn_interface.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <set>
#include <ev++.h>
#include <mavros/mavconn_interface.h>
#include <mavros/utils.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/ros.h>

#include <mavros/mavconn_serial.h>
#include <mavros/mavconn_udp.h>
#include <mavros/mavconn_tcp.h>

namespace mavconn {

#if MAVLINK_CRC_EXTRA
const uint8_t MAVConnInterface::mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;
#endif
std::set<int> MAVConnInterface::allocated_channels;

static ev::default_loop default_loop;
static boost::thread default_loop_thd;

static void loop_spinner() {
	while (ros::ok()) {
		ROS_DEBUG_NAMED("mavconn", "EV: starting default loop");
		default_loop.run(0);
		ROS_DEBUG_NAMED("mavconn", "EV: default loop stopped");
	}
}

void MAVConnInterface::start_default_loop() {
	if (default_loop_thd.joinable())
		return;

	boost::thread t(loop_spinner);
	mavutils::set_thread_name(t, "ev_default_loop");
	default_loop_thd.swap(t);
}

MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id) :
	sys_id(system_id),
	comp_id(component_id)
{
	channel = new_channel();
	ROS_ASSERT_MSG(channel >= 0, "channel allocation failure");
}

int MAVConnInterface::new_channel() {
	int chan = 0;

	for (chan = 0; chan < MAVLINK_COMM_NUM_BUFFERS; chan++) {
		if (allocated_channels.count(chan) == 0) {
			ROS_DEBUG_NAMED("mavconn", "Allocate new channel: %d", chan);
			allocated_channels.insert(chan);
			return chan;
		}
	}

	ROS_ERROR_NAMED("mavconn", "channel overrun");
	return -1;
}

void MAVConnInterface::delete_channel(int chan) {
	ROS_DEBUG_NAMED("mavconn", "Freeing channel: %d", chan);
	allocated_channels.erase(allocated_channels.find(chan));
}

int MAVConnInterface::channes_available() {
	return MAVLINK_COMM_NUM_BUFFERS - allocated_channels.size();
}

/**
 * Parse host:port pairs
 */
static void url_parse_host(std::string host,
		std::string &host_out, int &port_out,
		const std::string def_host, const int def_port)
{
	std::string port;

	auto sep_it = std::find(host.begin(), host.end(), ':');
	if (sep_it == host.end()) {
		// host
		if (!host.empty()) {
			host_out = host;
			port_out = def_port;
		}
		else {
			host_out = def_host;
			port_out = def_port;
		}
		return;
	}

	if (sep_it == host.begin()) {
		// :port
		host_out = def_host;
	}
	else {
		// host:port
		host_out.assign(host.begin(), sep_it);
	}

	port.assign(sep_it + 1, host.end());
	port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
static void url_parse_query(std::string query, uint8_t &sysid, uint8_t &compid)
{
	const std::string ids_end("ids=");
	std::string sys, comp;

	if (query.empty())
		return;

	auto ids_it = std::search(query.begin(), query.end(),
			ids_end.begin(), ids_end.end());
	if (ids_it == query.end()) {
		ROS_WARN_NAMED("mavconn", "URL: unknown query arguments");
		return;
	}

	std::advance(ids_it, ids_end.length());
	auto comma_it = std::find(ids_it, query.end(), ',');
	if (comma_it == query.end()) {
		ROS_ERROR_NAMED("mavconn", "URL: no comma in ids= query");
		return;
	}

	sys.assign(ids_it, comma_it);
	comp.assign(comma_it + 1, query.end());

	sysid = std::stoi(sys);
	compid = std::stoi(comp);

	ROS_DEBUG_NAMED("mavconn", "URL: found system/component id = [%u, %u]",
			sysid, compid);
}

static boost::shared_ptr<MAVConnInterface> url_parse_serial(
		std::string path, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string file_path;
	int baudrate;

	// /dev/ttyACM0:57600
	url_parse_host(path, file_path, baudrate, "/dev/ttyACM0", 57600);
	url_parse_query(query, system_id, component_id);

	return boost::make_shared<MAVConnSerial>(system_id, component_id,
			file_path, baudrate);
}

static boost::shared_ptr<MAVConnInterface> url_parse_udp(
		std::string hosts, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string bind_pair, remote_pair;
	std::string bind_host, remote_host;
	int bind_port, remote_port;

	auto sep_it = std::find(hosts.begin(), hosts.end(), '@');
	if (sep_it == hosts.end()) {
		ROS_ERROR_NAMED("mavconn", "UDP URL should contain @!");
		throw DeviceError("url", "UDP separator not found");
	}

	bind_pair.assign(hosts.begin(), sep_it);
	remote_pair.assign(sep_it + 1, hosts.end());

	// udp://0.0.0.0:14555@:14550
	url_parse_host(bind_pair, bind_host, bind_port, "0.0.0.0", 14555);
	url_parse_host(remote_pair, remote_host, remote_port, "", 14550);
	url_parse_query(query, system_id, component_id);

	return boost::make_shared<MAVConnUDP>(system_id, component_id,
			bind_host, bind_port,
			remote_host, remote_port);
}

static boost::shared_ptr<MAVConnInterface> url_parse_tcp_client(
		std::string host, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string server_host;
	int server_port;

	// tcp://localhost:5760
	url_parse_host(host, server_host, server_port, "localhost", 5760);
	url_parse_query(query, system_id, component_id);

	return boost::make_shared<MAVConnTCPClient>(system_id, component_id,
			server_host, server_port);
}

static boost::shared_ptr<MAVConnInterface> url_parse_tcp_server(
		std::string host, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string bind_host;
	int bind_port;

	// tcp-l://0.0.0.0:5760
	url_parse_host(host, bind_host, bind_port, "0.0.0.0", 5760);
	url_parse_query(query, system_id, component_id);

	return boost::make_shared<MAVConnTCPServer>(system_id, component_id,
			bind_host, bind_port);
}

boost::shared_ptr<MAVConnInterface> MAVConnInterface::open_url(std::string url,
		uint8_t system_id, uint8_t component_id) {

	/* Based on code found here:
	 * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
	 */

	const std::string proto_end("://");
	std::string proto;
	std::string host;
	std::string path;
	std::string query;

	auto proto_it = std::search(
			url.begin(), url.end(),
			proto_end.begin(), proto_end.end());
	if (proto_it == url.end()) {
		// looks like file path
		ROS_DEBUG_NAMED("mavconn", "URL: %s: looks like file path", url.c_str());
		return url_parse_serial(url, "", system_id, component_id);
	}

	// copy protocol
	proto.reserve(std::distance(url.begin(), proto_it));
	std::transform(url.begin(), proto_it,
			std::back_inserter(proto),
			std::ref(tolower));

	// copy host
	std::advance(proto_it, proto_end.length());
	auto path_it = std::find(proto_it, url.end(), '/');
	std::transform(proto_it, path_it,
			std::back_inserter(host),
			std::ref(tolower));

	// copy path, and query if exists
	auto query_it = std::find(path_it, url.end(), '?');
	path.assign(path_it, query_it);
	if (query_it != url.end())
		++query_it;
	query.assign(query_it, url.end());

	ROS_DEBUG_NAMED("mavconn", "URL: %s: proto: %s, host: %s, path: %s, query: %s",
			url.c_str(), proto.c_str(), host.c_str(), path.c_str(), query.c_str());

	if (proto == "udp")
		return url_parse_udp(host, query, system_id, component_id);
	else if (proto == "tcp")
		return url_parse_tcp_client(host, query, system_id, component_id);
	else if (proto == "tcp-l")
		return url_parse_tcp_server(host, query, system_id, component_id);
	else if (proto == "serial")
		return url_parse_serial(path, query, system_id, component_id);
	else
		throw DeviceError("url", "Unknown URL type");
}

}; // namespace mavconn
