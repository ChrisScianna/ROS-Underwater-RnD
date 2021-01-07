/*  QinetiQ North America (QNA)
 *  350 Second Avenue
 *  Waltham, MA 02451
 *
 *  Proprietary Licensed Information.
 *  Not to be duplicated, used, or disclosed,
 *  except under terms of the license.
 *
 *  Copyright © 2018 QinetiQ North America  All rights reserved.
 */

/* Overview:
 * This node has provides navigation information to the system. It has
 * two functions. First, it "consumes" UDP data from the ixBlue C3 inerial
 * navigation system (INS). Once the UDP packet data is decoded, the information
 * is published to the other ROS nodes for their handling.
 */

/* To-Do:
 * - hook in publishing code.
 * - add "normal" and "error" logging.
 * - build unit test harness, likely using python & scapy to send simulated INS UDP packets.
 * - unit test on target with simulation.
 * - unit test on target with C3 INS unit.
 * - unit test with all other ROS nodes.
 */

#include <arpa/inet.h>
#include <assert.h>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <errno.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdint.h>
#include <sstream>
#include <math.h>

#include "c3_protocol.h"
#include "ixblue_c3_ins/C3Ins.h"

namespace ixblue_c3_ins
{

#define NODE_NAME "ixblue_c3_ins"
#define TOPIC_NAME "C3Ins"
#define NODE_VERSION "0.0.0"
#define SERVER_READ_BUF_SIZE 1024
#define DEFAULT_IPADDR "192.168.36.112"
#define DEFAULT_PORT 2255

class C3InsNode {
private:
    // listening socket support to "consume" UDP data from the C3 INS unit
    int fd;
    std::string iface_addr;
    int listen_port;
    int openUdpSock(const char *iface, unsigned short port);
    void serverProc(); // handles select in 2nd thread
    boost::shared_ptr<boost::thread> s_thread;

    /* ROS hooks. We need to publish the navigation info we recieve */
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle;
	ros::Publisher c3_ins_pub;
    ixblue_c3_ins::C3Ins m_c3_ins_data;
    void publish_data(c3_protocol::nav_long& obj);

public:
    C3InsNode(ros::NodeHandle node_handle);
    void start();
    void stop();
};

void C3InsNode::publish_data(c3_protocol::nav_long& obj)
{    
    c3_protocol::nav_long::nav_long_data_t nl_data;
    nl_data = obj.get_data();

    m_c3_ins_data.nl_header = nl_data.header;
    m_c3_ins_data.user_status = nl_data.user_status;
    m_c3_ins_data.algo_status_0 = nl_data.algo_status[0];
    m_c3_ins_data.algo_status_1 = nl_data.algo_status[1];
    m_c3_ins_data.heading = nl_data.heading;
    m_c3_ins_data.roll = nl_data.roll;
    m_c3_ins_data.pitch = nl_data.pitch;
    m_c3_ins_data.north_speed = nl_data.north_speed;
    m_c3_ins_data.east_speed = nl_data.east_speed;
    m_c3_ins_data.vertical_speed = nl_data.vertical_speed;
    m_c3_ins_data.latitude = nl_data.latitude*8.382E-8f;
    m_c3_ins_data.longitude = nl_data.longitude*8.382E-8f;
    //ROS_INFO("%s: publishing longitude %f %f %f", NODE_NAME,(float)nl_data.longitude,nl_data.longitude*8.382E-8f,  m_c3_ins_data.longitude);
    m_c3_ins_data.altitude = nl_data.altitude;
    m_c3_ins_data.timestamp = nl_data.timestamp;
    m_c3_ins_data.heading_err_sd = nl_data.heading_err_sd;
    m_c3_ins_data.roll_err_sd = nl_data.roll_err_sd;
    m_c3_ins_data.pitch_err_sd = nl_data.pitch_err_sd;
    m_c3_ins_data.north_speed_err_sd = nl_data.north_speed_err_sd;
    m_c3_ins_data.east_speed_err_sd = nl_data.east_speed_err_sd;
    m_c3_ins_data.vertical_speed_err_sd = nl_data.vertical_speed_err_sd;
    m_c3_ins_data.latitude_err_sd = nl_data.latitude_err_sd;
    m_c3_ins_data.longitude_err_sd = nl_data.longitude_err_sd;
    m_c3_ins_data.altitude_err_sd = nl_data.altitude_err_sd;
    
    m_c3_ins_data.header.frame_id = NODE_NAME;
    m_c3_ins_data.header.stamp = ros::Time::now();

    c3_ins_pub.publish(m_c3_ins_data);
    //ROS_INFO("%s: published UDP data", NODE_NAME);
}

int C3InsNode::openUdpSock(const char *iface, unsigned short port)
{
	int fd = -1;
	struct sockaddr_in saddr;
	
	if (0 > (fd = socket(AF_INET, SOCK_DGRAM, 0))) {
		ROS_ERROR("Failed to open socket - %s", strerror(errno));
		return -1;
	}
    
	memset(&saddr, 0, sizeof(saddr));
	saddr.sin_family = AF_INET;
	saddr.sin_addr.s_addr = inet_addr(iface);
	saddr.sin_port = htons(port);
    
    /* UDP socket, we only need to bind, not listen, connect or accept */
	if (0 > bind(fd, (struct sockaddr *)&saddr, sizeof(saddr))) {
		ROS_ERROR("bind() failed - %s", strerror(errno));
		close(fd);
		return -1;
	}

	if (0 > setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &saddr, sizeof(saddr))) {
		ROS_ERROR("setsockopt(SO_REUSEADDR) failed - %s", strerror(errno));
		close(fd);
		return -1;
	}
	
	return fd;
}

C3InsNode::C3InsNode(ros::NodeHandle node_handle) :
    nh(node_handle)
{
    ros::NodeHandle c3_ins_node_handle(node_handle,  NODE_NAME);

    fd = -1; // poison

    /* get operating parameters */
    if (!nh.getParam("/ixblue_c3_ins_node/iface_addr", iface_addr)) {
		ROS_INFO("%s: no configured iface_addr. Using default [%s]",
                 NODE_NAME, DEFAULT_IPADDR);
        nh.param("iface_addr", iface_addr, std::string(DEFAULT_IPADDR));
    }
    if (!nh.getParam("/ixblue_c3_ins_node/listen_port", listen_port)) {
		ROS_INFO("%s: no configured UDP listening port. Using default [%d]",
                 NODE_NAME, DEFAULT_PORT);
        nh.param("listen_port", listen_port, DEFAULT_PORT);
    }
    // FIX. should also validate the IP address parameter
    if (0 > listen_port) {
		ROS_ERROR("%s: invalid UDP listener port. Exiting", NODE_NAME);
		exit(-1);
    }
    c3_ins_pub = c3_ins_node_handle.advertise<ixblue_c3_ins::C3Ins>(TOPIC_NAME, 0);
}

void C3InsNode::start()
{
    fd = openUdpSock(iface_addr.c_str(), (unsigned short)listen_port);
    if (-1 == fd) {
        /* additional error handling needed? */
        ROS_ERROR("open upd socket failed");
        return;
    }
    ROS_INFO("C3InsNode: Listening for UDP connections on %s:%d", iface_addr.c_str(), listen_port);
    s_thread = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&C3InsNode::serverProc, this)));
    ROS_INFO("C3InsNode: initialized.");
}

void C3InsNode::stop() 
{
    assert(s_thread);
    s_thread->join();
    /* close listening UDP socket and poison */
    close(fd);
    fd  = -1;

    /* We have no subscribers to stop */
}

/* This runs in its own thread */
void C3InsNode::serverProc() {
    while (ros::ok()) {
       ssize_t nbytes;
       int rc;
       fd_set fds;
       char buf[SERVER_READ_BUF_SIZE];

       assert(fd >= 0);

       FD_ZERO(&fds);
       FD_SET(fd, &fds);
       
       /* No timeout necessary. This thread can wait forever for data... */
       rc = select(fd + 1, &fds, NULL, NULL, NULL);
       if (-1 == rc) {
           perror("select()");
           ROS_ERROR("select failed");
       } else if (0 == rc) {
           printf("Timeout\n");
           ROS_ERROR("UDP select timed out");
           continue;
       }  else {
           /* received msg  */
           nbytes = read(fd, buf, sizeof(buf));
           //ROS_INFO("%s: server RX %ld bytes", NODE_NAME, nbytes);
           if (0 >= nbytes) {
               continue;
           }
           /* Hand off the message for validation and parsing.
            * The message rate should be low enough that we can
            * do so in this same thread. If that turns out not
            * to be the case, then we'll need fork a new thread.
            * "NAVIGATION LONG' message should be 90 bytes.
            */
           c3_protocol::nav_long nav(buf, nbytes);
           if (0 == nav.get_parse_success()) {
               //ROS_INFO("%s: publishing RX UDP data", NODE_NAME);
               publish_data(nav);
           }      else {
               ROS_INFO("%s: dropping RX UDP data", NODE_NAME);
           }
           // Handle any call backs; likely none since no subscribers.
           ros::spinOnce();
       }         
   }
}
} /* end namespace ixblue_c3_ins */

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  ROS_INFO("starting %s node version: [%s]", NODE_NAME, NODE_VERSION);

  ixblue_c3_ins::C3InsNode c3InsNode(nh);
  c3InsNode.start();
  /* Single threaded. spin() handles all callbacks.
   * Might need one consumer thread and one publisher thread.
   * And, as we have no subscribers, could remove ros::spin().
   */
  ros::spin();

  c3InsNode.stop();

  return(0);
}

