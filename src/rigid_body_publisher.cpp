/*
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mocap_nokov/rigid_body_publisher.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

namespace mocap_nokov
{

namespace utilities
{
  geometry_msgs::PoseStamped getRosPose(RigidBody const& body)
  {
    geometry_msgs::PoseStamped poseStampedMsg;
    poseStampedMsg.pose.position.x = body.pose.position.x;
    poseStampedMsg.pose.position.y = body.pose.position.y;
    poseStampedMsg.pose.position.z = body.pose.position.z;

    poseStampedMsg.pose.orientation.x = body.pose.orientation.x;
    poseStampedMsg.pose.orientation.y = body.pose.orientation.y;
    poseStampedMsg.pose.orientation.z = body.pose.orientation.z;
    poseStampedMsg.pose.orientation.w = body.pose.orientation.w;
    
    return poseStampedMsg;
  }  

  nav_msgs::Odometry getRosOdom(RigidBody const& body)
  {
  nav_msgs::Odometry OdometryMsg;

  OdometryMsg.pose.pose.position.x = body.pose.position.x;
  OdometryMsg.pose.pose.position.y = body.pose.position.y;
  OdometryMsg.pose.pose.position.z = body.pose.position.z;

  OdometryMsg.pose.pose.orientation.x = body.pose.orientation.x;
  OdometryMsg.pose.pose.orientation.y = body.pose.orientation.y;
  OdometryMsg.pose.pose.orientation.z = body.pose.orientation.z;
  OdometryMsg.pose.pose.orientation.w = body.pose.orientation.w;

  return OdometryMsg;
  } 
}

RigidBodyPublisher::RigidBodyPublisher(ros::NodeHandle &nh, 
  Version const& sdkVersion,
  PublisherConfiguration const& config) :
    config(config)
{
  if (config.publishPose)
    posePublisher = nh.advertise<geometry_msgs::PoseStamped>(config.poseTopicName, 1000);

  if (config.publishPose2d)
    pose2dPublisher = nh.advertise<geometry_msgs::Pose2D>(config.pose2dTopicName, 1000);

  if (config.publishOdom)
    odomPublisher = nh.advertise<nav_msgs::Odometry>(config.odomTopicName, 1000);
}

RigidBodyPublisher::~RigidBodyPublisher()
{
}

void RigidBodyPublisher::publish(ros::Time const& time, RigidBody const& body)
{
  // don't do anything if no new data was provided
  if (!body.hasValidData())
  {
    return;
  }

  // NaN?
  if (body.pose.position.x != body.pose.position.x)
  {
    return;
  }

  geometry_msgs::PoseStamped pose = utilities::getRosPose(body);
  nav_msgs::Odometry odom =  utilities::getRosOdom(body);

  pose.header.stamp = time;
  odom.header.stamp = time;

  if (config.publishPose)
  {
    ////pose.header.frame_id = config.parentFrameId;
    //pose.header.frame_id = std::to_string(body.iFrame);
    pose.header.frame_id = "world";//shusei
    posePublisher.publish(pose);
  }

  tf::Quaternion q(pose.pose.orientation.x,
                   pose.pose.orientation.y,
                   pose.pose.orientation.z,
                   pose.pose.orientation.w);

  if (config.publishOdom)
  {
    ////odom.header.frame_id = config.parentFrameId;
    //odom.header.frame_id = std::to_string(body.iFrame);
    odom.header.frame_id = "world";//shusei
    odom.child_frame_id = config.childFrameId;
    odomPublisher.publish(odom);
  }
  //if (config.publishOdom)//komento out
  //{
  //  odom.header.frame_id = config.parentFrameId;
  //  odom.child_frame_id = config.childFrameId;
  //  odomPublisher.publish(odom);
  //}
  // publish 2D pose
  if (config.publishPose2d)
  {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = pose.pose.position.x;
    pose2d.y = pose.pose.position.y;
    pose2d.theta = tf::getYaw(q);
    pose2dPublisher.publish(pose2d);
  }

  if (config.publishTf)
  {
    // publish transform
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.pose.position.x,
                                    pose.pose.position.y,
                                    pose.pose.position.z));

    transform.setRotation(q);
    //tfPublisher.sendTransform(tf::StampedTransform(transform,
    //                          time,
    //                          std::to_string(body.iFrame),
    //                          config.childFrameId));
    tfPublisher.sendTransform(tf::StampedTransform(transform,
                              time,
                              "world",
                              config.childFrameId));//shusei
  }
}

RigidBodyPublishDispatcher::RigidBodyPublishDispatcher(
  ros::NodeHandle &node, 
  Version const& sdkVersion,
  PublisherConfigurations const& configs)
{
  for (auto const& config : configs)
  {
    rigidBodyPublisherMap[config.rigidBodyId] = 
      RigidBodyPublisherPtr(new RigidBodyPublisher(node, sdkVersion, config));
  }
}

void RigidBodyPublishDispatcher::publish(
  ros::Time const& time, 
  std::vector<RigidBody> const& rigidBodies
  )
{
  for (auto const& rigidBody : rigidBodies)
  {
    auto const& iter = rigidBodyPublisherMap.find(rigidBody.bodyId);

    if (iter != rigidBodyPublisherMap.end())
    {
      (*iter->second).publish(time, rigidBody);
    }
  }
}


} // namespace
