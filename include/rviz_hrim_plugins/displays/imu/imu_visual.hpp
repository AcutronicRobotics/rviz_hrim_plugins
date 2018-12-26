/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_HRIM_PLUGINS__DISPLAYS__IMU__IMU_VISUAL_HPP_
#define RVIZ_HRIM_PLUGINS__DISPLAYS__IMU__IMU_VISUAL_HPP_

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "hrim_sensor_imu_msgs/msg/imu.hpp"

namespace Ogre
{
class SceneNode;
class SceneManager;
class Vector3;
class Quaternion;
}

namespace rviz_rendering
{
class Arrow;
}

namespace rviz_hrim_plugins
{

namespace displays
{

class ImuVisual
{
public:
  ImuVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
  virtual ~ImuVisual();

  void setMessage( hrim_sensor_imu_msgs::msg::IMU::ConstSharedPtr msg );

  void setFramePosition( Ogre::Vector3& position );

  void setFrameOrientation( Ogre::Quaternion& orientation );

  void setColor( float r, float g, float b, float a );

private:
  std::shared_ptr<rviz_rendering::Arrow> acceleration_arrow_;

  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;
};

} // namespace displays
} // namespace rviz_hrim_plugin

#endif // RVIZ_HRIM_PLUGINS__DISPLAYS__IMU__IMU_VISUAL_HPP_
