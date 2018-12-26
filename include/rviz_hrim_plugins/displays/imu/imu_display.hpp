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

#ifndef RVIZ_HRIM_PLUGINS__DISPLAYS__IMU__IMU_DISPLAY_HPP_
#define RVIZ_HRIM_PLUGINS__DISPLAYS__IMU__IMU_DISPLAY_HPP_

#include <memory>

#ifndef Q_MOC_RUN

#include "rviz_common/ros_topic_display.hpp"
#include "hrim_sensor_imu_msgs/msg/imu.hpp"
#include "hrim_geometry_msgs/msg/vector3.hpp"

#include "rviz_hrim_plugins/visibility_control.hpp"
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{

namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

}

namespace rviz_hrim_plugins
{

namespace displays
{

class ImuVisual;

class RVIZ_HRIM_PLUGINS_PUBLIC ImuDisplay: public rviz_common::RosTopicDisplay<hrim_sensor_imu_msgs::msg::IMU>
{
Q_OBJECT
public:

  ImuDisplay();
  ~ImuDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

private:
  void processMessage( const hrim_sensor_imu_msgs::msg::IMU::ConstSharedPtr msg ) override;

  std::deque<std::shared_ptr<ImuVisual>> visuals_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::IntProperty * history_length_property_;
};

} // namespace displays

} // namespace rviz_hrim_plugin_

#endif // RVIZ_HRIM_PLUGINS__DISPLAYS__IMU__IMU_DISPLAY_HPP_
