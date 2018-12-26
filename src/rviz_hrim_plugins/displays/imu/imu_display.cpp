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

#include "rviz_hrim_plugins/displays/imu/imu_display.hpp"
#include "rviz_hrim_plugins/displays/imu/imu_visual.hpp"

#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <tf2_ros/transform_listener.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/frame_manager_iface.hpp"

namespace rviz_hrim_plugins
{

namespace displays
{

ImuDisplay::ImuDisplay()
{
  color_property_ = new rviz_common::properties::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz_common::properties::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  history_length_property_ = new rviz_common::properties::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
  history_length_property_->setMin( 1 );
  history_length_property_->setMax( 5 );
}

void ImuDisplay::onInitialize()
{
  RTDClass::onInitialize();
  updateHistoryLength();
}

ImuDisplay::~ImuDisplay()
{
}

// Clear the visuals by deleting their objects.
void ImuDisplay::reset()
{
  RTDClass::reset();
  visuals_.clear();
}

// Set the current color and alpha values for each visual.
void ImuDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (auto visual : visuals_)
  {
    visual->setColor( color.r, color.g, color.b, alpha );
  }
}

// Set the number of past visuals to show.
void ImuDisplay::updateHistoryLength()
{
  visuals_.resize(history_length_property_->getInt());
}

// Handle an incoming message.
void ImuDisplay::processMessage( hrim_sensor_imu_msgs::msg::IMU::ConstSharedPtr msg )
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  position, orientation ))
  {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id.c_str() << "' to frame '" << qPrintable(fixed_frame_) << "'");
    return;
  }

  std::shared_ptr<ImuVisual> visual;

  if( visuals_.size() >= 5 )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new ImuVisual( context_->getSceneManager(), scene_node_ ));
  }

  // Set or update the contents of the chosen visual.
  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor( color.r, color.g, color.b, alpha );

  visuals_.push_back(visual);
}

} // namespace displays

} // namespace rviz_hrim_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_hrim_plugins::displays::ImuDisplay, rviz_common::Display)
