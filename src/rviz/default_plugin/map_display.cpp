/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <boost/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/custom_parameter_indices.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "map_display.h"

namespace rviz
{

MapDisplay::MapDisplay()
  : Display()
  , manual_object_( NULL )
  , loaded_( false )
  , resolution_( 0.0f )
  , width_( 0 )
  , height_( 0 )
{
  connect(this, SIGNAL( mapUpdated() ), this, SLOT( showMap() ));
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<nav_msgs::OccupancyGrid>() ),
                                          "nav_msgs::OccupancyGrid topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  alpha_property_ = new FloatProperty( "Alpha", 0.7,
                                       "Amount of transparency to apply to the map.",
                                       this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  color_scheme_property_ = new EnumProperty( "Color Scheme", "map", "How to color the occupancy values.",
                                             this, SLOT( updatePalette() ));
  // Option values here must correspond to indices in palette_textures_ array in onInitialize() below.
  color_scheme_property_->addOption( "map", 0 );
  color_scheme_property_->addOption( "costmap", 1 );
  color_scheme_property_->addOption( "jet (101 levels)", 2 );
  color_scheme_property_->addOption( "jet (256 levels)", 3 );
  color_scheme_property_->addOption( "hot (101 levels)", 4 );
  color_scheme_property_->addOption( "hot (256 levels)", 5 );
  
  draw_under_property_ = new Property( "Draw Behind", false,
                                       "Rendering option, controls whether or not the map is always"
                                       " drawn behind everything else.",
                                       this, SLOT( updateDrawUnder() ));

  resolution_property_ = new FloatProperty( "Resolution", 0,
                                            "Resolution of the map. (not editable)", this );
  resolution_property_->setReadOnly( true );

  width_property_ = new IntProperty( "Width", 0,
                                     "Width of the map, in meters. (not editable)", this );
  width_property_->setReadOnly( true );
  
  height_property_ = new IntProperty( "Height", 0,
                                      "Height of the map, in meters. (not editable)", this );
  height_property_->setReadOnly( true );

  position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO,
                                           "Position of the bottom left corner of the map, in meters. (not editable)",
                                           this );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY,
                                                  "Orientation of the map. (not editable)",
                                                  this );
  orientation_property_->setReadOnly( true );
}

MapDisplay::~MapDisplay()
{
  unsubscribe();
  clear();
}

unsigned char* makeMapPalette()
{
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;
  // Standard gray map palette values
  for( int i = 0; i <= 100; i++ )
  {
    unsigned char v = 255 - (255 * i) / 100;
    *palette_ptr++ = v; // red
    *palette_ptr++ = v; // green
    *palette_ptr++ = v; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal positive values in green
  for( int i = 101; i <= 127; i++ )
  {
    *palette_ptr++ = 0; // red
    *palette_ptr++ = 255; // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal negative (char) values in shades of red/yellow
  for( int i = 128; i <= 254; i++ )
  {
    *palette_ptr++ = 255; // red
    *palette_ptr++ = (255*(i-128))/(254-128); // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // legal -1 value is tasteful blueish greenish grayish color
  *palette_ptr++ = 0x70; // red
  *palette_ptr++ = 0x89; // green
  *palette_ptr++ = 0x86; // blue
  *palette_ptr++ = 255; // alpha

  return palette;
}

unsigned char* makeCostmapPalette()
{
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;

  // zero values have alpha=0
  *palette_ptr++ = 0; // red
  *palette_ptr++ = 0; // green
  *palette_ptr++ = 0; // blue
  *palette_ptr++ = 0; // alpha

  // Blue to red spectrum for most normal cost values
  for( int i = 1; i <= 98; i++ )
  {
    unsigned char v = (255 * i) / 100;
    *palette_ptr++ = v; // red
    *palette_ptr++ = 0; // green
    *palette_ptr++ = 255 - v; // blue
    *palette_ptr++ = 255; // alpha
  }
  // inscribed obstacle values (99) in cyan
  *palette_ptr++ = 0; // red
  *palette_ptr++ = 255; // green
  *palette_ptr++ = 255; // blue
  *palette_ptr++ = 255; // alpha
  // lethal obstacle values (100) in purple
  *palette_ptr++ = 255; // red
  *palette_ptr++ = 255; // green
  *palette_ptr++ = 0; // blue
  *palette_ptr++ = 255; // alpha
  // illegal positive values in green
  for( int i = 101; i <= 127; i++ )
  {
    *palette_ptr++ = 0; // red
    *palette_ptr++ = 255; // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal negative (char) values in shades of red/yellow
  for( int i = 128; i <= 254; i++ )
  {
    *palette_ptr++ = 255; // red
    *palette_ptr++ = (255*(i-128))/(254-128); // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // legal -1 value is tasteful blueish greenish grayish color
  *palette_ptr++ = 0x70; // red
  *palette_ptr++ = 0x89; // green
  *palette_ptr++ = 0x86; // blue
  *palette_ptr++ = 255; // alpha

  return palette;
}

unsigned char* makeJetPalette256() {
  static unsigned char palette[] = {
    129, 255, 125, 255, 133, 255, 121, 255, 137, 255, 117, 255, 141, 255, 113, 255, 145, 255, 109, 255, 149, 255, 105, 255, 153, 255, 101, 255, 157, 255, 97, 255, 161, 255, 93, 255, 165, 255, 89, 255, 169, 255, 85, 255, 173, 255, 81, 255, 177, 255, 77, 255, 181, 255, 73, 255, 185, 255, 69, 255, 189, 255, 65, 255, 193, 255, 61, 255, 197, 255, 57, 255, 201, 255, 53, 255, 205, 255, 49, 255, 209, 255, 45, 255, 213, 255, 41, 255, 217, 255, 37, 255, 221, 255, 33, 255, 225, 255, 29, 255, 229, 255, 25, 255, 233, 255, 21, 255, 237, 255, 17, 255, 241, 255, 13, 255, 245, 255, 9, 255, 249, 255, 5, 255, 253, 255, 1, 255, 255, 252, 0, 255, 255, 248, 0, 255, 255, 244, 0, 255, 255, 240, 0, 255, 255, 236, 0, 255, 255, 232, 0, 255, 255, 228, 0, 255, 255, 224, 0, 255, 255, 220, 0, 255, 255, 216, 0, 255, 255, 212, 0, 255, 255, 208, 0, 255, 255, 204, 0, 255, 255, 200, 0, 255, 255, 196, 0, 255, 255, 192, 0, 255, 255, 188, 0, 255, 255, 184, 0, 255, 255, 180, 0, 255, 255, 176, 0, 255, 255, 172, 0, 255, 255, 168, 0, 255, 255, 164, 0, 255, 255, 160, 0, 255, 255, 156, 0, 255, 255, 152, 0, 255, 255, 148, 0, 255, 255, 144, 0, 255, 255, 140, 0, 255, 255, 136, 0, 255, 255, 132, 0, 255, 255, 128, 0, 255, 255, 124, 0, 255, 255, 120, 0, 255, 255, 116, 0, 255, 255, 112, 0, 255, 255, 108, 0, 255, 255, 104, 0, 255, 255, 100, 0, 255, 255, 96, 0, 255, 255, 92, 0, 255, 255, 88, 0, 255, 255, 84, 0, 255, 255, 80, 0, 255, 255, 76, 0, 255, 255, 72, 0, 255, 255, 68, 0, 255, 255, 64, 0, 255, 255, 60, 0, 255, 255, 56, 0, 255, 255, 52, 0, 255, 255, 48, 0, 255, 255, 44, 0, 255, 255, 40, 0, 255, 255, 36, 0, 255, 255, 32, 0, 255, 255, 28, 0, 255, 255, 24, 0, 255, 255, 20, 0, 255, 255, 16, 0, 255, 255, 12, 0, 255, 255, 8, 0, 255, 255, 4, 0, 255, 255, 0, 0, 255, 251, 0, 0, 255, 247, 0, 0, 255, 243, 0, 0, 255, 239, 0, 0, 255, 235, 0, 0, 255, 231, 0, 0, 255, 227, 0, 0, 255, 223, 0, 0, 255, 219, 0, 0, 255, 215, 0, 0, 255, 211, 0, 0, 255, 207, 0, 0, 255, 203, 0, 0, 255, 199, 0, 0, 255, 195, 0, 0, 255, 191, 0, 0, 255, 187, 0, 0, 255, 183, 0, 0, 255, 179, 0, 0, 255, 175, 0, 0, 255, 171, 0, 0, 255, 167, 0, 0, 255, 163, 0, 0, 255, 159, 0, 0, 255, 155, 0, 0, 255, 151, 0, 0, 255, 147, 0, 0, 255, 143, 0, 0, 255, 139, 0, 0, 255, 135, 0, 0, 255, 131, 0, 0, 255, 127, 0, 0, 255, 0, 0, 127, 255, 0, 0, 131, 255, 0, 0, 135, 255, 0, 0, 139, 255, 0, 0, 143, 255, 0, 0, 147, 255, 0, 0, 151, 255, 0, 0, 155, 255, 0, 0, 159, 255, 0, 0, 163, 255, 0, 0, 167, 255, 0, 0, 171, 255, 0, 0, 175, 255, 0, 0, 179, 255, 0, 0, 183, 255, 0, 0, 187, 255, 0, 0, 191, 255, 0, 0, 195, 255, 0, 0, 199, 255, 0, 0, 203, 255, 0, 0, 207, 255, 0, 0, 211, 255, 0, 0, 215, 255, 0, 0, 219, 255, 0, 0, 223, 255, 0, 0, 227, 255, 0, 0, 231, 255, 0, 0, 235, 255, 0, 0, 239, 255, 0, 0, 243, 255, 0, 0, 247, 255, 0, 0, 251, 255, 0, 0, 255, 255, 0, 4, 255, 255, 0, 8, 255, 255, 0, 12, 255, 255, 0, 16, 255, 255, 0, 20, 255, 255, 0, 24, 255, 255, 0, 28, 255, 255, 0, 32, 255, 255, 0, 36, 255, 255, 0, 40, 255, 255, 0, 44, 255, 255, 0, 48, 255, 255, 0, 52, 255, 255, 0, 56, 255, 255, 0, 60, 255, 255, 0, 64, 255, 255, 0, 68, 255, 255, 0, 72, 255, 255, 0, 76, 255, 255, 0, 80, 255, 255, 0, 84, 255, 255, 0, 88, 255, 255, 0, 92, 255, 255, 0, 96, 255, 255, 0, 100, 255, 255, 0, 104, 255, 255, 0, 108, 255, 255, 0, 112, 255, 255, 0, 116, 255, 255, 0, 120, 255, 255, 0, 124, 255, 255, 0, 128, 255, 255, 0, 132, 255, 255, 0, 136, 255, 255, 0, 140, 255, 255, 0, 144, 255, 255, 0, 148, 255, 255, 0, 152, 255, 255, 0, 156, 255, 255, 0, 160, 255, 255, 0, 164, 255, 255, 0, 168, 255, 255, 0, 172, 255, 255, 0, 176, 255, 255, 0, 180, 255, 255, 0, 184, 255, 255, 0, 188, 255, 255, 0, 192, 255, 255, 0, 196, 255, 255, 0, 200, 255, 255, 0, 204, 255, 255, 0, 208, 255, 255, 0, 212, 255, 255, 0, 216, 255, 255, 0, 220, 255, 255, 0, 224, 255, 255, 0, 228, 255, 255, 0, 232, 255, 255, 0, 236, 255, 255, 0, 240, 255, 255, 0, 244, 255, 255, 0, 248, 255, 255, 0, 252, 255, 255, 1, 255, 253, 255, 5, 255, 249, 255, 9, 255, 245, 255, 13, 255, 241, 255, 17, 255, 237, 255, 21, 255, 233, 255, 25, 255, 229, 255, 29, 255, 225, 255, 33, 255, 221, 255, 37, 255, 217, 255, 41, 255, 213, 255, 45, 255, 209, 255, 49, 255, 205, 255, 53, 255, 201, 255, 57, 255, 197, 255, 61, 255, 193, 255, 65, 255, 189, 255, 69, 255, 185, 255, 73, 255, 181, 255, 77, 255, 177, 255, 81, 255, 173, 255, 85, 255, 169, 255, 89, 255, 165, 255, 93, 255, 161, 255, 97, 255, 157, 255, 101, 255, 153, 255, 105, 255, 149, 255, 109, 255, 145, 255, 113, 255, 141, 255, 117, 255, 137, 255, 121, 255, 133, 255, 125, 255, 129, 255
  };
  return palette;
}

unsigned char* makeJetPalette101() {
  static unsigned char palette[] = {
    0, 0, 127, 255, 0, 0, 135, 255, 0, 0, 147, 255, 0, 0, 155, 255, 0, 0, 167, 255, 0, 0, 175, 255, 0, 0, 187, 255, 0, 0, 195, 255, 0, 0, 207, 255, 0, 0, 215, 255, 0, 0, 227, 255, 0, 0, 239, 255, 0, 0, 247, 255, 0, 4, 255, 255, 0, 12, 255, 255, 0, 24, 255, 255, 0, 32, 255, 255, 0, 44, 255, 255, 0, 52, 255, 255, 0, 64, 255, 255, 0, 76, 255, 255, 0, 84, 255, 255, 0, 96, 255, 255, 0, 104, 255, 255, 0, 116, 255, 255, 0, 124, 255, 255, 0, 136, 255, 255, 0, 144, 255, 255, 0, 156, 255, 255, 0, 164, 255, 255, 0, 176, 255, 255, 0, 188, 255, 255, 0, 196, 255, 255, 0, 208, 255, 255, 0, 216, 255, 255, 0, 228, 255, 255, 0, 236, 255, 255, 0, 248, 255, 255, 1, 255, 253, 255, 13, 255, 241, 255, 25, 255, 229, 255, 33, 255, 221, 255, 45, 255, 209, 255, 53, 255, 201, 255, 65, 255, 189, 255, 73, 255, 181, 255, 85, 255, 169, 255, 93, 255, 161, 255, 105, 255, 149, 255, 113, 255, 141, 255, 125, 255, 129, 255, 137, 255, 117, 255, 145, 255, 109, 255, 157, 255, 97, 255, 165, 255, 89, 255, 177, 255, 77, 255, 185, 255, 69, 255, 197, 255, 57, 255, 205, 255, 49, 255, 217, 255, 37, 255, 229, 255, 25, 255, 237, 255, 17, 255, 249, 255, 5, 255, 255, 252, 0, 255, 255, 240, 0, 255, 255, 232, 0, 255, 255, 220, 0, 255, 255, 212, 0, 255, 255, 200, 0, 255, 255, 192, 0, 255, 255, 180, 0, 255, 255, 168, 0, 255, 255, 160, 0, 255, 255, 148, 0, 255, 255, 140, 0, 255, 255, 128, 0, 255, 255, 120, 0, 255, 255, 108, 0, 255, 255, 100, 0, 255, 255, 88, 0, 255, 255, 76, 0, 255, 255, 68, 0, 255, 255, 56, 0, 255, 255, 48, 0, 255, 255, 36, 0, 255, 255, 28, 0, 255, 255, 16, 0, 255, 255, 8, 0, 255, 251, 0, 0, 255, 243, 0, 0, 255, 231, 0, 0, 255, 219, 0, 0, 255, 211, 0, 0, 255, 199, 0, 0, 255, 191, 0, 0, 255, 179, 0, 0, 255, 171, 0, 0, 255, 159, 0, 0, 255, 151, 0, 0, 255, 139, 0, 0, 255, 127, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  return palette;
}

unsigned char* makeHotPalette256() {
  static unsigned char palette[] = {
    255, 65, 0, 255, 255, 67, 0, 255, 255, 69, 0, 255, 255, 72, 0, 255, 255, 74, 0, 255, 255, 77, 0, 255, 255, 80, 0, 255, 255, 82, 0, 255, 255, 84, 0, 255, 255, 87, 0, 255, 255, 89, 0, 255, 255, 92, 0, 255, 255, 95, 0, 255, 255, 97, 0, 255, 255, 99, 0, 255, 255, 102, 0, 255, 255, 104, 0, 255, 255, 107, 0, 255, 255, 109, 0, 255, 255, 112, 0, 255, 255, 114, 0, 255, 255, 117, 0, 255, 255, 119, 0, 255, 255, 122, 0, 255, 255, 125, 0, 255, 255, 127, 0, 255, 255, 130, 0, 255, 255, 132, 0, 255, 255, 135, 0, 255, 255, 137, 0, 255, 255, 140, 0, 255, 255, 142, 0, 255, 255, 145, 0, 255, 255, 147, 0, 255, 255, 149, 0, 255, 255, 152, 0, 255, 255, 154, 0, 255, 255, 157, 0, 255, 255, 160, 0, 255, 255, 162, 0, 255, 255, 164, 0, 255, 255, 167, 0, 255, 255, 169, 0, 255, 255, 172, 0, 255, 255, 175, 0, 255, 255, 177, 0, 255, 255, 180, 0, 255, 255, 182, 0, 255, 255, 185, 0, 255, 255, 187, 0, 255, 255, 190, 0, 255, 255, 192, 0, 255, 255, 195, 0, 255, 255, 197, 0, 255, 255, 200, 0, 255, 255, 202, 0, 255, 255, 205, 0, 255, 255, 207, 0, 255, 255, 210, 0, 255, 255, 212, 0, 255, 255, 215, 0, 255, 255, 217, 0, 255, 255, 220, 0, 255, 255, 222, 0, 255, 255, 225, 0, 255, 255, 227, 0, 255, 255, 229, 0, 255, 255, 232, 0, 255, 255, 234, 0, 255, 255, 237, 0, 255, 255, 240, 0, 255, 255, 242, 0, 255, 255, 244, 0, 255, 255, 247, 0, 255, 255, 249, 0, 255, 255, 251, 1, 255, 255, 252, 4, 255, 255, 253, 7, 255, 255, 254, 10, 255, 255, 255, 14, 255, 255, 255, 19, 255, 255, 255, 24, 255, 255, 255, 29, 255, 255, 255, 34, 255, 255, 255, 39, 255, 255, 255, 44, 255, 255, 255, 49, 255, 255, 255, 54, 255, 255, 255, 59, 255, 255, 255, 64, 255, 255, 255, 69, 255, 255, 255, 74, 255, 255, 255, 80, 255, 255, 255, 85, 255, 255, 255, 90, 255, 255, 255, 95, 255, 255, 255, 100, 255, 255, 255, 105, 255, 255, 255, 110, 255, 255, 255, 115, 255, 255, 255, 120, 255, 255, 255, 125, 255, 255, 255, 130, 255, 255, 255, 135, 255, 255, 255, 140, 255, 255, 255, 145, 255, 255, 255, 149, 255, 255, 255, 154, 255, 255, 255, 160, 255, 255, 255, 165, 255, 255, 255, 170, 255, 255, 255, 175, 255, 255, 255, 180, 255, 255, 255, 185, 255, 255, 255, 190, 255, 255, 255, 195, 255, 255, 255, 199, 255, 255, 255, 205, 255, 255, 255, 210, 255, 255, 255, 215, 255, 255, 255, 220, 255, 255, 255, 224, 255, 255, 255, 229, 255, 255, 255, 234, 255, 255, 255, 240, 255, 255, 255, 245, 255, 255, 255, 250, 255, 255, 255, 255, 255, 0, 0, 0, 255, 2, 0, 0, 255, 5, 0, 0, 255, 7, 0, 0, 255, 10, 0, 0, 255, 12, 0, 0, 255, 15, 0, 0, 255, 17, 0, 0, 255, 20, 0, 0, 255, 22, 0, 0, 255, 24, 0, 0, 255, 27, 0, 0, 255, 29, 0, 0, 255, 32, 0, 0, 255, 34, 0, 0, 255, 37, 0, 0, 255, 39, 0, 0, 255, 42, 0, 0, 255, 44, 0, 0, 255, 47, 0, 0, 255, 50, 0, 0, 255, 52, 0, 0, 255, 55, 0, 0, 255, 57, 0, 0, 255, 60, 0, 0, 255, 62, 0, 0, 255, 65, 0, 0, 255, 67, 0, 0, 255, 70, 0, 0, 255, 72, 0, 0, 255, 75, 0, 0, 255, 77, 0, 0, 255, 79, 0, 0, 255, 82, 0, 0, 255, 84, 0, 0, 255, 87, 0, 0, 255, 89, 0, 0, 255, 92, 0, 0, 255, 94, 0, 0, 255, 97, 0, 0, 255, 100, 0, 0, 255, 102, 0, 0, 255, 105, 0, 0, 255, 107, 0, 0, 255, 110, 0, 0, 255, 112, 0, 0, 255, 115, 0, 0, 255, 117, 0, 0, 255, 120, 0, 0, 255, 122, 0, 0, 255, 125, 0, 0, 255, 127, 0, 0, 255, 130, 0, 0, 255, 132, 0, 0, 255, 135, 0, 0, 255, 137, 0, 0, 255, 140, 0, 0, 255, 142, 0, 0, 255, 145, 0, 0, 255, 147, 0, 0, 255, 150, 0, 0, 255, 152, 0, 0, 255, 155, 0, 0, 255, 157, 0, 0, 255, 160, 0, 0, 255, 162, 0, 0, 255, 164, 0, 0, 255, 167, 0, 0, 255, 170, 0, 0, 255, 172, 0, 0, 255, 175, 0, 0, 255, 177, 0, 0, 255, 179, 0, 0, 255, 182, 0, 0, 255, 184, 0, 0, 255, 187, 0, 0, 255, 190, 0, 0, 255, 192, 0, 0, 255, 195, 0, 0, 255, 197, 0, 0, 255, 200, 0, 0, 255, 202, 0, 0, 255, 204, 0, 0, 255, 207, 0, 0, 255, 210, 0, 0, 255, 212, 0, 0, 255, 215, 0, 0, 255, 217, 0, 0, 255, 220, 0, 0, 255, 222, 0, 0, 255, 225, 0, 0, 255, 227, 0, 0, 255, 230, 0, 0, 255, 232, 0, 0, 255, 235, 0, 0, 255, 237, 0, 0, 255, 240, 0, 0, 255, 242, 0, 0, 255, 244, 0, 0, 255, 247, 0, 0, 255, 250, 0, 0, 255, 252, 0, 0, 255, 253, 1, 0, 255, 253, 3, 0, 255, 254, 5, 0, 255, 254, 7, 0, 255, 255, 9, 0, 255, 255, 12, 0, 255, 255, 14, 0, 255, 255, 17, 0, 255, 255, 20, 0, 255, 255, 22, 0, 255, 255, 25, 0, 255, 255, 27, 0, 255, 255, 30, 0, 255, 255, 32, 0, 255, 255, 35, 0, 255, 255, 37, 0, 255, 255, 40, 0, 255, 255, 42, 0, 255, 255, 45, 0, 255, 255, 47, 0, 255, 255, 49, 0, 255, 255, 52, 0, 255, 255, 54, 0, 255, 255, 57, 0, 255, 255, 60, 0, 255, 255, 62, 0, 255
  };
  return palette;
}

unsigned char* makeHotPalette101() {
  static unsigned char palette[] = {
    0, 0, 0, 255, 5, 0, 0, 255, 12, 0, 0, 255, 17, 0, 0, 255, 24, 0, 0, 255, 29, 0, 0, 255, 37, 0, 0, 255, 42, 0, 0, 255, 50, 0, 0, 255, 55, 0, 0, 255, 62, 0, 0, 255, 70, 0, 0, 255, 75, 0, 0, 255, 82, 0, 0, 255, 87, 0, 0, 255, 94, 0, 0, 255, 100, 0, 0, 255, 107, 0, 0, 255, 112, 0, 0, 255, 120, 0, 0, 255, 127, 0, 0, 255, 132, 0, 0, 255, 140, 0, 0, 255, 145, 0, 0, 255, 152, 0, 0, 255, 157, 0, 0, 255, 164, 0, 0, 255, 170, 0, 0, 255, 177, 0, 0, 255, 182, 0, 0, 255, 190, 0, 0, 255, 197, 0, 0, 255, 202, 0, 0, 255, 210, 0, 0, 255, 215, 0, 0, 255, 222, 0, 0, 255, 227, 0, 0, 255, 235, 0, 0, 255, 240, 0, 0, 255, 247, 0, 0, 255, 253, 1, 0, 255, 254, 5, 0, 255, 255, 12, 0, 255, 255, 17, 0, 255, 255, 25, 0, 255, 255, 30, 0, 255, 255, 37, 0, 255, 255, 42, 0, 255, 255, 49, 0, 255, 255, 54, 0, 255, 255, 62, 0, 255, 255, 69, 0, 255, 255, 74, 0, 255, 255, 82, 0, 255, 255, 87, 0, 255, 255, 95, 0, 255, 255, 99, 0, 255, 255, 107, 0, 255, 255, 112, 0, 255, 255, 119, 0, 255, 255, 127, 0, 255, 255, 132, 0, 255, 255, 140, 0, 255, 255, 145, 0, 255, 255, 152, 0, 255, 255, 157, 0, 255, 255, 164, 0, 255, 255, 169, 0, 255, 255, 177, 0, 255, 255, 182, 0, 255, 255, 190, 0, 255, 255, 197, 0, 255, 255, 202, 0, 255, 255, 210, 0, 255, 255, 215, 0, 255, 255, 222, 0, 255, 255, 227, 0, 255, 255, 234, 0, 255, 255, 240, 0, 255, 255, 247, 0, 255, 255, 252, 4, 255, 255, 254, 10, 255, 255, 255, 24, 255, 255, 255, 34, 255, 255, 255, 49, 255, 255, 255, 59, 255, 255, 255, 74, 255, 255, 255, 85, 255, 255, 255, 100, 255, 255, 255, 110, 255, 255, 255, 125, 255, 255, 255, 140, 255, 255, 255, 149, 255, 255, 255, 165, 255, 255, 255, 175, 255, 255, 255, 190, 255, 255, 255, 199, 255, 255, 255, 215, 255, 255, 255, 224, 255, 255, 255, 240, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  return palette;
}

Ogre::TexturePtr makePaletteTexture( unsigned char *palette_bytes )
{
  Ogre::DataStreamPtr palette_stream;
  palette_stream.bind( new Ogre::MemoryDataStream( palette_bytes, 256*4 ));

  static int palette_tex_count = 0;
  std::stringstream ss;
  ss << "MapPaletteTexture" << palette_tex_count++;
  return Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                           palette_stream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0 );
}

void MapDisplay::onInitialize()
{
  // Order of palette textures here must match option indices for color_scheme_property_ above.
  palette_textures_.push_back( makePaletteTexture( makeMapPalette() ));
  color_scheme_transparency_.push_back( false );
  
  palette_textures_.push_back( makePaletteTexture( makeCostmapPalette() ));
  color_scheme_transparency_.push_back( true );
  
  palette_textures_.push_back( makePaletteTexture( makeJetPalette101() ));
  color_scheme_transparency_.push_back( true );
  
  palette_textures_.push_back( makePaletteTexture( makeJetPalette256() ));
  color_scheme_transparency_.push_back( false );
  
  palette_textures_.push_back( makePaletteTexture( makeHotPalette101() ));
  color_scheme_transparency_.push_back( true );
  
  palette_textures_.push_back( makePaletteTexture( makeHotPalette256() ));
  color_scheme_transparency_.push_back( false );

  // Set up map material
  static int material_count = 0;
  std::stringstream ss;
  ss << "MapMaterial" << material_count++;
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone( ss.str() );

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias( -16.0f, 0.0f );
  material_->setCullingMode( Ogre::CULL_NONE );
  material_->setDepthWriteEnabled(false);

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "MapObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject( ss2.str() );
  scene_node_->attachObject( manual_object_ );

  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( 1.0f, 1.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top left
      manual_object_->position( 0.0f, 1.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      manual_object_->position( 1.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( 1.0f, 1.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  manual_object_->end();

  if( draw_under_property_->getValue().toBool() )
  {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  // don't show map until the plugin is actually enabled
  manual_object_->setVisible( false );

  updateAlpha();
}

void MapDisplay::onEnable()
{
  subscribe();
}

void MapDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void MapDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if( !topic_property_->getTopic().isEmpty() )
  {
    try
    {
      map_sub_ = update_nh_.subscribe( topic_property_->getTopicStd(), 1, &MapDisplay::incomingMap, this );
      setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
    }

    try
    {
      update_sub_ = update_nh_.subscribe( topic_property_->getTopicStd() + "_updates", 1, &MapDisplay::incomingUpdate, this );
      setStatus( StatusProperty::Ok, "Update Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Update Topic", QString( "Error subscribing: " ) + e.what() );
    }
  }
}

void MapDisplay::unsubscribe()
{
  map_sub_.shutdown();
  update_sub_.shutdown();
}

// helper class to set alpha parameter on all renderables.
class AlphaSetter: public Ogre::Renderable::Visitor
{
public:
  AlphaSetter( float alpha )
  : alpha_vec_( alpha, alpha, alpha, alpha )
  {}

  void visit( Ogre::Renderable *rend, ushort lodIndex, bool isDebug, Ogre::Any *pAny=0)
  {
    rend->setCustomParameter( ALPHA_PARAMETER, alpha_vec_ );
  }
private:
  Ogre::Vector4 alpha_vec_;
};

void MapDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  Ogre::Pass* pass = material_->getTechnique( 0 )->getPass( 0 );
  Ogre::TextureUnitState* tex_unit = NULL;

  if( alpha < 0.9998 || color_scheme_transparency_[ color_scheme_property_->getOptionInt() ])
  {
    material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->setDepthWriteEnabled( false );
  }
  else
  {
    material_->setSceneBlending( Ogre::SBT_REPLACE );
    material_->setDepthWriteEnabled( !draw_under_property_->getValue().toBool() );
  }

  AlphaSetter alpha_setter( alpha );
  if( manual_object_ )
  {
    manual_object_->visitRenderables( &alpha_setter );
  }
}

void MapDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getValue().toBool();

  if( alpha_property_->getFloat() >= 0.9998 )
  {
    material_->setDepthWriteEnabled( !draw_under );
  }

  if( manual_object_ )
  {
    if( draw_under )
    {
      manual_object_->setRenderQueueGroup( Ogre::RENDER_QUEUE_4 );
    }
    else
    {
      manual_object_->setRenderQueueGroup( Ogre::RENDER_QUEUE_MAIN );
    }
  }
}

void MapDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  clear();
}

void MapDisplay::clear()
{
  setStatus( StatusProperty::Warn, "Message", "No map received" );

  if( !loaded_ )
  {
    return;
  }

  if( manual_object_ )
  {
    manual_object_->setVisible( false );
  }

  if( !texture_.isNull() )
  {
    Ogre::TextureManager::getSingleton().remove( texture_->getName() );
    texture_.setNull();
  }

  loaded_ = false;
}

bool validateFloats(const nav_msgs::OccupancyGrid& msg)
{
  bool valid = true;
  valid = valid && validateFloats( msg.info.resolution );
  valid = valid && validateFloats( msg.info.origin );
  return valid;
}

void MapDisplay::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  current_map_ = *msg;
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
  loaded_ = true;
}


void MapDisplay::incomingUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr& update)
{
  // Only update the map if we have gotten a full one first.
  if( !loaded_ )
  {
    return;
  }

  // Reject updates which have any out-of-bounds data.
  if( update->x < 0 ||
      update->y < 0 ||
      current_map_.info.width < update->x + update->width ||
      current_map_.info.height < update->y + update->height )
  {
    setStatus( StatusProperty::Error, "Update", "Update area outside of original map area." );
    return;
  }

  // Copy the incoming data into current_map_'s data.
  for( size_t y = 0; y < update->height; y++ )
  {
    memcpy( &current_map_.data[ (update->y + y) * current_map_.info.width + update->x ],
            &update->data[ y * update->width ],
            update->width );
  }
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
}

void MapDisplay::showMap()
{
  if (current_map_.data.empty())
  {
    return;
  }

  if( !validateFloats( current_map_ ))
  {
    setStatus( StatusProperty::Error, "Map", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  if( current_map_.info.width * current_map_.info.height == 0 )
  {
    std::stringstream ss;
    ss << "Map is zero-sized (" << current_map_.info.width << "x" << current_map_.info.height << ")";
    setStatus( StatusProperty::Error, "Map", QString::fromStdString( ss.str() ));
    return;
  }

  setStatus( StatusProperty::Ok, "Message", "Map received" );

  ROS_DEBUG( "Received a %d X %d map @ %.3f m/pix\n",
             current_map_.info.width,
             current_map_.info.height,
             current_map_.info.resolution );

  float resolution = current_map_.info.resolution;

  int width = current_map_.info.width;
  int height = current_map_.info.height;


  Ogre::Vector3 position( current_map_.info.origin.position.x,
                          current_map_.info.origin.position.y,
                          current_map_.info.origin.position.z );
  Ogre::Quaternion orientation( current_map_.info.origin.orientation.w,
                                current_map_.info.origin.orientation.x,
                                current_map_.info.origin.orientation.y,
                                current_map_.info.origin.orientation.z );
  frame_ = current_map_.header.frame_id;
  if (frame_.empty())
  {
    frame_ = "/map";
  }

  unsigned int pixels_size = width * height;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  bool map_status_set = false;
  unsigned int num_pixels_to_copy = pixels_size;
  if( pixels_size != current_map_.data.size() )
  {
    std::stringstream ss;
    ss << "Data size doesn't match width*height: width = " << width
       << ", height = " << height << ", data size = " << current_map_.data.size();
    setStatus( StatusProperty::Error, "Map", QString::fromStdString( ss.str() ));
    map_status_set = true;

    // Keep going, but don't read past the end of the data.
    if( current_map_.data.size() < pixels_size )
    {
      num_pixels_to_copy = current_map_.data.size();
    }
  }

  memcpy( pixels, &current_map_.data[0], num_pixels_to_copy );

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind( new Ogre::MemoryDataStream( pixels, pixels_size ));

  if( !texture_.isNull() )
  {
    Ogre::TextureManager::getSingleton().remove( texture_->getName() );
    texture_.setNull();
  }

  static int tex_count = 0;
  std::stringstream ss;
  ss << "MapTexture" << tex_count++;
  try
  {
    texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                 pixel_stream, width, height, Ogre::PF_L8, Ogre::TEX_TYPE_2D,
                                                                 0);

    if( !map_status_set )
    {
      setStatus( StatusProperty::Ok, "Map", "Map OK" );
    }
  }
  catch(Ogre::RenderingAPIException&)
  {
    Ogre::Image image;
    pixel_stream->seek(0);
    float fwidth = width;
    float fheight = height;
    if( width > height )
    {
      float aspect = fheight / fwidth;
      fwidth = 2048;
      fheight = fwidth * aspect;
    }
    else
    {
      float aspect = fwidth / fheight;
      fheight = 2048;
      fwidth = fheight * aspect;
    }

    {
      std::stringstream ss;
      ss << "Map is larger than your graphics card supports.  Downsampled from [" << width << "x" << height << "] to [" << fwidth << "x" << fheight << "]";
      setStatus(StatusProperty::Ok, "Map", QString::fromStdString( ss.str() ));
    }

    ROS_WARN("Failed to create full-size map texture, likely because your graphics card does not support textures of size > 2048.  Downsampling to [%d x %d]...", (int)fwidth, (int)fheight);
    //ROS_INFO("Stream size [%d], width [%f], height [%f], w * h [%f]", pixel_stream->size(), width, height, width * height);
    image.loadRawData(pixel_stream, width, height, Ogre::PF_L8);
    image.resize(fwidth, fheight, Ogre::Image::FILTER_NEAREST);
    ss << "Downsampled";
    texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
  }

  delete [] pixels;

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_->getName());
  tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  updatePalette();

  resolution_property_->setValue( resolution );
  width_property_->setValue( width );
  height_property_->setValue( height );
  position_property_->setVector( position );
  orientation_property_->setQuaternion( orientation );

  transformMap();
  manual_object_->setVisible( true );
  scene_node_->setScale( resolution * width, resolution * height, 1.0 );

  context_->queueRender();
}

void MapDisplay::updatePalette()
{
  int palette_index = color_scheme_property_->getOptionInt();

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* palette_tex_unit = NULL;
  if( pass->getNumTextureUnitStates() > 1 )
  {
    palette_tex_unit = pass->getTextureUnitState( 1 );
  }
  else
  {
    palette_tex_unit = pass->createTextureUnitState();
  }
  palette_tex_unit->setTextureName( palette_textures_[ palette_index ]->getName() );
  palette_tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  updateAlpha();
}

void MapDisplay::transformMap()
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(frame_, ros::Time(), current_map_.info.origin, position, orientation))
  {
    ROS_DEBUG( "Error transforming map '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), frame_.c_str(), qPrintable( fixed_frame_ ));

    setStatus( StatusProperty::Error, "Transform",
               "No transform from [" + QString::fromStdString( frame_ ) + "] to [" + fixed_frame_ + "]" );
  }
  else
  {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );
}

void MapDisplay::fixedFrameChanged()
{
  transformMap();
}

void MapDisplay::reset()
{
  Display::reset();

  clear();
  // Force resubscription so that the map will be re-sent
  updateTopic();
}

void MapDisplay::setTopic( const QString &topic, const QString &datatype )
{
  topic_property_->setString( topic );
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::MapDisplay, rviz::Display )
