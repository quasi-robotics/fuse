/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FUSE_VIZ__MAPPED_COVARIANCE_VISUAL_HPP_
#define FUSE_VIZ__MAPPED_COVARIANCE_VISUAL_HPP_

#include <OgreColourValue.h>
#include <Ogre.h>

#include <Eigen/Dense>
#include <cmath>

#include <rviz_rendering/objects/object.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include <geometry_msgs/msg/pose_with_covariance.hpp>


namespace Ogre
{
class SceneManager;
class SceneNode;
class Any;
}  // namespace Ogre

namespace Eigen
{
typedef Matrix<double, 6, 6> Matrix6d;
}

namespace fuse_viz
{

/**
 * \class MappedCovarianceVisual
 * \brief MappedCovarianceVisual consisting in a ellipse for position and 2D ellipses along the axis
 *                               for orientation.
 *
 * This is mostly a copy of CovarianceVisual from rviz/default_plugin/covariance_visual.h that
 * allows MappedCovarianceProperty be a friend class of MappedCovarianceVisual, so it can call its
 * constructor.
 */
class MappedCovarianceVisual : public rviz_rendering::Object
{
public:
  enum ShapeIndex
  {
    kRoll = 0,
    kPitch = 1,
    kYaw = 2,
    kYaw2D = 3,
    kNumOriShapes
  };

private:
  /**
   * \brief Private Constructor
   *
   * MappedCovarianceVisual can only be constructed by friend class MappedCovarianceProperty.
   *
   * @param scene_manager The scene manager to use to construct any necessary objects
   * @param parent_object A rviz object that this covariance will be attached.
   * @param is_local_rotation Initial attachment of the rotation part
   * @param is_visible Initial visibility
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  MappedCovarianceVisual(
    Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node, bool is_local_rotation,
    bool is_visible = true, float pos_scale = 1.0f, float ori_scale = 0.1f,
    float ori_offset = 0.1f);

public:
  ~MappedCovarianceVisual() override;

  /**
   * \brief Set the position and orientation scales for this covariance
   *
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  void setScales(float pos_scale, float ori_scale);
  void setPositionScale(float pos_scale);
  void setOrientationOffset(float ori_offset);
  void setOrientationScale(float ori_scale);

  /**
   * \brief Set the color of the position covariance. Values are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  virtual void setPositionColor(float r, float g, float b, float a);
  void setPositionColor(const Ogre::ColourValue & color);

  /**
   * \brief Set the color of the orientation covariance. Values are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  virtual void setOrientationColor(float r, float g, float b, float a);
  void setOrientationColor(const Ogre::ColourValue & color);
  void setOrientationColorToRGB(float a);

  /** @brief Set the covariance.
   *
   * This effectively changes the orientation and scale of position and orientation
   * covariance shapes
   */
  virtual void setCovariance(const geometry_msgs::msg::PoseWithCovariance & pose);

  virtual const Ogre::Vector3 & getPositionCovarianceScale();
  virtual const Ogre::Quaternion & getPositionCovarianceOrientation();

  /**
   * \brief Get the root scene node of the position part of this covariance
   * @return the root scene node of the position part of this covariance
   */
  Ogre::SceneNode * getPositionSceneNode()
  {
    return position_scale_node_;
  }

  /**
   * \brief Get the root scene node of the orientation part of this covariance
   * @return the root scene node of the orientation part of this covariance
   */
  Ogre::SceneNode * getOrientationSceneNode()
  {
    return orientation_root_node_;
  }

  /**
   * \brief Get the shape used to display position covariance
   * @return the shape used to display position covariance
   */
  rviz_rendering::Shape * getPositionShape()
  {
    return position_shape_;
  }

  /**
   * \brief Get the shape used to display orientation covariance in an especific axis
   * @return the shape used to display orientation covariance in an especific axis
   */
  rviz_rendering::Shape * getOrientationShape(ShapeIndex index);

  /**
   * \brief Sets user data on all ogre objects we own
   */
  virtual void setUserData(const Ogre::Any & data);

  /**
   * \brief Sets visibility of this covariance
   *
   * Convenience method that sets visibility of both position and orientation parts.
   */
  virtual void setVisible(bool visible);

  /**
   * \brief Sets visibility of the position part of this covariance
   */
  virtual void setPositionVisible(bool visible);

  /**
   * \brief Sets visibility of the orientation part of this covariance
   */
  virtual void setOrientationVisible(bool visible);

  /**
   * \brief Sets position of the frame this covariance is attached
   */
  virtual void setPosition(const Ogre::Vector3 & position);

  /**
   * \brief Sets orientation of the frame this covariance is attached
   */
  virtual void setOrientation(const Ogre::Quaternion & orientation);

  /**
   * \brief Sets which frame to attach the covariance of the orientation
   */
  virtual void setRotatingFrame(bool use_rotating_frame);

private:
  void updatePosition(const Eigen::Matrix6d & covariance);
  void updateOrientation(const Eigen::Matrix6d & covariance, ShapeIndex index);
  void updateOrientationVisibility();

  Ogre::SceneNode * root_node_ = nullptr;
  Ogre::SceneNode * fixed_orientation_node_ = nullptr;
  Ogre::SceneNode * position_scale_node_ = nullptr;
  Ogre::SceneNode * position_node_ = nullptr;

  Ogre::SceneNode * orientation_root_node_ = nullptr;
  Ogre::SceneNode * orientation_offset_node_[kNumOriShapes];

  rviz_rendering::Shape * position_shape_;                    //!< Ellipse used for the position
                                                              //!< covariance
  rviz_rendering::Shape * orientation_shape_[kNumOriShapes];  //!< Cylinders used for the
                                                              //!< orientation covariance

  bool local_rotation_;

  bool pose_2d_;

  bool orientation_visible_;  //!< If the orientation component is visible.

  Ogre::Vector3 current_ori_scale_[kNumOriShapes];
  float current_ori_scale_factor_;

  static const float max_degrees;

private:
  // Hide Object methods we don't want to expose
  // NOTE: Apparently we still need to define them...
  virtual void setScale(const Ogre::Vector3 &)
  {
  }
  virtual void setColor(float, float, float, float)
  {
  }
  virtual const Ogre::Vector3 & getPosition();
  virtual const Ogre::Quaternion & getOrientation();

  // Make MappedCovarianceProperty friend class so it create MappedCovarianceVisual objects
  friend class MappedCovarianceProperty;
};

}  // namespace fuse_viz

#endif  // FUSE_VIZ__MAPPED_COVARIANCE_VISUAL_HPP_
