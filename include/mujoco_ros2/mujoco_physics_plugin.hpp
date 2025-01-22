/* 
 * @file mujoco_physics_plugin.hpp
 * @date 1/22/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef MUJOCO_PHYSICS_PLUGIN_HPP
#define MUJOCO_PHYSICS_PLUGIN_HPP

#include "mujoco/mujoco.h"

namespace mujoco_ros2 {
class MujocoPhysicsPlugin {
public:
  virtual ~MujocoPhysicsPlugin() = default;

  // interface
  virtual void Configure(mjModel *model, mjData *data) = 0;
  virtual void Reset(mjModel *model, mjData *data) = 0;
  virtual void PreUpdate(mjModel *model, mjData *data) {};
  virtual void PostUpdate(mjModel *model, mjData *data) {};

protected:
  MujocoPhysicsPlugin() = default;
};
} // mujoco_ros2

#endif //MUJOCO_PHYSICS_PLUGIN_HPP