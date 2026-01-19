#include <rlc_planner/scene_env_adapter.hpp>

#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/world.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tesseract_common/eigen_types.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_geometry/box.h>
#include <tesseract_geometry/cone.h>
#include <tesseract_geometry/cylinder.h>
#include <tesseract_geometry/mesh.h>
#include <tesseract_geometry/plane.h>
#include <tesseract_geometry/sphere.h>
#include <tesseract_rosutils/ros_resource_locator.h>
#include <tesseract_scene_graph/collision.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cctype>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rlc_planner {

namespace {

std::string sanitize_token(const std::string& s) {
  std::string out;
  out.reserve(s.size());

  for (char c : s) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) || c == '_') {
      out.push_back(c);
    } else {
      out.push_back('_');
    }
  }

  if (out.empty()) out = "obj";
  if (std::isdigit(static_cast<unsigned char>(out.front())))
    out.insert(out.begin(), '_');
  return out;
}

std::string make_link_name(const SceneEnvAdapter::Options& opt,
                           const std::string& object_id) {
  const std::string token =
      opt.sanitize_object_ids ? sanitize_token(object_id) : object_id;
  return opt.world_object_link_prefix + token;
}

std::string make_joint_name(const SceneEnvAdapter::Options& opt,
                            const std::string& object_id) {
  const std::string token =
      opt.sanitize_object_ids ? sanitize_token(object_id) : object_id;
  return opt.world_object_joint_prefix + token;
}

std::shared_ptr<const tesseract_geometry::Geometry> to_tesseract_geometry(
    const shapes::ShapeConstPtr& s) {
  if (!s) throw std::runtime_error("to_tesseract_geometry: null shape");

  switch (s->type) {
    case shapes::BOX: {
      const auto* box = static_cast<const shapes::Box*>(s.get());
      return std::make_shared<tesseract_geometry::Box>(
          box->size[0], box->size[1], box->size[2]);
    }
    case shapes::SPHERE: {
      const auto* sph = static_cast<const shapes::Sphere*>(s.get());
      return std::make_shared<tesseract_geometry::Sphere>(sph->radius);
    }
    case shapes::CYLINDER: {
      const auto* cyl = static_cast<const shapes::Cylinder*>(s.get());
      return std::make_shared<tesseract_geometry::Cylinder>(cyl->radius,
                                                            cyl->length);
    }
    case shapes::CONE: {
      const auto* cone = static_cast<const shapes::Cone*>(s.get());
      return std::make_shared<tesseract_geometry::Cone>(cone->radius,
                                                        cone->length);
    }
    case shapes::PLANE: {
      const auto* p = static_cast<const shapes::Plane*>(s.get());
      return std::make_shared<tesseract_geometry::Plane>(p->a, p->b, p->c,
                                                         p->d);
    }
    case shapes::MESH: {
      const auto* m = static_cast<const shapes::Mesh*>(s.get());

      if (m->vertex_count == 0 || m->triangle_count == 0)
        throw std::runtime_error("to_tesseract_geometry: empty mesh");

      auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
      vertices->reserve(m->vertex_count);
      for (unsigned int i = 0; i < m->vertex_count; ++i) {
        const double x = m->vertices[3 * i + 0];
        const double y = m->vertices[3 * i + 1];
        const double z = m->vertices[3 * i + 2];
        vertices->emplace_back(x, y, z);
      }

      // Faces encoding: for each triangle => [3, i0, i1, i2]
      auto faces = std::make_shared<Eigen::VectorXi>();
      faces->resize(static_cast<int>(m->triangle_count) * 4);
      for (unsigned int t = 0; t < m->triangle_count; ++t) {
        (*faces)[static_cast<int>(4 * t + 0)] = 3;
        (*faces)[static_cast<int>(4 * t + 1)] =
            static_cast<int>(m->triangles[3 * t + 0]);
        (*faces)[static_cast<int>(4 * t + 2)] =
            static_cast<int>(m->triangles[3 * t + 1]);
        (*faces)[static_cast<int>(4 * t + 3)] =
            static_cast<int>(m->triangles[3 * t + 2]);
      }

      return std::make_shared<tesseract_geometry::Mesh>(
          vertices, faces, static_cast<int>(m->triangle_count));
    }
    default:
      throw std::runtime_error("to_tesseract_geometry: unsupported shape type");
  }
}

}  // namespace

void SceneEnvAdapter::initialize(const std::string& urdf_xml,
                                 const std::string& srdf_xml, Options options) {
  std::scoped_lock<std::mutex> lk(mutex_);

  options_ = std::move(options);

  env_robot_base_ = std::make_shared<tesseract_environment::Environment>();

  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_robot_base_->init(urdf_xml, srdf_xml, locator)) {
    throw std::runtime_error(
        "SceneEnvAdapter::initialize: tesseract Environment init failed");
  }

  // Default parent link for world objects if not provided.
  if (options_.world_parent_link.empty()) {
    options_.world_parent_link = env_robot_base_->getRootLinkName();
  }

  // Validate parent link exists.
  if (!env_robot_base_->getLink(options_.world_parent_link)) {
    throw std::runtime_error(
        "SceneEnvAdapter::initialize: world_parent_link '" +
        options_.world_parent_link + "' does not exist in the environment");
  }
}

tesseract_environment::Environment::UPtr SceneEnvAdapter::make_snapshot(
    const std::shared_ptr<const planning_scene::PlanningScene>& scene) const {
  if (!scene)
    throw std::runtime_error(
        "SceneEnvAdapter::make_snapshot: planning_scene is null");

  std::shared_ptr<tesseract_environment::Environment> base;
  Options opt_copy;

  {
    std::scoped_lock<std::mutex> lk(mutex_);
    if (!env_robot_base_)
      throw std::runtime_error(
          "SceneEnvAdapter::make_snapshot: initialize() was not called");

    base = env_robot_base_;
    opt_copy = options_;
  }

  auto env = base->clone();
  if (!env)
    throw std::runtime_error(
        "SceneEnvAdapter::make_snapshot: env_robot_base_->clone() failed");

  // Rebuild request-scoped state.
  add_world_objects(*env, *scene);
  sync_robot_state(*env, *scene);

  return env;
}

void SceneEnvAdapter::add_world_objects(
    tesseract_environment::Environment& env,
    const planning_scene::PlanningScene& scene) const {
  const auto world = scene.getWorld();
  if (!world)
    throw std::runtime_error(
        "SceneEnvAdapter::add_world_objects: scene world is null");

  const auto ids = world->getObjectIds();

  for (const auto& id : ids) {
    const auto obj = world->getObject(id);
    if (!obj) continue;

    if (obj->shapes_.size() != obj->shape_poses_.size()) {
      throw std::runtime_error(
          "SceneEnvAdapter::add_world_objects: shapes_ and shape_poses_ size "
          "mismatch for id '" +
          id + "'");
    }

    const std::string link_name = make_link_name(options_, id);
    const std::string joint_name = make_joint_name(options_, id);

    tesseract_scene_graph::Link link(link_name);

    link.collision.reserve(obj->shapes_.size());
    for (std::size_t i = 0; i < obj->shapes_.size(); ++i) {
      auto col = std::make_shared<tesseract_scene_graph::Collision>();
      col->name = link_name + "__col__" + std::to_string(i);
      col->origin = obj->shape_poses_[i];  // local to object frame
      col->geometry = to_tesseract_geometry(obj->shapes_[i]);
      link.collision.push_back(std::move(col));
    }

    const Eigen::Isometry3d world_T_obj = world->getTransform(id);

    tesseract_scene_graph::Joint joint(joint_name);
    joint.parent_link_name = options_.world_parent_link;
    joint.child_link_name = link_name;
    joint.type = tesseract_scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = world_T_obj;

    // replace = true is safe and makes this idempotent if the caller ever
    // reuses names.
    auto cmd = std::make_shared<tesseract_environment::AddLinkCommand>(
        link, joint, true);
    if (!env.applyCommand(cmd)) {
      throw std::runtime_error(
          "SceneEnvAdapter::add_world_objects: failed AddLinkCommand for world "
          "object id '" +
          id + "' (link '" + link_name + "')");
    }
  }
}

void SceneEnvAdapter::sync_robot_state(
    tesseract_environment::Environment& env,
    const planning_scene::PlanningScene& scene) const {
  const auto& rs = scene.getCurrentState();
  const auto& rm = scene.getRobotModel();

  if (!rm)
    throw std::runtime_error(
        "SceneEnvAdapter::sync_robot_state: robot model is null");

  // Use Tesseract's active joint list to avoid mismatches and to keep this
  // extensible.
  const std::vector<std::string> active_joints = env.getActiveJointNames();

  std::unordered_map<std::string, double> joint_values;
  joint_values.reserve(active_joints.size());

  for (const auto& joint_name : active_joints) {
    const moveit::core::JointModel* jm = rm->getJointModel(joint_name);
    if (!jm) {
      // Joint exists in Tesseract env but not in MoveIt model (e.g., extra
      // fixed/world joints).
      continue;
    }

    // Essentials: handle single-DOF joints directly.
    // Extension point: planar/floating joints can be supported by also passing
    // floating_joints.
    if (jm->getVariableCount() != 1) {
      continue;
    }

    const std::string& var = jm->getVariableNames().front();
    joint_values.emplace(joint_name, rs.getVariablePosition(var));
  }

  env.setState(joint_values);
}

}  // namespace rlc_planner
