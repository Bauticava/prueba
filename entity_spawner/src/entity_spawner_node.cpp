#include <cctype>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/variant.hpp>

#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "entity_spawner/srv/load_entity.hpp"
#include "entity_spawner/srv/spawn_entity.hpp"
#include "entity_spawner/srv/remove_entity.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"

namespace entity_spawner {

struct Entity {
  std::string name;
  std::string source_path;
  shape_msgs::msg::Mesh mesh;
};

shape_msgs::msg::Mesh scaleMesh(const shape_msgs::msg::Mesh &mesh,
                                double scale) {
  shape_msgs::msg::Mesh scaled_mesh = mesh;
  for (auto &vertex : scaled_mesh.vertices) {
    vertex.x *= scale;
    vertex.y *= scale;
    vertex.z *= scale;
  }
  return scaled_mesh;
}

class EntitySpawnerNode : public rclcpp::Node {
public:
  EntitySpawnerNode() : rclcpp::Node("entity_spawner_node") {
    simulation_client_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    load_entity_service_ =
        this->create_service<entity_spawner::srv::LoadEntity>(
            "loadEntity",
            std::bind(&EntitySpawnerNode::handleLoadEntity, this,
                      std::placeholders::_1, std::placeholders::_2));

    spawn_entity_service_ =
        this->create_service<entity_spawner::srv::SpawnEntity>(
            "spawnEntity",
            std::bind(&EntitySpawnerNode::handleSpawnEntity, this,
                      std::placeholders::_1, std::placeholders::_2));
    
    remove_entity_service_ =
        this->create_service<entity_spawner::srv::RemoveEntity>(
            "removeEntity",
            std::bind(&EntitySpawnerNode::handleRemoveEntity, this,
                      std::placeholders::_1, std::placeholders::_2));

    default_sim_world_ = this->declare_parameter<std::string>(
        "simulation.default_world", "default");
    if (default_sim_world_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Parameter 'simulation.default_world' "
                                      "was empty. Falling back to 'default'.");
      default_sim_world_ = "default";
    }

    default_sim_reference_frame_ = this->declare_parameter<std::string>(
        "simulation.reference_frame", "world");
    if (default_sim_reference_frame_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Parameter 'simulation.reference_frame' "
                                      "was empty. Falling back to 'world'.");
      default_sim_reference_frame_ = "world";
    }

    const double timeout_seconds =
        this->declare_parameter<double>("simulation.service_timeout", 5.0);
    if (timeout_seconds <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter 'simulation.service_timeout' is not positive "
                  "(%.3f). Using 2.0 seconds instead.",
                  timeout_seconds);
      simulation_service_timeout_ = std::chrono::milliseconds(2000);
    } else {
      simulation_service_timeout_ =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>(timeout_seconds));
    }

    RCLCPP_INFO(this->get_logger(),
                "EntitySpawnerNode ready to load and spawn entities.");
  }

private:
  void handleLoadEntity(
      const std::shared_ptr<entity_spawner::srv::LoadEntity::Request> request,
      std::shared_ptr<entity_spawner::srv::LoadEntity::Response> response) {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto entity_name = request->entity_name;
    if (entity_name.empty()) {
      response->success = false;
      response->message = "Entity name must not be empty.";
      return;
    }

    if (entity_cache_.count(entity_name) != 0) {
      response->success = false;
      response->message = "Entity name already loaded.";
      return;
    }

    std::error_code ec;
    auto resolved_path = std::filesystem::absolute(request->stl_path, ec);
    if (ec) {
      response->success = false;
      response->message = "Unable to resolve STL path: " + ec.message();
      return;
    }

    const std::string resolved_path_str = resolved_path.string();

    if (!std::filesystem::exists(resolved_path)) {
      response->success = false;
      response->message = "STL file does not exist: " + resolved_path_str;
      return;
    }

    if (!resolved_path.has_extension() || resolved_path.extension() != ".stl") {
      response->success = false;
      response->message =
          "Provided file is not an .stl model: " + resolved_path_str;
      return;
    }

    const std::string resource_uri = "file://" + resolved_path_str;

    shapes::Mesh *mesh_ptr = shapes::createMeshFromResource(resource_uri);
    if (!mesh_ptr) {
      response->success = false;
      response->message = "Failed to parse STL mesh: " + resolved_path_str;
      return;
    }

    shapes::ShapePtr shape_ptr(mesh_ptr);
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(shape_ptr.get(), shape_msg);

    shape_msgs::msg::Mesh mesh_msg;
    try {
      mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
    } catch (const boost::bad_get & /*ex*/) {
      response->success = false;
      response->message = "Loaded shape is not a mesh: " + resolved_path_str;
      return;
    }

    Entity entity;
    entity.name = entity_name;
    entity.source_path = resolved_path_str;
    entity.mesh = mesh_msg;

    entity_cache_.emplace(entity_name, std::move(entity));

    response->success = true;
    response->message = "Entity loaded and queued: " + entity_name;

    RCLCPP_INFO(get_logger(), "Loaded entity '%s' from '%s'",
                entity_name.c_str(), resolved_path_str.c_str());
  }

  void handleSpawnEntity(
      const std::shared_ptr<entity_spawner::srv::SpawnEntity::Request> request,
      std::shared_ptr<entity_spawner::srv::SpawnEntity::Response> response) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = entity_cache_.find(request->entity_name);
    if (it == entity_cache_.end()) {
      response->success = false;
      response->message = "Entity not found in queue: " + request->entity_name;
      return;
    }

    bool add_to_planning_scene = request->add_to_planning_scene;
    bool spawn_in_sim_world = request->spawn_in_sim_world;
    if (!add_to_planning_scene && !spawn_in_sim_world) {
      add_to_planning_scene = true;
    }

    if (add_to_planning_scene && request->frame_id.empty()) {
      response->success = false;
      response->message =
          "Frame id must not be empty when adding to the planning scene.";
      return;
    }

    if (request->scale <= 0.0) {
      response->success = false;
      response->message = "Scale must be greater than zero.";
      return;
    }

    std::string target_world = default_sim_world_;
    std::string target_reference_frame = default_sim_reference_frame_;
    if (spawn_in_sim_world) {
      if (!request->sim_world.empty()) {
        target_world = request->sim_world;
      }
      if (!request->sim_reference_frame.empty()) {
        target_reference_frame = request->sim_reference_frame;
      }
      if (target_world.empty()) {
        response->success = false;
        response->message =
            "Simulation world name resolved to an empty string.";
        return;
      }
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = request->x;
    pose.position.y = request->y;
    pose.position.z = request->z;

    tf2::Quaternion orientation;
    orientation.setRPY(request->roll, request->pitch, request->yaw);
    pose.orientation = tf2::toMsg(orientation);

    bool planning_scene_applied = false;
    if (add_to_planning_scene) {
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.id = request->entity_name;
      collision_object.header.frame_id = request->frame_id;
      collision_object.meshes.push_back(
          scaleMesh(it->second.mesh, request->scale));
      collision_object.mesh_poses.push_back(pose);
      collision_object.operation = collision_object.ADD;

      const bool applied =
          planning_scene_interface_.applyCollisionObject(collision_object);
      if (!applied) {
        response->success = false;
        response->message =
            "Failed to apply collision object to planning scene.";
        return;
      }
      planning_scene_applied = true;
    }

    if (spawn_in_sim_world) {
      std::string error_message;
      if (!spawnEntityInSimulation(it->second, pose, *request, target_world,
                                   target_reference_frame, error_message)) {
        if (planning_scene_applied) {
          planning_scene_interface_.removeCollisionObjects(
              {request->entity_name});
        }
        response->success = false;
        response->message = error_message;
        return;
      }
    }

    response->success = true;
    if (add_to_planning_scene && spawn_in_sim_world) {
      response->message = "Entity '" + request->entity_name +
                          "' spawned in planning scene and world '" +
                          target_world + "'.";
    } else if (add_to_planning_scene) {
      response->message =
          "Entity spawned in planning scene: " + request->entity_name;
    } else {
      response->message = "Entity '" + request->entity_name +
                          "' spawned in world '" + target_world + "'.";
    }

    if (spawn_in_sim_world) {
      RCLCPP_INFO(get_logger(),
                  "Spawned entity '%s' in world '%s' at (%.3f, %.3f, %.3f) "
                  "with scale %.3f (reference '%s')",
                  request->entity_name.c_str(), target_world.c_str(),
                  request->x, request->y, request->z, request->scale,
                  target_reference_frame.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
                  "Spawned entity '%s' in planning scene at (%.3f, %.3f, %.3f) "
                  "frame '%s' scale %.3f",
                  request->entity_name.c_str(), request->x, request->y,
                  request->z, request->frame_id.c_str(), request->scale);
    }

    entity_cache_.erase(it);
  }

  void handleRemoveEntity(
      const std::shared_ptr<entity_spawner::srv::RemoveEntity::Request> request,
      std::shared_ptr<entity_spawner::srv::RemoveEntity::Response> response) {
  
  // Eliminar de la escena de planificación de MoveIt
    planning_scene_interface_.removeCollisionObjects({request->entity_name});
    RCLCPP_INFO(get_logger(), "Entity '%s' removed from planning scene.", request->entity_name.c_str());

  // Eliminar de Gazebo
  // NOTA: Asumimos que el nombre en Gazebo es el mismo.
  // En un futuro, podríamos guardar un mapa de nombres si se sanitizan.
    const std::string simulation_name = sanitizeName(request->entity_name);

  // Usamos el cliente de spawn, pero para el servicio de borrado.
  // Esto puede ser refactorizado, pero funciona para nuestro propósito.
  // Creamos un cliente para el servicio de borrado de Gazebo.
  // El nombre del mundo es "default" por ahora.
    const std::string world_name = "default";
    const std::string service_name = "/world/" + world_name + "/delete_entity";
    auto delete_client = this->create_client<ros_gz_interfaces::srv::DeleteEntity>(
        service_name, rmw_qos_profile_services_default, simulation_client_group_);
  
    if (!delete_client->wait_for_service(simulation_service_timeout_)) {
      response->success = false;
      response->message = "Gazebo delete service not available for world '" + world_name + "'.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
  
    auto delete_request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    delete_request->entity.name = simulation_name;
  
    auto future = delete_client->async_send_request(delete_request);
    const auto future_status = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future, simulation_service_timeout_);

    if (future_status != rclcpp::FutureReturnCode::SUCCESS) {
      response->success = false;
      response->message = "Timed out or was interrupted while calling Gazebo delete service.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

  // Comprobar si Gazebo lo borró con éxito
    try {
        const auto result = future.get();
        if (!result->success) {
            RCLCPP_WARN(get_logger(), "Gazebo reported failure trying to delete '%s'. Maybe it was already gone?", simulation_name.c_str());
        }
    } catch (const std::exception &ex) {
        response->success = false;
        response->message = std::string("Exception while calling Gazebo delete service: ") + ex.what();
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    response->success = true;
    response->message = "Entity '" + request->entity_name + "' removed successfully.";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }


  bool spawnEntityInSimulation(
      const Entity &entity, const geometry_msgs::msg::Pose &pose,
      const entity_spawner::srv::SpawnEntity::Request &request,
      const std::string &world, const std::string &reference_frame,
      std::string &error_message) {
    auto client = getSimulationClient(world);
    if (!client) {
      error_message = "Failed to create simulation spawn client.";
      return false;
    }

    if (!client->wait_for_service(simulation_service_timeout_)) {
      std::ostringstream oss;
      oss << "Simulation spawn service '/world/" << world
          << "/create' not available.";
      error_message = oss.str();
      return false;
    }

    auto request_msg =
        std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    const std::string simulation_name = sanitizeName(entity.name);
    if (simulation_name != entity.name) {
      RCLCPP_WARN(
          this->get_logger(),
          "Entity name '%s' adapted to '%s' for simulation compatibility.",
          entity.name.c_str(), simulation_name.c_str());
    }
    request_msg->entity_factory.name = simulation_name;
    request_msg->entity_factory.allow_renaming = false;
    request_msg->entity_factory.sdf =
        buildSimulationSdf(entity, request.scale, simulation_name);
    request_msg->entity_factory.pose = pose;
    request_msg->entity_factory.relative_to =
        reference_frame.empty() ? "world" : reference_frame;

    auto future = client->async_send_request(request_msg);
    const auto future_status = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future, simulation_service_timeout_);

    if (future_status != rclcpp::FutureReturnCode::SUCCESS) {
      if (future_status == rclcpp::FutureReturnCode::TIMEOUT) {
        error_message =
            "Timed out while awaiting response from simulation spawn service.";
      } else {
        error_message = "Interrupted while awaiting response from simulation "
                        "spawn service.";
      }
      return false;
    }

    try {
      const auto result = future.get();
      if (!result->success) {
        error_message = "Simulation spawn service reported failure while "
                        "spawning entity '" +
                        request.entity_name + "'.";
        return false;
      }
    } catch (const std::exception &ex) {
      error_message =
          std::string("Failed to retrieve simulation spawn response: ") +
          ex.what();
      return false;
    }

    return true;
  }

  std::string sanitizeName(const std::string &name) const {
    if (name.empty()) {
      return "entity";
    }

    std::string sanitized = name;
    for (char &ch : sanitized) {
      const unsigned char value = static_cast<unsigned char>(ch);
      if (!std::isalnum(value) && ch != '_' && ch != '-') {
        ch = '_';
      }
    }

    if (!std::isalpha(static_cast<unsigned char>(sanitized.front())) &&
        sanitized.front() != '_') {
      sanitized.insert(sanitized.begin(), '_');
    }

    return sanitized;
  }

  std::string buildSimulationSdf(const Entity &entity, double scale,
                                 const std::string &simulation_name) const {
    const std::string link_name = simulation_name + "_link";
    const std::string visual_name = simulation_name + "_visual";
    const std::string collision_name = simulation_name + "_collision";

    std::ostringstream sdf;
    sdf << std::fixed << std::setprecision(6);
    sdf << "<?xml version=\"1.0\"?>\n";
    sdf << "<sdf version=\"1.10\">\n";
    sdf << "  <model name=\"" << simulation_name << "\">\n";
    sdf << "    <static>true</static>\n";
    sdf << "    <link name=\"" << link_name << "\">\n";
    sdf << "      <visual name=\"" << visual_name << "\">\n";
    sdf << "        <geometry>\n";
    sdf << "          <mesh>\n";
    sdf << "            <uri>file://" << entity.source_path << "</uri>\n";
    sdf << "            <scale>" << scale << " " << scale << " " << scale
        << "</scale>\n";
    sdf << "          </mesh>\n";
    sdf << "        </geometry>\n";
    sdf << "      </visual>\n";
    sdf << "      <collision name=\"" << collision_name << "\">\n";
    sdf << "        <geometry>\n";
    sdf << "          <mesh>\n";
    sdf << "            <uri>file://" << entity.source_path << "</uri>\n";
    sdf << "            <scale>" << scale << " " << scale << " " << scale
        << "</scale>\n";
    sdf << "          </mesh>\n";
    sdf << "        </geometry>\n";
    sdf << "      </collision>\n";
    sdf << "    </link>\n";
    sdf << "  </model>\n";
    sdf << "</sdf>\n";
    return sdf.str();
  }

  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr
  getSimulationClient(const std::string &world) {
    auto it = sim_spawn_clients_.find(world);
    if (it != sim_spawn_clients_.end()) {
      return it->second;
    }

    const std::string service_name = "/world/" + world + "/create";
    auto client = this->create_client<ros_gz_interfaces::srv::SpawnEntity>(
        service_name, rclcpp::ServicesQoS(), simulation_client_group_);
    sim_spawn_clients_.emplace(world, client);
    return client;
  }

  rclcpp::Service<entity_spawner::srv::LoadEntity>::SharedPtr
      load_entity_service_;
  rclcpp::Service<entity_spawner::srv::SpawnEntity>::SharedPtr
      spawn_entity_service_;
  rclcpp::Service<entity_spawner::srv::RemoveEntity>::SharedPtr  
      remove_entity_service_;    

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::string default_sim_world_;
  std::string default_sim_reference_frame_;
  std::chrono::milliseconds simulation_service_timeout_{5000};
  rclcpp::CallbackGroup::SharedPtr simulation_client_group_;
  std::unordered_map<
      std::string,
      rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr>
      sim_spawn_clients_;

  std::mutex mutex_;
  std::unordered_map<std::string, Entity> entity_cache_;
};

} // namespace entity_spawner
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<entity_spawner::EntitySpawnerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
