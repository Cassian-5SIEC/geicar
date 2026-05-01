#include "behaviortree_ros2/bt_action_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "visualization_msgs/msg/marker.hpp"
#include <thread>
#include <atomic>
#include <cmath>

using namespace BT;

struct Wayepoint {
    double x, y, theta;
};

class SelectNearestWaypoint : public StatefulActionNode
{
public:
    SelectNearestWaypoint(const std::string& name, const NodeConfig& config, const std::shared_ptr<rclcpp::Node> nh)
        : StatefulActionNode(name, config), node_(nh)
    {
        // 1. Initialisation TF2 (Indispensable pour savoir où est le robot)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher pour visualiser le point choisi dans Rviz
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/patrol_selection_marker", 10);

        // Définition des points (Idéalement chargé via paramètres ROS)
        waypoints_ = {
            {-0.847, 0.972, 1.199},   // WP 0
            {1.348, 2.033, -0.320},  // WP 1
            {3.137, 0.109, -1.915},  // WP 2
            {1.361, -1.704, -0.390}  // WP 3
            
        };
    }

    static PortsList providedPorts()
    {
        return {
            OutputPort<int>("selected_index", "L'index du meilleur point de reprise")
        };
    }

    // --- GESTION DU THREAD (Comme ton exemple) ---

    NodeStatus onStart() override {
        isRunning_ = true;
        execSuccess_ = false;
        // On lance le calcul dans un thread séparé
        exec_thread_ = std::make_shared<std::thread>(std::bind(&SelectNearestWaypoint::findNearest, this));
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        if (isRunning_) {
            return NodeStatus::RUNNING;
        }
        if (exec_thread_->joinable()) exec_thread_->join();
        
        return execSuccess_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

    void onHalted() override {
        isRunning_ = false;
        if (exec_thread_ && exec_thread_->joinable()) exec_thread_->join();
    }

private:

    void findNearest() {
        // Sécurité : Attendre que TF soit dispo
        if (!waitForTransform("map", "base_link")) {
            execSuccess_ = false;
            isRunning_ = false;
            return;
        }

        try {
            // 1. Récupérer la position robot
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            double rx = t.transform.translation.x;
            double ry = t.transform.translation.y;
            double r_yaw = tf2::getYaw(t.transform.rotation);

            int best_index = -1;
            double best_score = std::numeric_limits<double>::max();

            // 2. Algorithme de sélection
            for (size_t i = 0; i < waypoints_.size(); ++i) {
                double wx = waypoints_[i].x;
                double wy = waypoints_[i].y;

                double dist = std::hypot(wx - rx, wy - ry);
                
                // Produit scalaire pour savoir si c'est devant ou derrière
                double dx = wx - rx;
                double dy = wy - ry;
                double local_x = dx * std::cos(r_yaw) + dy * std::sin(r_yaw);

                double score = dist;
                if (local_x < 0) score *= 2.5; // Pénalité si derrière

                if (score < best_score) {
                    best_score = score;
                    best_index = i;
                }
            }

            if (best_index != -1) {
                setOutput("selected_index", best_index);
                
                // Visualisation : Dessiner une sphère verte sur le point choisi
                drawMarker(waypoints_[best_index], best_index);
                
                RCLCPP_INFO(node_->get_logger(), "Reprise patrouille au point %d (Dist: %.2fm)", best_index, best_score);
                execSuccess_ = true;
            } else {
                execSuccess_ = false;
            }

        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF Error: %s", ex.what());
            execSuccess_ = false;
        }

        isRunning_ = false;
    }

    bool waitForTransform(const std::string& target, const std::string& source) {
        // Petite boucle d'attente sécurisée
        for (int i = 0; i < 10; i++) {
            if (!isRunning_) return false;
            if (tf_buffer_->canTransform(target, source, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
                return true;
            }
        }
        return false;
    }

    void drawMarker(const Wayepoint& wp, int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "patrol_selection";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = wp.x;
        marker.pose.position.y = wp.y;
        marker.pose.position.z = 0.5;
        marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 0.5;
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Vert
        marker_pub_->publish(marker);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::vector<Wayepoint> waypoints_;
    
    // Threading variables
    std::shared_ptr<std::thread> exec_thread_;
    std::atomic<bool> isRunning_;
    std::atomic<bool> execSuccess_;

    // TF variables
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};