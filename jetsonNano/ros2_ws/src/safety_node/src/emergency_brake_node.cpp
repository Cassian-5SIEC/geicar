#include "rclcpp/rclcpp.hpp"
// Messages d'interfaces
#include <interfaces/msg/control.hpp> 
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/emergency_stop_request.hpp"
// Messages standard ROS
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp" // Pour le point transformé
// Outils de transformation NÉCESSAIRES
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Permet tf_buffer_.transform()

#include <functional> 
#include <cmath> 
#include <chrono> 
#include <string> 

using namespace std::placeholders;

// Nom du Frame sur lequel toutes les décisions doivent être basées
const std::string TARGET_FRAME = "base_link"; 
const int SAFETY_DISTANCE_DEFAULT = 500; // 500 mm (0.5 m)

class EmergencyBrakeNode : public rclcpp::Node
{
public:
    EmergencyBrakeNode() : Node("emergency_brake_node"), 
                           tf_buffer_(this->get_clock()),
                           tf_listener_(tf_buffer_) // Initialisation du Listener TF2
    {
        // --- DÉCLARATION DES PARAMÈTRES (Sécurité et Dimensions du Véhicule)
        this->declare_parameter("safety_distance_mm", SAFETY_DISTANCE_DEFAULT);
        // Offsets calculés (0.83m pour l'avant, -0.66m pour l'arrière)
        this->declare_parameter("L_OFFSET_FRONT", 0.83);   
        this->declare_parameter("L_OFFSET_REAR", -0.66);  
        this->declare_parameter("W_ROBOT", 0.523);        
        this->declare_parameter("LIDAR_FRAME_ID", "rplidar_link"); 

        // 1. DÉFINITION DU PUBLISHER
        stop_request_publisher_ = this->create_publisher<interfaces::msg::EmergencyStopRequest>(
            "emergency_stop_request", 10);

        // 2. DÉFINITION DU SUBSCRIBER ULTRASONS (Logique inchangée)
        us_subscriber_ = this->create_subscription<interfaces::msg::Ultrasonic>(
            "us_data", 10, std::bind(&EmergencyBrakeNode::us_callback, this, _1));

        // 3. DÉFINITION DU SUBSCRIBER LIDAR
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&EmergencyBrakeNode::scan_callback, this, _1));

        // 4. DÉFINITION DU TIMER DE FUSION (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&EmergencyBrakeNode::publish_fused_decision, this)
        );

        // INITIALISATION DES SEUILS après la déclaration des paramètres
        update_safety_thresholds();

        RCLCPP_INFO(this->get_logger(), "Emergency Brake Node started. Listening on /us_data AND /scan.");
    }

private:
    // Déclarations des objets ROS
    rclcpp::Publisher<interfaces::msg::EmergencyStopRequest>::SharedPtr stop_request_publisher_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr us_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_; 
    rclcpp::TimerBase::SharedPtr timer_; 

    // Objets TF2 (Transformation)
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_; 

    // Variables d'état et de configuration
    bool us_threat_detected_ = false;
    bool lidar_threat_detected_ = false;
    // Les seuils totaux et la largeur sont désormais des variables membres
    double D_FRONT_THRESHOLD_; // Seuil d'arrêt total AVANT (m)
    double D_REAR_THRESHOLD_;  // Seuil d'arrêt total ARRIÈRE (m)
    double W_ROBOT_;           // Largeur du robot (m)
    
    // Utile pour le log
    float current_safety_distance_m_ = (float)SAFETY_DISTANCE_DEFAULT / 1000.0f; 


    // --- FONCTION UTILITAIRE : Mise à jour des seuils
    void update_safety_thresholds()
    {
        int safety_distance_mm;
        double L_OFFSET_FRONT, L_OFFSET_REAR;
        
        // Lire tous les paramètres
        this->get_parameter("safety_distance_mm", safety_distance_mm);
        this->get_parameter("L_OFFSET_FRONT", L_OFFSET_FRONT);
        this->get_parameter("L_OFFSET_REAR", L_OFFSET_REAR);
        this->get_parameter("W_ROBOT", W_ROBOT_);

        double safety_distance_m = (double)safety_distance_mm / 1000.0;
        current_safety_distance_m_ = safety_distance_m;
        
        // Calcul des seuils totaux (Offset + Marge de sécurité)
        D_FRONT_THRESHOLD_ = L_OFFSET_FRONT + safety_distance_m; 
        D_REAR_THRESHOLD_ = L_OFFSET_REAR - safety_distance_m;  
    }


    // --- CALLBACK LIDAR : LOGIQUE TF2 ET GÉOMÉTRIQUE ROBUSTE
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        update_safety_thresholds(); // Recalculer les seuils au cas où ils sont modifiés dynamiquement
        bool threat_detected = false;
        
        double half_width = W_ROBOT_ / 2.0;
        
        // --- BOUCLE D'ANALYSE DU LASER SCAN ---
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            
            // 1. VÉRIFICATION DE BASE (Ignorer l'infini, NaN, etc.)
            if (std::isinf(range) || std::isnan(range) || range < msg->range_min || range > msg->range_max) {
                continue; 
            }
            
            // Optimisation: Si la portée est en dehors des deux zones de danger, ignorer la transformation
            if (range > D_FRONT_THRESHOLD_ && std::abs(range) > std::abs(D_REAR_THRESHOLD_)) {
                continue;
            }

            // 2. CALCUL DU POINT DANS LE FRAME DU LIDAR
            double angle = msg->angle_min + i * msg->angle_increment;
            
            geometry_msgs::msg::PointStamped point_in_lidar;
            point_in_lidar.header.frame_id = msg->header.frame_id; // Frame du Lidar
            point_in_lidar.header.stamp = msg->header.stamp;
            
            // Coordonnées polaires -> cartésiennes (dans le frame du Lidar)
            point_in_lidar.point.x = range * std::cos(angle);
            point_in_lidar.point.y = range * std::sin(angle);
            point_in_lidar.point.z = 0.0; 

            // 3. TRANSFORMATION vers /base_link (le Frame de décision)
            geometry_msgs::msg::PointStamped point_in_base_link;
            try {
                tf_buffer_.transform(point_in_lidar, point_in_base_link, TARGET_FRAME);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                     "TF Exception: Could not transform point from %s to %s. Check static_transform_publisher.",
                                     point_in_lidar.header.frame_id.c_str(), TARGET_FRAME.c_str());
                continue; 
            }

            // 4. LOGIQUE DE FREINAGE BIDIRECTIONNELLE
            double x_obs = point_in_base_link.point.x;
            double y_obs = point_in_base_link.point.y;

            // Vérification de la largeur (Y)
            if (std::abs(y_obs) >= half_width) {
                continue; 
            }
            
            // Freinage AVANT (X positif)
            // L'obstacle est devant (x > 0) ET plus près que le seuil de 1.33m
            if (x_obs > 0.0 && x_obs < D_FRONT_THRESHOLD_) {
                RCLCPP_WARN(this->get_logger(), "LIDAR DANGER (Frontal) at X=%.2f m, Y=%.2f m", x_obs, y_obs);
                threat_detected = true;
                break;
            }

            // Freinage ARRIÈRE (X négatif)
            // L'obstacle est derrière (x < 0) ET plus près que le seuil de -1.16m
            if (x_obs < 0.0 && x_obs > D_REAR_THRESHOLD_) {
                RCLCPP_WARN(this->get_logger(), "LIDAR DANGER (Rear) at X=%.2f m, Y=%.2f m", x_obs, y_obs);
                threat_detected = true;
                break;
            }
        }
        
        lidar_threat_detected_ = threat_detected;
    }


    // --- CALLBACK ULTRASONS (Logique US inchangée)
    void us_callback(const interfaces::msg::Ultrasonic::SharedPtr msg)
    {
        int safety_distance_mm;
        this->get_parameter("safety_distance_mm", safety_distance_mm); 

        bool stop_avant_us = (msg->front_left <= safety_distance_mm) || 
                             (msg->front_center <= safety_distance_mm) ||
                             (msg->front_right <= safety_distance_mm);

        bool stop_arriere_us = (msg->rear_left <= safety_distance_mm) || 
                               (msg->rear_center <= safety_distance_mm) ||
                               (msg->rear_right <= safety_distance_mm);
        
        us_threat_detected_ = stop_avant_us || stop_arriere_us;
    }


    // --- FONCTION DE FUSION ET PUBLICATION PÉRIODIQUE (inchangée)
    void publish_fused_decision()
    {
        auto emergency_msg = interfaces::msg::EmergencyStopRequest();
        
        // Décision : Si l'un des capteurs voit un problème, on stoppe tout.
        bool full_stop_required = lidar_threat_detected_ || us_threat_detected_;

        emergency_msg.stop_avant = full_stop_required;
        emergency_msg.stop_arriere = full_stop_required;

        stop_request_publisher_->publish(emergency_msg);
        
        // Affichage pour le débogage
        if (full_stop_required) {
            RCLCPP_WARN(this->get_logger(), "FUSION STOP (Limit: %.2f m): Lidar: %s, US: %s.", 
                        current_safety_distance_m_, 
                        lidar_threat_detected_ ? "TRUE" : "FALSE", 
                        us_threat_detected_ ? "TRUE" : "FALSE");
        } 
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyBrakeNode>());
    rclcpp::shutdown();
    return 0;
}