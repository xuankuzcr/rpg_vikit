/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 */

 #ifndef VIKIT_CAMERA_LOADER_H_
 #define VIKIT_CAMERA_LOADER_H_
 
 #include <string>
 #include <memory>
 #include <rclcpp/rclcpp.hpp>
 #include <vikit/abstract_camera.h>
 #include <vikit/atan_camera.h>
 #include <vikit/equidistant_camera.h>
 #include <vikit/omni_camera.h>
 #include <vikit/pinhole_camera.h>
 #include <vikit/polynomial_camera.h>
 
 namespace vk {
 namespace camera_loader {
 
 // Helper template function to get parameters in ROS2
 template<typename T>
 T get_parameter(const std::string& param_name, rclcpp::Node* node) {
   T value;
   if (!node->get_parameter(param_name, value)) {
     throw std::runtime_error("Failed to get parameter: " + param_name);
   }
   return value;
 }
 
 // Helper template function with default value
 template<typename T>
 T get_parameter(const std::string& param_name, const T& default_value, rclcpp::Node* node) {
   T value;
   if (!node->get_parameter(param_name, value)) {
     return default_value;
   }
   return value;
 }
 
 /// Load from ROS Namespace
 inline bool loadFromRosNs(const std::string &ns, vk::AbstractCamera *&cam, rclcpp::Node* node) {
   std::string cam_model = get_parameter<std::string>(ns + ".cam_model", "", node);
 
   if (cam_model == "Omni") {
     cam = new vk::OmniCamera(get_parameter<std::string>(ns + ".cam_calib_file", "", node));
   } else if (cam_model == "Pinhole") {
     cam = new vk::PinholeCamera(
         get_parameter<int>(ns + ".cam_width", node), 
         get_parameter<int>(ns + ".cam_height", node),
         get_parameter<double>(ns + ".scale", 1.0, node), 
         get_parameter<double>(ns + ".cam_fx", node),
         get_parameter<double>(ns + ".cam_fy", node), 
         get_parameter<double>(ns + ".cam_cx", node),
         get_parameter<double>(ns + ".cam_cy", node), 
         get_parameter<double>(ns + ".cam_d0", 0.0, node),
         get_parameter<double>(ns + ".cam_d1", 0.0, node),
         get_parameter<double>(ns + ".cam_d2", 0.0, node),
         get_parameter<double>(ns + ".cam_d3", 0.0, node));
   } else if (cam_model == "PinholeSimple") {
     cam = new vk::PinholeCamera(
         get_parameter<int>(ns + ".cam_width", node), 
         get_parameter<int>(ns + ".cam_height", node),
         get_parameter<double>(ns + ".scale", 1.0, node), 
         get_parameter<double>(ns + ".cam_fx", node),
         get_parameter<double>(ns + ".cam_fy", node), 
         get_parameter<double>(ns + ".cam_cx", node),
         get_parameter<double>(ns + ".cam_cy", node));
   } else if (cam_model == "EquidistantCamera") {
     cam = new vk::EquidistantCamera(
         get_parameter<int>(ns + ".cam_width", node), 
         get_parameter<int>(ns + ".cam_height", node),
         get_parameter<double>(ns + ".scale", 1.0, node), 
         get_parameter<double>(ns + ".cam_fx", node),
         get_parameter<double>(ns + ".cam_fy", node), 
         get_parameter<double>(ns + ".cam_cx", node),
         get_parameter<double>(ns + ".cam_cy", node), 
         get_parameter<double>(ns + ".k1", 0.0, node),
         get_parameter<double>(ns + ".k2", 0.0, node), 
         get_parameter<double>(ns + ".k3", 0.0, node),
         get_parameter<double>(ns + ".k4", 0.0, node));
   } else if (cam_model == "PolynomialCamera") {
     cam = new vk::PolynomialCamera(
         get_parameter<int>(ns + ".cam_width", node), 
         get_parameter<int>(ns + ".cam_height", node),
         // get_parameter<double>(ns+"/scale", 1.0, node),
         get_parameter<double>(ns + ".cam_fx", node), 
         get_parameter<double>(ns + ".cam_fy", node),
         get_parameter<double>(ns + ".cam_cx", node), 
         get_parameter<double>(ns + ".cam_cy", node),
         get_parameter<double>(ns + ".cam_skew", node), 
         get_parameter<double>(ns + ".k2", 0.0, node),
         get_parameter<double>(ns + ".k3", 0.0, node), 
         get_parameter<double>(ns + ".k4", 0.0, node),
         get_parameter<double>(ns + ".k5", 0.0, node), 
         get_parameter<double>(ns + ".k6", 0.0, node),
         get_parameter<double>(ns + ".k7", 0.0, node));
   } else if (cam_model == "ATAN") {
     cam = new vk::ATANCamera(
         get_parameter<int>(ns + ".cam_width", node), 
         get_parameter<int>(ns + ".cam_height", node),
         get_parameter<double>(ns + ".cam_fx", node), 
         get_parameter<double>(ns + ".cam_fy", node),
         get_parameter<double>(ns + ".cam_cx", node), 
         get_parameter<double>(ns + ".cam_cy", node),
         get_parameter<double>(ns + ".cam_d0", node));
   } else {
     cam = NULL;
     throw std::runtime_error("Camera model not supported.");
   }
   return true;
 }
 
 inline bool loadFromRosNs(const std::string &ns,
                    std::vector<vk::AbstractCamera *> &cam_list,
                    rclcpp::Node* node) {
   bool res = true;
   std::string cam_model = get_parameter<std::string>(ns + "/cam_model", node);
   int cam_num = get_parameter<int>(ns + "/cam_num", node);
   for (int i = 0; i < cam_num; i++) {
     std::string cam_ns = ns + "/cam_" + std::to_string(i);
     std::string cam_model = get_parameter<std::string>(cam_ns + "/cam_model", node);
     if (cam_model == "FishPoly") {
       cam_list.push_back(new vk::PolynomialCamera(
           get_parameter<int>(cam_ns + "/image_width", node),
           get_parameter<int>(cam_ns + "/image_height", node),
           // get_parameter<double>(cam_ns+"/scale", 1.0, node),
           get_parameter<double>(cam_ns + "/A11", node), // cam_fx
           get_parameter<double>(cam_ns + "/A22", node), // cam_fy
           get_parameter<double>(cam_ns + "/u0", node),  // cam_cx
           get_parameter<double>(cam_ns + "/v0", node),  // cam_cy
           get_parameter<double>(cam_ns + "/A12", node), // cam_skew
           get_parameter<double>(cam_ns + "/k2", 0.0, node),
           get_parameter<double>(cam_ns + "/k3", 0.0, node),
           get_parameter<double>(cam_ns + "/k4", 0.0, node),
           get_parameter<double>(cam_ns + "/k5", 0.0, node),
           get_parameter<double>(cam_ns + "/k6", 0.0, node),
           get_parameter<double>(cam_ns + "/k7", 0.0, node)));
     } else if (cam_model == "Pinhole") {
       cam_list.push_back(new vk::PinholeCamera(
           get_parameter<int>(ns + "/cam_width", node), 
           get_parameter<int>(ns + "/cam_height", node),
           get_parameter<double>(ns + "/scale", 1.0, node),
           get_parameter<double>(ns + "/cam_fx", node), 
           get_parameter<double>(ns + "/cam_fy", node),
           get_parameter<double>(ns + "/cam_cx", node), 
           get_parameter<double>(ns + "/cam_cy", node),
           get_parameter<double>(ns + "/cam_d0", 0.0, node),
           get_parameter<double>(ns + "/cam_d1", 0.0, node),
           get_parameter<double>(ns + "/cam_d2", 0.0, node),
           get_parameter<double>(ns + "/cam_d3", 0.0, node)));
     } else {
       // cam_list.clear();
       res = false;
     }
   }
 
   return res;
 }
 
 } // namespace camera_loader
 } // namespace vk
 
 #endif // VIKIT_CAMERA_LOADER_H_