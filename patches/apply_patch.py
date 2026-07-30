#!/usr/bin/env python3
"""
Apply edits to a freshly-cloned ros2_orb_slam3 source tree.

Usage:
  apply_patch.py <src_root> <install_root>
Where:
  <src_root>     = /opt/orbslam3_ws/src/ros2_orb_slam3
  <install_root> = /opt/orbslam3_ws/install  (passed to colcon)

Edits applied:
  common.hpp / common.cpp / package.xml  — mono /slam/pose publisher
  CMakeLists.txt                         — geometry_msgs + message_filters + rgbd_node_cpp
  src/rgbd_example.cpp                   — new RGB-D ROS2 node (written, not patched)
  mono_driver_node.py                    — skip disk dataset read (live images only)
"""
from __future__ import annotations

import sys
from pathlib import Path


def patch(path: Path, edits: list[tuple[str, str]], description: str) -> None:
    text = path.read_text()
    for old, new in edits:
        if old not in text:
            raise SystemExit(
                f"[{description}] expected snippet not found in {path}:\n---\n{old!r}\n---"
            )
        text = text.replace(old, new, 1)
    path.write_text(text)
    print(f"[{description}] patched {path}")


def write_file(path: Path, content: str, description: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    print(f"[{description}] written {path}")


RGBD_NODE_CPP = """\
/*
 * ROS2 RGB-D ORB-SLAM3 node for splatograph-orbslam3.
 * Pattern from zang09/ORB_SLAM3_ROS2 (MIT licence).
 *
 * ROS2 params (--ros-args -p ...):
 *   voc_file_arg            path to ORB vocabulary (.txt or .bin)
 *   settings_file_path_arg  directory that contains <settings_name_arg>.yaml
 *   settings_name_arg       yaml basename without extension
 *   color_topic             raw colour image topic (default /camera/color/image_raw)
 *   depth_topic             raw depth image topic  (default /camera/depth/image_raw)
 *
 * Publishes geometry_msgs/PoseStamped on /slam/pose (Twc, frame_id="map").
 * Depth image must be 16-bit unsigned (mm); set DepthMapFactor: 1000.0 in the yaml.
 */
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include "System.h"

class RGBDNode : public rclcpp::Node
{
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

public:
    RGBDNode() : Node("rgbd_node_cpp")
    {
        this->declare_parameter("voc_file_arg", "");
        this->declare_parameter("settings_file_path_arg", "");
        this->declare_parameter("settings_name_arg", "");
        this->declare_parameter("color_topic", "/camera/color/image_raw");
        this->declare_parameter("depth_topic", "/camera/depth/image_raw");

        auto voc    = this->get_parameter("voc_file_arg").as_string();
        auto sdir   = this->get_parameter("settings_file_path_arg").as_string();
        auto sname  = this->get_parameter("settings_name_arg").as_string();
        auto ctopic = this->get_parameter("color_topic").as_string();
        auto dtopic = this->get_parameter("depth_topic").as_string();

        std::string cfg = sdir + sname + ".yaml";
        RCLCPP_INFO(this->get_logger(),
                    "vocab=%s  cfg=%s\\ncolor=%s  depth=%s",
                    voc.c_str(), cfg.c_str(), ctopic.c_str(), dtopic.c_str());

        pSLAM_ = new ORB_SLAM3::System(voc, cfg, ORB_SLAM3::System::RGBD, false);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);

        color_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, ctopic);
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, dtopic);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *color_sub_, *depth_sub_);
        sync_->registerCallback(&RGBDNode::GrabRGBD, this);

        RCLCPP_INFO(this->get_logger(), "RGB-D ORB-SLAM3 node ready");
    }

    ~RGBDNode()
    {
        if (pSLAM_) {
            pSLAM_->Shutdown();
            pSLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
            delete pSLAM_;
        }
    }

private:
    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB,
                  const sensor_msgs::msg::Image::SharedPtr msgD)
    {
        cv_bridge::CvImageConstPtr cv_rgb, cv_depth;
        try { cv_rgb   = cv_bridge::toCvShare(msgRGB);  }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge RGB: %s", e.what()); return; }
        try { cv_depth = cv_bridge::toCvShare(msgD); }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge depth: %s", e.what()); return; }

        double t = msgRGB->header.stamp.sec + msgRGB->header.stamp.nanosec * 1e-9;
        Sophus::SE3f Tcw = pSLAM_->TrackRGBD(cv_rgb->image, cv_depth->image, t);

        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f tr = Twc.translation();
        Eigen::Quaternionf q  = Twc.unit_quaternion();

        Eigen::Matrix3f B;
        B << 0.0f, 0.0f, 1.0f,
             -1.0f, 0.0f, 0.0f,
             0.0f, -1.0f, 0.0f;
        Eigen::Vector3f tr_ros = B * tr;
        Eigen::Matrix3f R_ros = B * q.toRotationMatrix() * B.transpose();
        Eigen::Quaternionf q_ros(R_ros);
        q_ros.normalize();

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = msgRGB->header.stamp;
        msg.header.frame_id = "map";
        msg.pose.position.x    = static_cast<double>(tr_ros.x());
        msg.pose.position.y    = static_cast<double>(tr_ros.y());
        msg.pose.position.z    = static_cast<double>(tr_ros.z());
        msg.pose.orientation.x = static_cast<double>(q_ros.x());
        msg.pose.orientation.y = static_cast<double>(q_ros.y());
        msg.pose.orientation.z = static_cast<double>(q_ros.z());
        msg.pose.orientation.w = static_cast<double>(q_ros.w());
        pose_pub_->publish(msg);
    }

    ORB_SLAM3::System* pSLAM_ = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RGBDNode>());
    rclcpp::shutdown();
    return 0;
}
"""


# Shared pose-publish + IMU-buffer helpers reused by the inertial/stereo nodes.
_POSE_PUB = """\
    void publishPose(const Sophus::SE3f& Tcw, const builtin_interfaces::msg::Time& stamp)
    {
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f tr = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        Eigen::Matrix3f B;
        B << 0.0f, 0.0f, 1.0f,
             -1.0f, 0.0f, 0.0f,
             0.0f, -1.0f, 0.0f;
        Eigen::Vector3f tr_ros = B * tr;
        Eigen::Matrix3f R_ros = B * q.toRotationMatrix() * B.transpose();
        Eigen::Quaternionf q_ros(R_ros);
        q_ros.normalize();

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.pose.position.x = static_cast<double>(tr_ros.x());
        msg.pose.position.y = static_cast<double>(tr_ros.y());
        msg.pose.position.z = static_cast<double>(tr_ros.z());
        msg.pose.orientation.x = static_cast<double>(q_ros.x());
        msg.pose.orientation.y = static_cast<double>(q_ros.y());
        msg.pose.orientation.z = static_cast<double>(q_ros.z());
        msg.pose.orientation.w = static_cast<double>(q_ros.w());
        pose_pub_->publish(msg);
    }
"""

# ── Orbbec Femto VIO: RGB-D + IMU (System::IMU_RGBD) ────────────────────────
RGBD_INERTIAL_NODE_CPP = """\
/* RGB-D-Inertial ORB-SLAM3 node (Orbbec Femto VIO: mono colour + depth + IMU).
 * Params: voc_file_arg, settings_file_path_arg, settings_name_arg,
 *         color_topic, depth_topic, imu_topic (default /camera/imu).
 * Publishes /slam/pose (Twc, frame_id="map"). Config needs IMU.* + Tbc. */
#include <iostream>
#include <queue>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include "System.h"
#include "ImuTypes.h"

class RGBDInertialNode : public rclcpp::Node
{
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
public:
    RGBDInertialNode() : Node("rgbd_inertial_node_cpp")
    {
        this->declare_parameter("voc_file_arg", "");
        this->declare_parameter("settings_file_path_arg", "");
        this->declare_parameter("settings_name_arg", "");
        this->declare_parameter("color_topic", "/camera/color/image_raw");
        this->declare_parameter("depth_topic", "/camera/depth/image_raw");
        this->declare_parameter("imu_topic", "/camera/imu");
        auto voc   = this->get_parameter("voc_file_arg").as_string();
        auto sdir  = this->get_parameter("settings_file_path_arg").as_string();
        auto sname = this->get_parameter("settings_name_arg").as_string();
        auto ctopic= this->get_parameter("color_topic").as_string();
        auto dtopic= this->get_parameter("depth_topic").as_string();
        auto itopic= this->get_parameter("imu_topic").as_string();
        std::string cfg = sdir + sname + ".yaml";
        RCLCPP_INFO(this->get_logger(), "RGBD-Inertial vocab=%s cfg=%s\\ncolor=%s depth=%s imu=%s",
                    voc.c_str(), cfg.c_str(), ctopic.c_str(), dtopic.c_str(), itopic.c_str());
        pSLAM_ = new ORB_SLAM3::System(voc, cfg, ORB_SLAM3::System::IMU_RGBD, false);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
        // Tracking-confidence signal (2026-07-30, splatograph consumer:
        // train_streaming.py's keyframe-admission scoring). Published on a
        // SEPARATE topic (not embedded in PoseStamped, to avoid a custom-msg
        // package) but ATOMICALLY -- same GrabRGBD call, always published
        // immediately after /slam/pose, every single track, unconditionally
        // -- so a consumer can pair them 1:1 by FIFO ARRIVAL ORDER rather
        // than by matching header stamps. This deliberately avoids the
        // ApproximateTime/independently-timestamped-topics sync-pairing bug
        // class this project already hit once for RGB-D color/depth (see
        // splatograph's CLAUDE.md HARD CONSTRAINT on hardware/exact sync).
        // Payload: Int32MultiArray data=[tracking_state, num_tracked_points].
        // tracking_state mirrors ORB_SLAM3::Tracking::eTrackingState:
        //   -1=SYSTEM_NOT_READY 0=NO_IMAGES_YET 1=NOT_INITIALIZED
        //    2=OK 3=RECENTLY_LOST 4=LOST 5=OK_KLT
        tracking_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/slam/tracking_status", 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            itopic, rclcpp::SensorDataQoS(),
            std::bind(&RGBDInertialNode::GrabImu, this, std::placeholders::_1));
        color_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, ctopic);
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, dtopic);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *color_sub_, *depth_sub_);
        sync_->registerCallback(&RGBDInertialNode::GrabRGBD, this);
        RCLCPP_INFO(this->get_logger(), "RGBD-Inertial ORB-SLAM3 node ready");
    }
    ~RGBDInertialNode() { if (pSLAM_) { pSLAM_->Shutdown(); delete pSLAM_; } }
private:
    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    { std::lock_guard<std::mutex> lk(imu_mtx_); imu_buf_.push(msg); }

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB,
                  const sensor_msgs::msg::Image::SharedPtr msgD)
    {
        cv_bridge::CvImageConstPtr cv_rgb, cv_depth;
        try { cv_rgb = cv_bridge::toCvShare(msgRGB); }
        catch (cv_bridge::Exception& e) { RCLCPP_ERROR(this->get_logger(), "rgb: %s", e.what()); return; }
        try { cv_depth = cv_bridge::toCvShare(msgD); }
        catch (cv_bridge::Exception& e) { RCLCPP_ERROR(this->get_logger(), "depth: %s", e.what()); return; }
        double t = msgRGB->header.stamp.sec + msgRGB->header.stamp.nanosec * 1e-9;
        std::vector<ORB_SLAM3::IMU::Point> vImu;
        {
            std::lock_guard<std::mutex> lk(imu_mtx_);
            while (!imu_buf_.empty()) {
                auto& m = imu_buf_.front();
                double ti = m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;
                if (ti > t) break;
                vImu.emplace_back(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z,
                                  m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z, ti);
                imu_buf_.pop();
            }
        }
        Sophus::SE3f Tcw = pSLAM_->TrackRGBD(cv_rgb->image, cv_depth->image, t, vImu);
        // Read tracking state/inlier count IMMEDIATELY after TrackRGBD, before
        // any other SLAM-internal state can advance, and publish it right
        // alongside the pose -- same callback, same thread, unconditionally.
        int trackingState = pSLAM_->GetTrackingState();
        int numTracked = static_cast<int>(pSLAM_->GetTrackedMapPoints().size());
        std_msgs::msg::Int32MultiArray statusMsg;
        statusMsg.data = {trackingState, numTracked};
        tracking_pub_->publish(statusMsg);
        publishPose(Tcw, msgRGB->header.stamp);
    }
__POSE_PUB__
    ORB_SLAM3::System* pSLAM_ = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr tracking_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_sub_, depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
    std::mutex imu_mtx_;
};
int main(int argc, char** argv)
{ rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<RGBDInertialNode>()); rclcpp::shutdown(); return 0; }
""".replace("__POSE_PUB__", _POSE_PUB)

# ── RealSense D435i stereo (System::STEREO; left/right IR) ──────────────────
STEREO_NODE_CPP = """\
/* Stereo ORB-SLAM3 node (RealSense D435i: left/right IR; emitter OFF).
 * Params: voc/settings*, left_topic (/camera/infra1/image_rect_raw),
 *         right_topic (/camera/infra2/image_rect_raw). Publishes /slam/pose. */
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include "System.h"

class StereoNode : public rclcpp::Node
{
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
public:
    StereoNode() : Node("stereo_node_cpp")
    {
        this->declare_parameter("voc_file_arg", "");
        this->declare_parameter("settings_file_path_arg", "");
        this->declare_parameter("settings_name_arg", "");
        this->declare_parameter("left_topic", "/camera/infra1/image_rect_raw");
        this->declare_parameter("right_topic", "/camera/infra2/image_rect_raw");
        auto voc   = this->get_parameter("voc_file_arg").as_string();
        auto sdir  = this->get_parameter("settings_file_path_arg").as_string();
        auto sname = this->get_parameter("settings_name_arg").as_string();
        auto ltopic= this->get_parameter("left_topic").as_string();
        auto rtopic= this->get_parameter("right_topic").as_string();
        std::string cfg = sdir + sname + ".yaml";
        RCLCPP_INFO(this->get_logger(), "Stereo vocab=%s cfg=%s\\nleft=%s right=%s",
                    voc.c_str(), cfg.c_str(), ltopic.c_str(), rtopic.c_str());
        pSLAM_ = new ORB_SLAM3::System(voc, cfg, ORB_SLAM3::System::STEREO, false);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, ltopic);
        right_sub_= std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, rtopic);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *left_sub_, *right_sub_);
        sync_->registerCallback(&StereoNode::GrabStereo, this);
        RCLCPP_INFO(this->get_logger(), "Stereo ORB-SLAM3 node ready");
    }
    ~StereoNode() { if (pSLAM_) { pSLAM_->Shutdown(); delete pSLAM_; } }
private:
    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgL,
                    const sensor_msgs::msg::Image::SharedPtr msgR)
    {
        cv_bridge::CvImageConstPtr cl, cr;
        try { cl = cv_bridge::toCvShare(msgL); } catch (cv_bridge::Exception& e) { RCLCPP_ERROR(this->get_logger(), "L: %s", e.what()); return; }
        try { cr = cv_bridge::toCvShare(msgR); } catch (cv_bridge::Exception& e) { RCLCPP_ERROR(this->get_logger(), "R: %s", e.what()); return; }
        double t = msgL->header.stamp.sec + msgL->header.stamp.nanosec * 1e-9;
        Sophus::SE3f Tcw = pSLAM_->TrackStereo(cl->image, cr->image, t);
        publishPose(Tcw, msgL->header.stamp);
    }
__POSE_PUB__
    ORB_SLAM3::System* pSLAM_ = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_, right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};
int main(int argc, char** argv)
{ rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<StereoNode>()); rclcpp::shutdown(); return 0; }
""".replace("__POSE_PUB__", _POSE_PUB)

# ── RealSense D435i VIO: stereo + IMU (System::IMU_STEREO) ──────────────────
STEREO_INERTIAL_NODE_CPP = """\
/* Stereo-Inertial ORB-SLAM3 node (RealSense D435i VIO: left/right IR + IMU).
 * Params: voc/settings*, left_topic, right_topic, imu_topic (/camera/imu).
 * Publishes /slam/pose. Config needs IMU.* + Tbc. */
#include <iostream>
#include <queue>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include "System.h"
#include "ImuTypes.h"

class StereoInertialNode : public rclcpp::Node
{
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
public:
    StereoInertialNode() : Node("stereo_inertial_node_cpp")
    {
        this->declare_parameter("voc_file_arg", "");
        this->declare_parameter("settings_file_path_arg", "");
        this->declare_parameter("settings_name_arg", "");
        this->declare_parameter("left_topic", "/camera/infra1/image_rect_raw");
        this->declare_parameter("right_topic", "/camera/infra2/image_rect_raw");
        this->declare_parameter("imu_topic", "/camera/imu");
        auto voc   = this->get_parameter("voc_file_arg").as_string();
        auto sdir  = this->get_parameter("settings_file_path_arg").as_string();
        auto sname = this->get_parameter("settings_name_arg").as_string();
        auto ltopic= this->get_parameter("left_topic").as_string();
        auto rtopic= this->get_parameter("right_topic").as_string();
        auto itopic= this->get_parameter("imu_topic").as_string();
        std::string cfg = sdir + sname + ".yaml";
        RCLCPP_INFO(this->get_logger(), "Stereo-Inertial vocab=%s cfg=%s\\nleft=%s right=%s imu=%s",
                    voc.c_str(), cfg.c_str(), ltopic.c_str(), rtopic.c_str(), itopic.c_str());
        pSLAM_ = new ORB_SLAM3::System(voc, cfg, ORB_SLAM3::System::IMU_STEREO, false);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            itopic, rclcpp::SensorDataQoS(),
            std::bind(&StereoInertialNode::GrabImu, this, std::placeholders::_1));
        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, ltopic);
        right_sub_= std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, rtopic);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *left_sub_, *right_sub_);
        sync_->registerCallback(&StereoInertialNode::GrabStereo, this);
        RCLCPP_INFO(this->get_logger(), "Stereo-Inertial ORB-SLAM3 node ready");
    }
    ~StereoInertialNode() { if (pSLAM_) { pSLAM_->Shutdown(); delete pSLAM_; } }
private:
    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    { std::lock_guard<std::mutex> lk(imu_mtx_); imu_buf_.push(msg); }

    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgL,
                    const sensor_msgs::msg::Image::SharedPtr msgR)
    {
        cv_bridge::CvImageConstPtr cl, cr;
        try { cl = cv_bridge::toCvShare(msgL); } catch (cv_bridge::Exception& e) { RCLCPP_ERROR(this->get_logger(), "L: %s", e.what()); return; }
        try { cr = cv_bridge::toCvShare(msgR); } catch (cv_bridge::Exception& e) { RCLCPP_ERROR(this->get_logger(), "R: %s", e.what()); return; }
        double t = msgL->header.stamp.sec + msgL->header.stamp.nanosec * 1e-9;
        std::vector<ORB_SLAM3::IMU::Point> vImu;
        {
            std::lock_guard<std::mutex> lk(imu_mtx_);
            while (!imu_buf_.empty()) {
                auto& m = imu_buf_.front();
                double ti = m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;
                if (ti > t) break;
                vImu.emplace_back(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z,
                                  m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z, ti);
                imu_buf_.pop();
            }
        }
        Sophus::SE3f Tcw = pSLAM_->TrackStereo(cl->image, cr->image, t, vImu);
        publishPose(Tcw, msgL->header.stamp);
    }
__POSE_PUB__
    ORB_SLAM3::System* pSLAM_ = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_, right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
    std::mutex imu_mtx_;
};
int main(int argc, char** argv)
{ rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<StereoInertialNode>()); rclcpp::shutdown(); return 0; }
""".replace("__POSE_PUB__", _POSE_PUB)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    src = Path(sys.argv[1])
    install = Path(sys.argv[2])

    hpp = src / "include" / "ros2_orb_slam3" / "common.hpp"
    cpp = src / "src" / "common.cpp"
    pkg = src / "package.xml"
    cmk = src / "CMakeLists.txt"

    patch(
        hpp,
        [
            (
                '#include "sensor_msgs/msg/image.hpp"\nusing std::placeholders::_1;',
                '#include "sensor_msgs/msg/image.hpp"\n#include "geometry_msgs/msg/pose_stamped.hpp"\nusing std::placeholders::_1;',
            ),
            (
                "        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;\n\n        //* ORB_SLAM3 related variables",
                "        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;\n\n        //* Pose publisher (Tcw.inverse() -> /slam/pose as geometry_msgs/PoseStamped)\n        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;\n\n        //* ORB_SLAM3 related variables",
            ),
        ],
        "common.hpp",
    )

    patch(
        cpp,
        [
            (
                "    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));\n\n    \n    RCLCPP_INFO(this->get_logger(), \"Waiting to finish handshake ......\");",
                "    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));\n\n    //* publish SLAM pose (Tcw.inverse() in world / \"map\" frame) on /slam/pose\n    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(\"/slam/pose\", 10);\n\n\n    RCLCPP_INFO(this->get_logger(), \"Waiting to finish handshake ......\");",
            ),
            (
                "    //* Perform all ORB-SLAM3 operations in Monocular mode\n    //! Pose with respect to the camera coordinate frame not the world coordinate frame\n    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); \n    \n    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3\n    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use\n\n}",
                "    //* Perform all ORB-SLAM3 operations in Monocular mode\n    //! Pose with respect to the camera coordinate frame not the world coordinate frame\n    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);\n\n    //* Publish Twc (camera->world) so the consumer can place the camera in a world frame.\n    //* The SLAM world frame is the monocular map origin, header.frame_id=\"map\".\n    Sophus::SE3f Twc = Tcw.inverse();\n    Eigen::Vector3f t = Twc.translation();\n    Eigen::Quaternionf q = Twc.unit_quaternion();\n\n    Eigen::Matrix3f B;\n    B << 0.0f, 0.0f, 1.0f,\n         -1.0f, 0.0f, 0.0f,\n         0.0f, -1.0f, 0.0f;\n    Eigen::Vector3f t_ros = B * t;\n    Eigen::Matrix3f R_ros = B * q.toRotationMatrix() * B.transpose();\n    Eigen::Quaternionf q_ros(R_ros);\n    q_ros.normalize();\n\n    geometry_msgs::msg::PoseStamped pose_msg;\n    pose_msg.header.stamp = msg.header.stamp; // image stamp, so a downstream consumer can sync to it\n    pose_msg.header.frame_id = \"map\";\n    pose_msg.pose.position.x = static_cast<double>(t_ros.x());\n    pose_msg.pose.position.y = static_cast<double>(t_ros.y());\n    pose_msg.pose.position.z = static_cast<double>(t_ros.z());\n    pose_msg.pose.orientation.x = static_cast<double>(q_ros.x());\n    pose_msg.pose.orientation.y = static_cast<double>(q_ros.y());\n    pose_msg.pose.orientation.z = static_cast<double>(q_ros.z());\n    pose_msg.pose.orientation.w = static_cast<double>(q_ros.w());\n    pose_publisher_->publish(pose_msg);\n}",
            ),
            (
                "    enablePangolinWindow = true; // Shows Pangolin window output\n    enableOpenCVWindow = true; // Shows OpenCV window output",
                "    enablePangolinWindow = false; // Shows Pangolin window output (PATCHED: no X11)\n    enableOpenCVWindow = false; // Shows OpenCV window output (PATCHED: no GUI)",
            ),
        ],
        "common.cpp",
    )

    # Patch mono_driver_node.py to skip the disk dataset read - the live
    # image adapter (rosbag-image-adapter.py) feeds images instead.
    py = src / "ros2_orb_slam3" / "mono_driver_node.py"
    if py.exists():
        patch(
            py,
            [
                (
                    "        # Read images from the chosen dataset, order them in ascending order and prepare timestep data as well\n        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = self.get_image_dataset_asl(self.image_sequence_dir, \"mav0\") \n\n        print(self.image_seq_dir)\n        print(len(self.imgz_seqz))",
                    "        # Read images from the chosen dataset, order them in ascending order and prepare timestep data as well\n        #* PATCHED for live ROS image feed (no EuRoC mav0 directory needed)\n        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = \"\", [], []\n        print('Skipped disk dataset read; live images will arrive on /mono_py_driver/img_msg')",
                ),
            ],
            "mono_driver_node.py",
        )
    else:
        print(f"NOTE: {py} not found - skip python driver patch")

    pkg_text = pkg.read_text()
    if "<depend>geometry_msgs</depend>" not in pkg_text and "<build_depend>geometry_msgs</build_depend>" not in pkg_text:
        patch(
            pkg,
            [
                (
                    "  <build_depend>cv_bridge</build_depend>\n  <build_depend>image_transport</build_depend>\n  <build_depend>Pangolin</build_depend>\n  <build_depend>boost_system</build_depend>\n  <build_depend>boost_serialization</build_depend>",
                    "  <build_depend>cv_bridge</build_depend>\n  <build_depend>image_transport</build_depend>\n  <build_depend>Pangolin</build_depend>\n  <build_depend>boost_system</build_depend>\n  <build_depend>geometry_msgs</build_depend>\n  <build_depend>message_filters</build_depend>\n  <build_depend>boost_serialization</build_depend>",
                ),
                (
                    "  <exec_depend>cv_bridge</exec_depend>\n  <exec_depend>image_transport</exec_depend>\n  <exec_depend>Pangolin</exec_depend>\n  <exec_depend>boost_system</exec_depend>\n  <exec_depend>boost_serialization</exec_depend>",
                    "  <exec_depend>cv_bridge</exec_depend>\n  <exec_depend>image_transport</exec_depend>\n  <exec_depend>Pangolin</exec_depend>\n  <exec_depend>boost_system</exec_depend>\n  <exec_depend>geometry_msgs</exec_depend>\n  <exec_depend>message_filters</exec_depend>\n  <exec_depend>boost_serialization</exec_depend>",
                ),
            ],
            "package.xml",
        )
    else:
        print("package.xml already has geometry_msgs - skipping (idempotent rerun)")

    patch(
        cmk,
        [
            # find_package: add geometry_msgs + message_filters before Pangolin
            (
                "find_package(Eigen3 3.3.0 REQUIRED) # Matched with Sophus\nfind_package(Pangolin REQUIRED)",
                "find_package(Eigen3 3.3.0 REQUIRED) # Matched with Sophus\nfind_package(geometry_msgs REQUIRED)\nfind_package(message_filters REQUIRED)\nfind_package(Pangolin REQUIRED)",
            ),
            # THIS_PACKAGE_INCLUDE_DEPENDS: add geometry_msgs + message_filters
            (
                "set(THIS_PACKAGE_INCLUDE_DEPENDS\n  rclcpp\n  rclpy\n  std_msgs\n  sensor_msgs\n  # your_custom_msg_interface\n  cv_bridge\n  image_transport\n  OpenCV\n  Eigen3\n  Pangolin\n)",
                "set(THIS_PACKAGE_INCLUDE_DEPENDS\n  rclcpp\n  rclpy\n  std_msgs\n  sensor_msgs\n  # your_custom_msg_interface\n  cv_bridge\n  image_transport\n  OpenCV\n  geometry_msgs\n  message_filters\n  Eigen3\n  Pangolin\n)",
            ),
            # Add rgbd_node_cpp executable after mono_node_cpp
            (
                "target_link_libraries(mono_node_cpp PUBLIC orb_slam3_lib) # Link a node with the internal shared library",
                "target_link_libraries(mono_node_cpp PUBLIC orb_slam3_lib) # Link a node with the internal shared library"
                + "".join(
                    f"\n\nadd_executable({node}\n  src/{src}\n)\n"
                    f"ament_target_dependencies({node}\n  PUBLIC ${{THIS_PACKAGE_INCLUDE_DEPENDS}}\n)\n"
                    f"target_link_libraries({node} PUBLIC orb_slam3_lib)"
                    for node, src in (
                        ("rgbd_node_cpp", "rgbd_example.cpp"),
                        ("rgbd_inertial_node_cpp", "rgbd_inertial_example.cpp"),
                        ("stereo_node_cpp", "stereo_example.cpp"),
                        ("stereo_inertial_node_cpp", "stereo_inertial_example.cpp"),
                    )
                ),
            ),
            # Add all new node executables to install(TARGETS...)
            (
                "install(TARGETS mono_node_cpp orb_slam3_lib DBoW2 g2o",
                "install(TARGETS mono_node_cpp rgbd_node_cpp rgbd_inertial_node_cpp "
                "stereo_node_cpp stereo_inertial_node_cpp orb_slam3_lib DBoW2 g2o",
            ),
        ],
        "CMakeLists.txt",
    )

    # Write the new node source files (RGB-D + Orbbec VIO + RealSense stereo/VIO)
    write_file(src / "src" / "rgbd_example.cpp", RGBD_NODE_CPP, "rgbd_example.cpp")
    write_file(src / "src" / "rgbd_inertial_example.cpp", RGBD_INERTIAL_NODE_CPP, "rgbd_inertial_example.cpp")
    write_file(src / "src" / "stereo_example.cpp", STEREO_NODE_CPP, "stereo_example.cpp")
    write_file(src / "src" / "stereo_inertial_example.cpp", STEREO_INERTIAL_NODE_CPP, "stereo_inertial_example.cpp")

    print("[rebuild] patches applied; deferring colcon build to caller")


if __name__ == "__main__":
    main()
