// Minimal ROS2 bridge for Livox Mid-70 (gen-1 protocol).
// Links the local first-generation Livox-SDK static library and republishes
// kExtendCartesian packets (data_type 2, int32 mm) as sensor_msgs/PointCloud2.

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "livox_def.h"
#include "livox_sdk.h"

namespace
{

struct XYZI
{
    float x;
    float y;
    float z;
    float intensity;
};

std::mutex g_buf_mutex;
std::vector<XYZI> g_points;
std::atomic<bool> g_sampling{false};

typedef enum {
    kDeviceStateDisconnect = 0,
    kDeviceStateConnect = 1,
    kDeviceStateSampling = 2,
} DeviceState;

struct DeviceItem
{
    uint8_t handle;
    DeviceState state;
    DeviceInfo info;
};

DeviceItem g_devices[kMaxLidarCount];

void OnLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *)
{
    (void)handle;
    if (!data || data_num == 0)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(g_buf_mutex);
    if (data->data_type == kExtendCartesian)
    {
        const LivoxExtendRawPoint *pts =
            reinterpret_cast<const LivoxExtendRawPoint *>(data->data);
        for (uint32_t i = 0; i < data_num; ++i)
        {
            // Skip zero returns
            if (pts[i].x == 0 && pts[i].y == 0 && pts[i].z == 0)
            {
                continue;
            }
            XYZI p;
            p.x = pts[i].x / 1000.0f;
            p.y = pts[i].y / 1000.0f;
            p.z = pts[i].z / 1000.0f;
            p.intensity = static_cast<float>(pts[i].reflectivity);
            g_points.push_back(p);
        }
    }
    else if (data->data_type == kCartesian)
    {
        const LivoxRawPoint *pts =
            reinterpret_cast<const LivoxRawPoint *>(data->data);
        for (uint32_t i = 0; i < data_num; ++i)
        {
            if (pts[i].x == 0 && pts[i].y == 0 && pts[i].z == 0)
            {
                continue;
            }
            XYZI p;
            p.x = pts[i].x / 1000.0f;
            p.y = pts[i].y / 1000.0f;
            p.z = pts[i].z / 1000.0f;
            p.intensity = static_cast<float>(pts[i].reflectivity);
            g_points.push_back(p);
        }
    }
}

void OnSampleStarted(livox_status status, uint8_t handle, uint8_t response, void *)
{
    printf("[mid70_bridge] StartSampling ack: status=%d handle=%u response=%u\n",
           status, handle, response);
    if (status == kStatusSuccess && response == 0)
    {
        g_sampling.store(true);
    }
}

void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type)
{
    if (!info || info->handle >= kMaxLidarCount)
    {
        return;
    }
    uint8_t handle = info->handle;

    if (type == kEventConnect)
    {
        printf("[mid70_bridge] Lidar %s connected\n", info->broadcast_code);
        g_devices[handle].state = kDeviceStateConnect;
        g_devices[handle].info = *info;
    }
    else if (type == kEventDisconnect)
    {
        printf("[mid70_bridge] Lidar %s disconnected\n", info->broadcast_code);
        g_devices[handle].state = kDeviceStateDisconnect;
        g_sampling.store(false);
    }
    else if (type == kEventStateChange)
    {
        g_devices[handle].info = *info;
    }

    if (g_devices[handle].state == kDeviceStateConnect &&
        g_devices[handle].info.state == kLidarStateNormal)
    {
        printf("[mid70_bridge] Lidar normal, start sampling\n");
        LidarStartSampling(handle, OnSampleStarted, nullptr);
        g_devices[handle].state = kDeviceStateSampling;
    }
}

void OnDeviceBroadcast(const BroadcastDeviceInfo *info)
{
    if (!info || info->dev_type == kDeviceTypeHub)
    {
        return;
    }
    printf("[mid70_bridge] Broadcast from %s\n", info->broadcast_code);

    uint8_t handle = 0;
    if (AddLidarToConnect(info->broadcast_code, &handle) == kStatusSuccess)
    {
        SetDataCallback(handle, OnLidarData, nullptr);
        g_devices[handle].handle = handle;
        g_devices[handle].state = kDeviceStateDisconnect;
    }
}

}  // namespace

class Mid70BridgeNode : public rclcpp::Node
{
public:
    Mid70BridgeNode() : Node("livox_mid70_bridge")
    {
        frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
        topic_ = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
        publish_rate_hz_ = declare_parameter<double>("publish_rate", 10.0);

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, 10);

        auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&Mid70BridgeNode::publishCloud, this));

        RCLCPP_INFO(get_logger(), "publishing %s (frame %s) at %.1f Hz",
                    topic_.c_str(), frame_id_.c_str(), publish_rate_hz_);
    }

private:
    void publishCloud()
    {
        std::vector<XYZI> pts;
        {
            std::lock_guard<std::mutex> lock(g_buf_mutex);
            pts.swap(g_points);
        }
        if (pts.empty())
        {
            return;
        }

        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.height = 1;
        msg.width = static_cast<uint32_t>(pts.size());

        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(pts.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_i(msg, "intensity");
        for (const auto &p : pts)
        {
            *it_x = p.x;
            *it_y = p.y;
            *it_z = p.z;
            *it_i = p.intensity;
            ++it_x;
            ++it_y;
            ++it_z;
            ++it_i;
        }

        pub_->publish(msg);
    }

    std::string frame_id_;
    std::string topic_;
    double publish_rate_hz_{10.0};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    memset(g_devices, 0, sizeof(g_devices));

    if (!Init())
    {
        fprintf(stderr, "[mid70_bridge] Livox SDK Init failed\n");
        return 1;
    }
    SetBroadcastCallback(OnDeviceBroadcast);
    SetDeviceStateUpdateCallback(OnDeviceInfoChange);
    if (!Start())
    {
        fprintf(stderr, "[mid70_bridge] Livox SDK Start failed\n");
        Uninit();
        return 1;
    }
    printf("[mid70_bridge] Livox SDK started, waiting for broadcast...\n");

    auto node = std::make_shared<Mid70BridgeNode>();
    rclcpp::spin(node);

    Uninit();
    rclcpp::shutdown();
    return 0;
}
