#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class MkArrBBPub {
public:
    MkArrBBPub(const std::string &name = "bbox3d", const std::string &frame_id = "rslidar", double rate = 10.0)
        : nh_(), rate_(rate), frame_id_(frame_id), life_time_(1.0 / rate) {
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>(name, 2);
    }

    void publish(const std::vector<std::vector<float>> &boxes_3d) {
        auto corners_3d_velos = boxes3d_to_corners3d_lidar(boxes_3d);
        
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < corners_3d_velos.size(); ++i) {
            const auto& corners_3d_velo = corners_3d_velos[i];
            
            // Create the cube marker
            visualization_msgs::Marker box_marker;
            _box_marker_setter(box_marker, corners_3d_velo, i);
            marker_array.markers.push_back(box_marker);
        }

        pub_.publish(marker_array);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    double life_time_;
    double rate_;
    std::string frame_id_;

    void _box_marker_setter(visualization_msgs::Marker &box_marker, const std::vector<std::vector<float>> &corners_3d_velo, size_t i) {
        box_marker.header.frame_id = frame_id_;
        box_marker.header.stamp = ros::Time::now();
        box_marker.ns = "box_marker";
        box_marker.id = i;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.action = visualization_msgs::Marker::ADD;
        box_marker.lifetime = ros::Duration(life_time_);

        // Set color (cyan)
        box_marker.color.r = 0.0;
        box_marker.color.g = 1.0;
        box_marker.color.b = 1.0;
        box_marker.color.a = 1.0;
        
        // Calculate the center and size of the cube
        float x_min = std::numeric_limits<float>::max(), y_min = std::numeric_limits<float>::max(), z_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::min(), y_max = std::numeric_limits<float>::min(), z_max = std::numeric_limits<float>::min();

        for (const auto& point : corners_3d_velo) {
            if (point[0] < x_min) x_min = point[0];
            if (point[0] > x_max) x_max = point[0];
            if (point[1] < y_min) y_min = point[1];
            if (point[1] > y_max) y_max = point[1];
            if (point[2] < z_min) z_min = point[2];
            if (point[2] > z_max) z_max = point[2];
        }

        box_marker.pose.position.x = (x_min + x_max) / 2.0;
        box_marker.pose.position.y = (y_min + y_max) / 2.0;
        box_marker.pose.position.z = (z_min + z_max) / 2.0;

        box_marker.scale.x = x_max - x_min;
        box_marker.scale.y = y_max - y_min;
        box_marker.scale.z = z_max - z_min;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "marker_array_bbox_publisher");

    MkArrBBPub mkArrBBPub("bbox3d", "rslidar", 10.0);

    std::vector<std::vector<float>> boxes_3d = {
        // Example box data: {center_x, center_y, center_z, length, width, height, rotation_y}
        // Add your box data here
    };

    ros::Rate rate(10.0);
    while (ros::ok()) {
        mkArrBBPub.publish(boxes_3d);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
