#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Eigen>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

const std::size_t ROBOT_STATE_SIZE = 3;
const std::size_t NUMBER_LANDMARKS = 12;
const double HUGE_COVARIANCE = 1e10;

class Slam : public rclcpp::Node
{
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // публикатор положения робота
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  // публикатор положений маяков
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr landmark_pub[NUMBER_LANDMARKS];
  
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  
  void on_odo(const nav_msgs::msg::Odometry& odom);
  void on_scan(const sensor_msgs::msg::LaserScan& scan);
  void publish_results(const std::string& frame, const rclcpp::Time& time);
  void predict(double dt);
  void advertize_landmark_publishers();
  // поиск координад маяков по скану
  void detect_landmarks(const sensor_msgs::msg::LaserScan& scan);
  void add_landmark(const sensor_msgs::msg::LaserScan& scan, std::size_t start, std::size_t finish);
  // поиск индекса маяка в векторе состояния подходящего для измерения, -1 - новый маяк
  int associate_measurement(const Eigen::Vector2d& landmark_measuriment);
  int add_landmark_to_state(int measurementIndex);
  void correct(int landmarkIndex, int measurementIndex);
  // публикуем результаты
  void publish_transform(const std_msgs::msg::Header& scan_header);

  void update_occupancy_grid(const sensor_msgs::msg::LaserScan& scan);
  int world_to_grid_x(double x);
  int world_to_grid_y(double y);

  double v = 0;
  double w = 0;

  // state vector
  // вектор состояния
  Eigen::VectorXd X;
  // system Jacobi
  // линеаризованная матрица системы
  Eigen::Matrix3d A;
  // covariation of system
  // матрица ковариации ошибок оценок
  Eigen::MatrixXd P;
  // covariation of measurement errors for each landmark
  // матрица ковариации ошибок измерения
  Eigen::Matrix2d Q;
  // covariation of system vulnerability for x y fi
  // матрица ковариации возмущений системы
  Eigen::Matrix3d R;

  rclcpp::Time last_time;

  std::size_t landmarks_found_quantity = 0;

  std::vector<Eigen::Vector2d> new_landmarks;
  std::vector<Eigen::Vector2d> new_landmarks_measurement;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  const std::string map_frame;

  double feature_rad;

  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  double grid_resolution_;  // meters per cell
  int grid_width_;         // cells
  int grid_height_;        // cells

public:
  Slam();
};
