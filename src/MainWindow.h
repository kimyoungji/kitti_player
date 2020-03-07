#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <thread>
#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <QTimer>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <kitti_player/kitti_playerConfig.h>

#include <kitti_player/KittiData.h>
#include <kitti_player/Datatypes.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen_conversions/eigen_msg.h>

//#include <irp_sen_msgs/encoder.h>
//#include <irp_sen_msgs/fog_3axis.h>

using namespace std;

namespace Ui {
class MainWindow;
}

class UniqueId
{
public:
  UniqueId():unique_id(0)
  {

  }
  int index() { return unique_id; }
  void init() { unique_id = 0; }
  void inc() { ++unique_id; }
private:
  int unique_id;
};

static UniqueId index_manager;

static UniqueId imu_index_manager;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void ros_init(ros::NodeHandle node, ros::NodeHandle private_nh);

    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event);

private slots:
    void onTimer();
    void on_comboBox_currentIndexChanged(int index);
    void on_layerSelector16_clicked();
    void on_layerSelector64_clicked();
    void on_startButton_clicked();
    void on_stepButton_clicked();

private:
    Ui::MainWindow *ui;

    QTimer *timer_;

    QString data_path_;
    QString str_seq_;
    KittiData kitti_data_;

    int delay_ms_;
    double speed_;
    double last_time_;

    ///Pointer to dynamic reconfigure service srv_
    std::thread spin_thread_;
    void spinner() { ros::spin(); }

    dynamic_reconfigure::Server<kitti_player::kitti_playerConfig> server_;
    dynamic_reconfigure::Server<kitti_player::kitti_playerConfig>::CallbackType f_;
    kitti_player::kitti_playerConfig config_;

    void dynamic_parameter_callback(kitti_player::kitti_playerConfig &config, uint32_t level);


    void initialize();
    void reset_sequence();
    void load_data();

    QPixmap image_;

    std::string str_path_;
    std::string str_left_topic_;
    std::string str_right_topic_;
    std::string str_left_color_topic_;
    std::string str_right_color_topic_;
    std::string str_velodyne_topic_;
    std::string str_imu_topic_;
    std::string str_posecov_topic_;

    bool is_left_image_pub_;
    bool is_right_image_pub_;
    bool is_left_color_image_pub_;
    bool is_right_color_image_pub_;
    bool is_velodyne_pub_;
    bool is_imu_pub_;
    bool is_pose_pub_;
    bool is_posecov_pub_;

    ros::NodeHandle nh_;
    ros::Publisher pc_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher posecov_pub_;

    image_transport::ImageTransport *it_;
    image_transport::CameraPublisher left_img_pub_;
    image_transport::CameraPublisher right_img_pub_;
    image_transport::CameraPublisher left_color_img_pub_;
    image_transport::CameraPublisher right_color_img_pub_;

    void publish_image(image_transport::CameraPublisher& img_pub, cv::Mat& img, Matrix3x4 P, ros::Time t);
    void publish_image(image_transport::Publisher& img_pub, cv::Mat& img, ros::Time t);
    void publish_velodyne(ros::Publisher& pc_pub, PointCloud& pc, ros::Time t);
    bool publish_imu(ros::Publisher& imu_pub, Vector6d imu);
    void publish_posecov(Eigen::Affine3d pose, ros::Time t);


private slots:
    void set_pixmap();
};

#endif // MAINWINDOW_H
