#include <string>
#include "MainWindow.h"
#include "ui_MainWindow.h"
//#include "odomproblem.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow), kitti_data_()
{
    ui->setupUi(this);

    // QTimer initialization
    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(onTimer()));
    timer_->stop();

    speed_ =1.0;

    is_left_color_image_pub_ = false;
    is_right_color_image_pub_ = false;
}

void MainWindow::initialize()
{
    kitti_data_.path(data_path_);

    ui->pathLabel->setText(data_path_);

    // set initial velodyne layer
    ui->layerSelector64->setChecked(true);
    if(ui->layerSelector64->isChecked()) kitti_data_.velodyne_layer(Layer64);
    if(ui->layerSelector16->isChecked()) kitti_data_.velodyne_layer(Layer16);

    // set initial sequence to 00
    reset_sequence();
    last_time_ = 0;
}

void MainWindow::ros_init(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    private_nh.param("data_path", str_path_, std::string("/var/data/kitti/dataset/"));
    private_nh.param("left_topic", str_left_topic_, std::string("/kitti/left_image"));
    private_nh.param("right_topic", str_right_topic_, std::string("/kitti/right_image"));
    private_nh.param("left_color_topic", str_left_color_topic_, std::string("/kitti/left_color_image"));
    private_nh.param("right_color_topic", str_right_color_topic_, std::string("/kitti/right_color_image"));
    private_nh.param("velodyne_topic", str_velodyne_topic_, std::string("/kitti/velodyne_points"));
    private_nh.param("imu_topic", str_imu_topic_, std::string("/kitti/imu"));

    private_nh.param("left_image_pub", is_left_image_pub_, true);
    private_nh.param("right_image_pub", is_right_image_pub_, false);
    private_nh.param("left_color_image_pub", is_left_color_image_pub_, true);
    private_nh.param("right_color_image_pub", is_right_color_image_pub_, false);
    private_nh.param("velodyne_pub", is_velodyne_pub_, false);
    private_nh.param("imu_pub", is_imu_pub_, false);
    private_nh.param("pose_pub", is_pose_pub_, true);

    cout << "left_color: " << is_left_color_image_pub_ << endl;

    data_path_ = QString::fromStdString(str_path_);

    initialize();

    this->nh_ = node;

    it_ = new image_transport::ImageTransport(nh_);
    left_img_pub_ = it_->advertiseCamera(str_left_topic_, 10);
    right_img_pub_ = it_->advertiseCamera(str_right_topic_, 10);
    left_color_img_pub_ = it_->advertiseCamera(str_left_color_topic_, 10);
    right_color_img_pub_ = it_->advertiseCamera(str_right_color_topic_, 10);

    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(str_velodyne_topic_, 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(str_imu_topic_, 10);

    f_ = boost::bind(&MainWindow::dynamic_parameter_callback, this, _1, _2);
    server_.setCallback(f_);

    spin_thread_ = std::thread(&MainWindow::spinner,this);

}

void MainWindow::dynamic_parameter_callback(kitti_player::kitti_playerConfig &config, uint32_t level)
{
    speed_ = config.speed;

    ui->startButton->setText("play");
}

MainWindow::~MainWindow()
{
    ros::shutdown();
    spin_thread_.join();
    delete ui;
}

void MainWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void MainWindow::reset_sequence()
{
    ui->lineEdit->setText("Sequence " + ui->comboBox->currentText());

    str_seq_ = ui->comboBox->currentText();
    kitti_data_.set_sequence(str_seq_);

    index_manager.init();
    imu_index_manager.init();

    // For slider bar
    ui->dataProgress->setMaximum(kitti_data_.data_length());
}

void MainWindow::load_data()
{
    if(index_manager.index() >= kitti_data_.data_length()) return;

    if(ui->layerSelector64->isChecked()) kitti_data_.velodyne_layer(Layer64);
    if(ui->layerSelector16->isChecked()) kitti_data_.velodyne_layer(Layer16);

    if(is_imu_pub_) {
        while( kitti_data_.get_imu_time(imu_index_manager.index())<kitti_data_.get_time(index_manager.index()) )
        {
            kitti_data_.set_imu(imu_index_manager.index());
            bool is_pub = publish_imu(imu_pub_, kitti_data_.imu_data());
            if(is_pub)imu_index_manager.inc();
        }
    }

    if(is_left_image_pub_) {
        kitti_data_.set_left_image(index_manager.index());
        publish_image(left_img_pub_, kitti_data_.left_image(), kitti_data_.P0());
    }

    if(is_right_image_pub_) {
        kitti_data_.set_right_image(index_manager.index());
        publish_image(right_img_pub_, kitti_data_.right_image(), kitti_data_.P1());
    }

    if(is_left_color_image_pub_) {
        kitti_data_.set_left_color_image(index_manager.index());
        publish_image(left_color_img_pub_, kitti_data_.left_color_image(), kitti_data_.P2());
    }
    if(is_right_color_image_pub_) {
        kitti_data_.set_right_color_image(index_manager.index());
        publish_image(right_color_img_pub_, kitti_data_.right_color_image(), kitti_data_.P3());
    }

    if(is_velodyne_pub_) {
        kitti_data_.set_velodyne(index_manager.index());
        publish_velodyne(pc_pub_, kitti_data_.velodyne_data());
    }



    // progress slider
    ui->dataProgress->setValue(index_manager.index());

    // For visualization
    QPixmap vis_image;


    QImage left_qimage;

    if(is_left_color_image_pub_)
        left_qimage = KittiData::cv_mat_to_qimage(kitti_data_.left_color_image());
    else if(is_right_color_image_pub_)
        left_qimage = KittiData::cv_mat_to_qimage(kitti_data_.right_color_image());
    else if(is_left_image_pub_)
        left_qimage = KittiData::cv_mat_to_qimage(kitti_data_.left_image());
    else
        left_qimage = KittiData::cv_mat_to_qimage(kitti_data_.right_image());

    vis_image.convertFromImage(left_qimage);

    ui->imageLabel->setPixmap(vis_image.scaledToWidth(ui->imageLabel->width()));


//  publish tfs
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,4>(0,0) = kitti_data_.Tr();
    Eigen::Affine3d eigen_affine_Tr(T);
    tf::transformEigenToTF(eigen_affine_Tr,transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/kitti/World", "/kitti/Velodyne"));

    if(is_pose_pub_){
    kitti_data_.set_pose(index_manager.index());
    Eigen::Affine3d eigen_affine_pose(kitti_data_.pose_data());
    tf::transformEigenToTF(eigen_affine_pose,transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/kitti/World", "/kitti/Current"));
    }

    // increase index
    index_manager.inc();
//    imu_index_manager.inc();

}

void MainWindow::publish_image(image_transport::CameraPublisher& img_pub, cv::Mat& img, Matrix3x4 P)
{
    cv_bridge::CvImage cv_image;
    cv_image.header.seq = index_manager.index();
    cv_image.header.stamp = ros::Time(kitti_data_.get_time(index_manager.index()));
    cv_image.header.frame_id = "kitti";
    if(img.type() == CV_8UC1)
        cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    else if(img.type() == CV_8UC3)
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;

    cv_image.image = img;

    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo());
    ci->header.frame_id = "kitti";
    ci->header.stamp = cv_image.header.stamp;
//    Eigen::Matrix<NumType, 4, 3> P_mat = P.transpose();
    double* middleP = P.data();
    for(int iter=0; iter<12; iter++)ci->P[iter] = middleP[iter];

    img_pub.publish(cv_image.toImageMsg(),ci);
}

void MainWindow::publish_image(image_transport::Publisher& img_pub, cv::Mat& img)
{
    cv_bridge::CvImage cv_image;
    cv_image.header.seq = index_manager.index();
    cv_image.header.stamp = ros::Time(kitti_data_.get_time(imu_index_manager.index()));
//    cv_image.header.stamp = sync_time_;
    cv_image.header.frame_id = "kitti";
    if(img.type() == CV_8UC1)
        cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    else if(img.type() == CV_8UC3)
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    else if(img.type() == CV_32F)
        cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    cv_image.image = img;

    img_pub.publish(cv_image.toImageMsg());
}


void MainWindow::publish_velodyne(ros::Publisher& pc_pub, PointCloud& pc)
{
    sensor_msgs::PointCloud2 out_pc;
    pcl::toROSMsg(pc, out_pc);

    out_pc.header.seq = index_manager.index();
    out_pc.header.stamp = ros::Time::now();
    out_pc.header.frame_id = "velodyne";
    pc_pub_.publish(out_pc);
}

bool MainWindow::publish_imu(ros::Publisher& imu_pub, Vector6d imu_data)
{
    if(ros::Time::now().toSec()-last_time_<0.01)
    {
    return false;
    }
    else{    
    last_time_ = ros::Time::now().toSec();
    sensor_msgs::Imu out_imu;
    out_imu.linear_acceleration.x = imu_data[0];
    out_imu.linear_acceleration.y = imu_data[1];
    out_imu.linear_acceleration.z = imu_data[2];
    out_imu.angular_velocity.x = imu_data[3];
    out_imu.angular_velocity.y = imu_data[4];
    out_imu.angular_velocity.z = imu_data[5];


    out_imu.header.seq = imu_index_manager.index();
    out_imu.header.stamp = ros::Time(kitti_data_.get_imu_time(imu_index_manager.index()));
    out_imu.header.frame_id = "imu";
    imu_pub_.publish(out_imu);
    return true;
    }
}

void MainWindow::set_pixmap()
{
////    cerr << "[MainWindow]\t Called set_pixmap()" << endl;
//    cv::Mat image = camlidar_calib_->frame();
//    QImage qimage;

//    switch (image.type()) {
//    case CV_8UC1:
//      qimage = QImage(image.data, image.cols, image.rows, static_cast<int> (image.step), QImage::Format_Grayscale8);
//      break;
//    case CV_8UC3:
//      qimage = QImage(image.data, image.cols, image.rows, static_cast<int> (image.step), QImage::Format_RGB888);
//      qimage = qimage.rgbSwapped();
//    default:
//      break;
//    }

//    image_ = QPixmap::fromImage(qimage);
//    ui->imageLabel->setPixmap(image_.scaledToWidth(ui->imageLabel->width()));
}

void MainWindow::onTimer()
{
    load_data();

    if(!ui->startButton->text().compare("play"))  timer_->stop();
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    reset_sequence();
}

void MainWindow::on_layerSelector64_clicked()
{
    kitti_data_.velodyne_layer(Layer64);
}

void MainWindow::on_layerSelector16_clicked()
{
    kitti_data_.velodyne_layer(Layer16);
}

void MainWindow::on_startButton_clicked()
{

    if(!ui->startButton->text().compare("play")) {
        ui->startButton->setText("stop");
        timer_->start(100);
        load_data();
    }
    else if(!ui->startButton->text().compare("stop")) {
        ui->startButton->setText("play");
        timer_->stop();
    }

}

void MainWindow::on_stepButton_clicked()
{
    load_data();
}
