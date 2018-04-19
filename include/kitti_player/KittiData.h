#ifndef KITTIDATA_H
#define KITTIDATA_H
#include <QString>
#include <QStringList>
#include <QFileInfoList>
#include <QVector>
#include <QtOpenGL>
#include <QMatrix4x4>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <kitti_player/Datatypes.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

enum VelodyneLayer {
    Layer16,
    Layer64
};

//Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

class KittiData
{
public:
    KittiData();
    KittiData(QString path);
    ~KittiData();

    void path(QString path) { path_ = path; }
    QString path() { return path_; }
    void set_sequence(QString str_seq);

    QString seq_path() { return path_+"sequences/"+str_seq_+"/"; }
    QString gt_path() { return path_+"poses/"; }
    QString gt_fname() { return gt_path()+str_seq_+".txt"; }
    QString calib_fname() { return seq_path()+"calib.txt"; }

    int velodyne_layer() { return velodyne_layer_; }
    void velodyne_layer(VelodyneLayer layer) { velodyne_layer_ = layer; }

    cv::Mat& left_image() { return left_image_; }
    cv::Mat& right_image() { return right_image_; }
    cv::Mat& left_color_image() { return left_color_image_; }
    cv::Mat& right_color_image() { return right_color_image_; }

    PointCloud& velodyne_data() { return velodyne_data_; }

    Eigen::Vector2i encoder_data() { return encoder_; }

    Eigen::Vector3d fog_data() { return fog_;}

    Eigen::Matrix4d imu_pose_data() { return imu_pose_;}

    Eigen::Matrix4d pose_data() { return pose_;}

    void set_left_image(int i) { left_image_ = cv::imread(get_abs_path(flist_left_image_, i).toStdString(),CV_LOAD_IMAGE_GRAYSCALE); }
    void set_right_image(int i) { right_image_ = cv::imread(get_abs_path(flist_right_image_, i).toStdString(),CV_LOAD_IMAGE_GRAYSCALE); }
    void set_left_color_image(int i) { left_color_image_ = cv::imread(get_abs_path(flist_left_color_image_, i).toStdString()); }
    void set_right_color_image(int i) { right_color_image_ = cv::imread(get_abs_path(flist_right_color_image_, i).toStdString()); }

    void set_velodyne(int i) { read_velodyne(get_abs_path(flist_velodyne_, i)); }

    void set_encoder(int i){ encoder_ = encoders_[i]; }

    void set_fog(int i) { fog_ = fogs_[i]; }

    void set_imu_pose(int i) { imu_pose_ = imu_poses_[i]; }

    void set_pose(int i) { pose_ = poses_[i]; }

    double get_time_diff(int i) { return times_[i+1]-times_[i]; }
    int data_length() { return times_.size(); }

    Matrix3x4& P0() { return P0_; }
    Matrix3x4& P1() { return P1_; }
    Matrix3x4& P2() { return P2_; }
    Matrix3x4& P3() { return P3_; }
    Matrix3x4& Tr() { return Tr_; }

    void read_velodyne(QString fname);

    static QImage cv_mat_to_qimage(cv::Mat &src);

private:
    QString path_;
    QString str_seq_;

    QString left_image_path_;
    QString right_image_path_;
    QString left_color_image_path_;
    QString right_color_image_path_;

    PointCloud velodyne_data_;

    QString velodyne_path_;
    QString gt_fname_;
    QString calib_fname_;

    QFileInfoList flist_left_image_;
    QFileInfoList flist_right_image_;
    QFileInfoList flist_left_color_image_;
    QFileInfoList flist_right_color_image_;
    QFileInfoList flist_velodyne_;

    VelodyneLayer velodyne_layer_;

    QVector<double> times_;
    std::vector<Eigen::Vector2i> encoders_;
    Eigen::Vector2i encoder_;
    std::vector<Eigen::Vector3d> fogs_;
    Eigen::Vector3d fog_;
    std::vector<Eigen::Matrix4d> imu_poses_;
    Eigen::Matrix4d imu_pose_;
    std::vector<Eigen::Matrix4d> poses_;
    Eigen::Matrix4d pose_;

    Matrix3x4 P0_, P1_, P2_, P3_;
    Matrix3x4 Tr_;


    // Current Data
    cv::Mat left_image_;
    cv::Mat right_image_;
    cv::Mat left_color_image_;
    cv::Mat right_color_image_;

    QString get_abs_path (QFileInfoList& flist, int i) { return flist.at(i).absoluteFilePath(); }

    QFileInfoList get_filelist(const QString path, const QString name_filter);
    void print_filelist(const QFileInfoList flist);

};

#endif // KITTIDATA_H
