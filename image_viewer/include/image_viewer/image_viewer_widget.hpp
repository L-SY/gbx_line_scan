#ifndef IMAGE_VIEWER_IMAGE_VIEWER_WIDGET_HPP
#define IMAGE_VIEWER_IMAGE_VIEWER_WIDGET_HPP

#include <QWidget>
#include <QLabel>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLineEdit>
#include <QStatusBar>
#include <QSlider>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QTimer>
#include <memory>
#include <mutex>

class ImageViewerWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ImageViewerWidget(QWidget *parent = nullptr);
  ~ImageViewerWidget();

public slots:
  void updateImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private slots:
  void onZoomIn();
  void onZoomOut();
  void onZoomFit();
  void onZoom100();
  void onZoomChanged(double value);
  void onTopicChanged();
  void onRefreshTopics();
  void onImageReceived();

signals:
  void imageReceived();

private:
  void setupUI();
  void updateImageView();
  void setZoomFactor(double zoom);
  void fitToWindow();
  QStringList getAvailableTopics();

  // ROS2
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  
  // UI Components
  QVBoxLayout* main_layout_;
  QHBoxLayout* toolbar_layout_;
  QScrollArea* scroll_area_;
  QLabel* image_label_;
  
  QPushButton* zoom_in_btn_;
  QPushButton* zoom_out_btn_;
  QPushButton* zoom_fit_btn_;
  QPushButton* zoom_100_btn_;
  QPushButton* refresh_topics_btn_;
  
  QDoubleSpinBox* zoom_spinbox_;
  QComboBox* topic_combo_;
  QLineEdit* topic_edit_;
  
  QLabel* status_label_;
  
  QSlider* zoom_slider_;
  
  // Image data
  cv::Mat current_image_;
  std::mutex image_mutex_;
  
  // View state
  double zoom_factor_;
  bool fit_to_window_;
  
  QTimer* status_timer_;
  int frame_count_;
  QTimer* ros_timer_;
};

#endif // IMAGE_VIEWER_IMAGE_VIEWER_WIDGET_HPP

