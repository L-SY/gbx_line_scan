#include "image_viewer/image_viewer_widget.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLineEdit>
#include <QStatusBar>
#include <QSlider>
#include <QGroupBox>
#include <QMessageBox>
#include <QPalette>
#include <QPixmap>
#include <QImage>
#include <QFileDialog>
#include <QDateTime>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <QApplication>
#include <QTimer>
#include <functional>

ImageViewerWidget::ImageViewerWidget(QWidget *parent)
  : QWidget(parent),
    zoom_factor_(1.0),
    fit_to_window_(true),
    frame_count_(0)
{
  // Initialize ROS2 node (if not already initialized)
  // Note: rclcpp::init should be called in main(), not here
  // This assumes rclcpp is already initialized
  
  node_ = std::make_shared<rclcpp::Node>("image_viewer_widget");
  
  setupUI();
  
  // Timer to process ROS callbacks
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, this, [this]() {
    rclcpp::spin_some(node_);
  });
  ros_timer_->start(10); // Process ROS callbacks every 10ms
  
  // Timer to update status
  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, [this]() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!current_image_.empty()) {
      status_label_->setText(
        QString("Size: %1x%2 | Zoom: %3% | Frames: %4")
        .arg(current_image_.cols)
        .arg(current_image_.rows)
        .arg(zoom_factor_ * 100, 0, 'f', 1)
        .arg(frame_count_)
      );
    } else {
      status_label_->setText("No image received");
    }
  });
  status_timer_->start(500); // Update status every 500ms
  
  connect(this, &ImageViewerWidget::imageReceived, this, &ImageViewerWidget::onImageReceived);
  
  // Subscribe to default topic
  topic_edit_->setText("/image_stitched");
  onTopicChanged();
  
  onRefreshTopics();
}

ImageViewerWidget::~ImageViewerWidget()
{
  ros_timer_->stop();
  status_timer_->stop();
  
  // Reset subscriber (shared_ptr will handle cleanup)
  image_sub_.reset();
  
  // Don't shutdown rclcpp here as other nodes might be using it
}

void ImageViewerWidget::setupUI()
{
  main_layout_ = new QVBoxLayout(this);
  main_layout_->setContentsMargins(5, 5, 5, 5);
  main_layout_->setSpacing(5);
  
  // Toolbar
  QGroupBox* toolbar_group = new QGroupBox("Controls");
  toolbar_layout_ = new QHBoxLayout(toolbar_group);
  toolbar_layout_->setSpacing(5);
  
  // Topic selection
  QLabel* topic_label = new QLabel("Topic:");
  toolbar_layout_->addWidget(topic_label);
  
  topic_combo_ = new QComboBox();
  topic_combo_->setEditable(true);
  topic_combo_->setMinimumWidth(200);
  topic_combo_->lineEdit()->setPlaceholderText("/image_stitched");
  connect(topic_combo_, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
          this, &ImageViewerWidget::onTopicChanged);
  toolbar_layout_->addWidget(topic_combo_);
  
  topic_edit_ = topic_combo_->lineEdit();
  connect(topic_edit_, &QLineEdit::returnPressed, this, &ImageViewerWidget::onTopicChanged);
  
  refresh_topics_btn_ = new QPushButton("Refresh");
  refresh_topics_btn_->setMaximumWidth(80);
  connect(refresh_topics_btn_, &QPushButton::clicked, this, &ImageViewerWidget::onRefreshTopics);
  toolbar_layout_->addWidget(refresh_topics_btn_);
  
  toolbar_layout_->addSpacing(20);
  
  // Zoom controls
  zoom_out_btn_ = new QPushButton("-");
  zoom_out_btn_->setMaximumWidth(30);
  zoom_out_btn_->setToolTip("Zoom Out");
  connect(zoom_out_btn_, &QPushButton::clicked, this, &ImageViewerWidget::onZoomOut);
  toolbar_layout_->addWidget(zoom_out_btn_);
  
  zoom_slider_ = new QSlider(Qt::Horizontal);
  zoom_slider_->setRange(10, 500); // 10% to 500%
  zoom_slider_->setValue(100);
  zoom_slider_->setMaximumWidth(150);
  zoom_slider_->setToolTip("Zoom Level");
  connect(zoom_slider_, &QSlider::valueChanged, this, [this](int value) {
    setZoomFactor(value / 100.0);
  });
  toolbar_layout_->addWidget(zoom_slider_);
  
  zoom_in_btn_ = new QPushButton("+");
  zoom_in_btn_->setMaximumWidth(30);
  zoom_in_btn_->setToolTip("Zoom In");
  connect(zoom_in_btn_, &QPushButton::clicked, this, &ImageViewerWidget::onZoomIn);
  toolbar_layout_->addWidget(zoom_in_btn_);
  
  zoom_spinbox_ = new QDoubleSpinBox();
  zoom_spinbox_->setRange(0.1, 10.0);
  zoom_spinbox_->setValue(1.0);
  zoom_spinbox_->setSingleStep(0.1);
  zoom_spinbox_->setSuffix("%");
  zoom_spinbox_->setDecimals(1);
  zoom_spinbox_->setMaximumWidth(80);
  zoom_spinbox_->setToolTip("Zoom Factor");
  connect(zoom_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &ImageViewerWidget::onZoomChanged);
  toolbar_layout_->addWidget(zoom_spinbox_);
  
  zoom_fit_btn_ = new QPushButton("Fit");
  zoom_fit_btn_->setMaximumWidth(50);
  zoom_fit_btn_->setToolTip("Fit to Window");
  connect(zoom_fit_btn_, &QPushButton::clicked, this, &ImageViewerWidget::onZoomFit);
  toolbar_layout_->addWidget(zoom_fit_btn_);
  
  zoom_100_btn_ = new QPushButton("100%");
  zoom_100_btn_->setMaximumWidth(60);
  zoom_100_btn_->setToolTip("Zoom to 100%");
  connect(zoom_100_btn_, &QPushButton::clicked, this, &ImageViewerWidget::onZoom100);
  toolbar_layout_->addWidget(zoom_100_btn_);
  
  toolbar_layout_->addSpacing(20);
  
  // Save button
  save_image_btn_ = new QPushButton("Save");
  save_image_btn_->setMaximumWidth(80);
  save_image_btn_->setToolTip("Save current image to file");
  connect(save_image_btn_, &QPushButton::clicked, this, &ImageViewerWidget::onSaveImage);
  toolbar_layout_->addWidget(save_image_btn_);
  
  toolbar_layout_->addStretch();
  
  main_layout_->addWidget(toolbar_group);
  
  // Image display area
  scroll_area_ = new QScrollArea();
  scroll_area_->setWidgetResizable(false);
  scroll_area_->setBackgroundRole(QPalette::Dark);
  scroll_area_->setAlignment(Qt::AlignCenter);
  
  image_label_ = new QLabel();
  image_label_->setScaledContents(false);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setText("Waiting for image...");
  image_label_->setMinimumSize(640, 480);
  
  scroll_area_->setWidget(image_label_);
  main_layout_->addWidget(scroll_area_, 1);
  
  // Status bar
  status_label_ = new QLabel("Ready");
  status_label_->setMinimumHeight(25);
  status_label_->setStyleSheet("background-color: #f0f0f0; padding: 3px;");
  main_layout_->addWidget(status_label_);
  
  setWindowTitle("Image Viewer - Stitched Images");
  resize(1200, 800);
}

void ImageViewerWidget::updateImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
      try {
        cv_ptr = cv_bridge::toCvCopy(msg);
      } catch (cv_bridge::Exception& e2) {
        QMessageBox::warning(this, "Image Conversion Error", 
                           QString("Failed to convert image: %1").arg(e2.what()));
        return;
      }
    }
    
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      current_image_ = cv_ptr->image.clone();
      frame_count_++;
    }
    
    Q_EMIT imageReceived();
    
  } catch (const std::exception& e) {
    QMessageBox::critical(this, "Error", QString("Exception: %1").arg(e.what()));
  }
}

void ImageViewerWidget::onImageReceived()
{
  updateImageView();
}

void ImageViewerWidget::updateImageView()
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  
  if (current_image_.empty()) {
    image_label_->setText("No image");
    return;
  }
  
  cv::Mat display_image = current_image_;
  
  // Apply zoom if not fitting to window
  if (!fit_to_window_ && zoom_factor_ != 1.0) {
    int new_width = static_cast<int>(current_image_.cols * zoom_factor_);
    int new_height = static_cast<int>(current_image_.rows * zoom_factor_);
    cv::resize(current_image_, display_image, cv::Size(new_width, new_height), 
               cv::INTER_LINEAR);
  } else if (fit_to_window_) {
    // Fit to window
    QSize scroll_size = scroll_area_->viewport()->size();
    double scale_x = static_cast<double>(scroll_size.width()) / current_image_.cols;
    double scale_y = static_cast<double>(scroll_size.height()) / current_image_.rows;
    double scale = std::min(scale_x, scale_y);
    
    if (scale < 1.0) {
      int new_width = static_cast<int>(current_image_.cols * scale);
      int new_height = static_cast<int>(current_image_.rows * scale);
      cv::resize(current_image_, display_image, cv::Size(new_width, new_height), 
                 cv::INTER_AREA);
    }
  }
  
  // Convert to QImage
  QImage qimage;
  if (display_image.channels() == 1) {
    qimage = QImage(display_image.data, display_image.cols, display_image.rows,
                    display_image.step, QImage::Format_Grayscale8).copy();
  } else if (display_image.channels() == 3) {
    cv::Mat rgb;
    cv::cvtColor(display_image, rgb, cv::COLOR_BGR2RGB);
    qimage = QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
  }
  
  if (!qimage.isNull()) {
    QPixmap pixmap = QPixmap::fromImage(qimage);
    image_label_->setPixmap(pixmap);
    image_label_->adjustSize();
  }
}

void ImageViewerWidget::setZoomFactor(double zoom)
{
  zoom_factor_ = zoom;
  fit_to_window_ = false;
  zoom_spinbox_->blockSignals(true);
  zoom_spinbox_->setValue(zoom * 100.0);
  zoom_spinbox_->blockSignals(false);
  
  zoom_slider_->blockSignals(true);
  zoom_slider_->setValue(static_cast<int>(zoom * 100));
  zoom_slider_->blockSignals(false);
  
  updateImageView();
}

void ImageViewerWidget::onZoomIn()
{
  setZoomFactor(std::min(zoom_factor_ + 0.1, 10.0));
}

void ImageViewerWidget::onZoomOut()
{
  setZoomFactor(std::max(zoom_factor_ - 0.1, 0.1));
}

void ImageViewerWidget::onZoomFit()
{
  fit_to_window_ = true;
  updateImageView();
}

void ImageViewerWidget::onZoom100()
{
  setZoomFactor(1.0);
}

void ImageViewerWidget::onZoomChanged(double value)
{
  setZoomFactor(value / 100.0);
}

void ImageViewerWidget::onTopicChanged()
{
  QString topic = topic_edit_->text().trimmed();
  if (topic.isEmpty()) {
    return;
  }
  
  // Unsubscribe from previous topic
  image_sub_.reset();
  
  // Subscribe to new topic using standard ROS2 subscription
  try {
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic.toStdString(),
      10,  // QoS depth
      std::bind(&ImageViewerWidget::updateImage, this, std::placeholders::_1)
    );
    status_label_->setText(QString("Subscribed to: %1").arg(topic));
  } catch (const std::exception& e) {
    QMessageBox::warning(this, "Subscription Error",
                        QString("Failed to subscribe to topic: %1\n%2")
                        .arg(topic).arg(e.what()));
  }
}

QStringList ImageViewerWidget::getAvailableTopics()
{
  QStringList topics;
  try {
    auto topic_list = node_->get_topic_names_and_types();
    for (const auto& [name, types] : topic_list) {
      for (const auto& type : types) {
        if (type == "sensor_msgs/msg/Image") {
          topics << QString::fromStdString(name);
        }
      }
    }
    topics.sort();
  } catch (...) {
    // Ignore errors
  }
  return topics;
}

void ImageViewerWidget::onRefreshTopics()
{
  QString current = topic_combo_->currentText();
  topic_combo_->clear();
  
  QStringList topics = getAvailableTopics();
  for (const QString& topic : topics) {
    topic_combo_->addItem(topic);
  }
  
  if (!current.isEmpty() && topics.contains(current)) {
    topic_combo_->setCurrentText(current);
  }
}

void ImageViewerWidget::onSaveImage()
{
  // Copy image data while holding lock, then release lock before opening dialog
  cv::Mat image_to_save;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    
    if (current_image_.empty()) {
      QMessageBox::warning(this, "No Image", "No image available to save.");
      return;
    }
    
    // Clone the image so we can release the lock
    image_to_save = current_image_.clone();
  } // Lock is released here
  
  // Generate default filename with timestamp
  QDateTime now = QDateTime::currentDateTime();
  QString default_filename = QString("image_%1.png")
                             .arg(now.toString("yyyyMMdd_hhmmss"));
  
  // Open file dialog (now without holding the lock)
  QString filename = QFileDialog::getSaveFileName(
    this,
    "Save Image",
    default_filename,
    "PNG Images (*.png);;JPEG Images (*.jpg *.jpeg);;All Files (*.*)"
  );
  
  if (filename.isEmpty()) {
    return; // User cancelled
  }
  
  // Save the image
  try {
    bool success = cv::imwrite(filename.toStdString(), image_to_save);
    if (success) {
      status_label_->setText(QString("Image saved to: %1").arg(filename));
      QMessageBox::information(this, "Success", 
                               QString("Image saved successfully to:\n%1").arg(filename));
    } else {
      QMessageBox::critical(this, "Error", 
                          QString("Failed to save image to:\n%1\n\nPlease check file permissions and path.").arg(filename));
    }
  } catch (const cv::Exception& e) {
    QMessageBox::critical(this, "Error", 
                         QString("OpenCV exception while saving:\n%1").arg(e.what()));
  } catch (const std::exception& e) {
    QMessageBox::critical(this, "Error", 
                         QString("Exception while saving:\n%1").arg(e.what()));
  }
}

