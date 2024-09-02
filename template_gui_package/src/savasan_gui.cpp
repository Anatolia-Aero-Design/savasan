#include "savasan_gui.h"
#include "ui_savasan_gui.h"
#include <QDebug>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <QtConcurrent/QtConcurrent>
#include <rosgraph_msgs/Log.h>

SavasanGui::SavasanGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::SavasanGui)
{
  ui->setupUi(this);

  // Connect the button to the slot
  connect(ui->pushButton_3, &QPushButton::clicked, this, &SavasanGui::onStartKamikazeButtonClicked);

  // Subscribe to the ROS log messages
  ros::NodeHandle nh;
  log_subscriber = nh.subscribe("/rosout_agg", 1000, &SavasanGui::handleRosLog, this);
  // Create a timer to periodically call ros::spinOnce()
  QTimer *rosTimer = new QTimer(this);
  connect(rosTimer, &QTimer::timeout, this, []() {
      ros::spinOnce();
  });
  rosTimer->start(100); // Call ros::spinOnce() every 100 ms
}

SavasanGui::~SavasanGui()
{
  delete ui;
}

void SavasanGui::onStartKamikazeButtonClicked()
{
  // Read input values
  int azimut = ui->Azimut_input->text().toInt();
  int diveAngle = ui->dive_input->text().toInt();
  int offset = ui->Offset_input->text().toInt();
  int approachDistance = ui->approach_input->text().toInt();

  // Set ROS parameters
  ros::param::set("azimut_degree", azimut);
  ros::param::set("dive_angle", diveAngle);
  ros::param::set("offset", offset);
  ros::param::set("approach_distance", approachDistance);

  // Optional: Log the parameters to the console for debugging
  ROS_INFO("Set ROS parameters: azimut_degree=%d, dive_angle=%d, offset=%d, approach_distance=%d",
           azimut, diveAngle, offset, approachDistance);

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/start_kamikaze");

  QtConcurrent::run([client]() mutable {
    std_srvs::Empty srv;
    if (client.call(srv)) {
        ROS_INFO("Service /start_kamikaze called successfully.");
    } else {
        ROS_ERROR("Failed to call service /start_kamikaze.");
    }
  });
}

void SavasanGui::handleRosLog(const rosgraph_msgs::Log::ConstPtr &msg)
{
  QString timestamp = QString::number(msg->header.stamp.toNSec());

  QString color;
      switch (msg->level) {
          case rosgraph_msgs::Log::DEBUG:
              color = "gray";  // Debug messages in gray
              break;
          case rosgraph_msgs::Log::INFO:
              color = "green";  // Info messages in green
              break;
          case rosgraph_msgs::Log::WARN:
              color = "orange";  // Warning messages in orange
              break;
          case rosgraph_msgs::Log::ERROR:
              color = "red";  // Error messages in red
              break;
          case rosgraph_msgs::Log::FATAL:
              color = "darkred";  // Fatal messages in dark red
              break;
          default:
              color = "black";  // Default to black for unknown levels
              break;
      }

  // Format the message
  QString logMessage = QString("<span style=\"color:%1\">[%2]: %3</span>")
                       .arg(color)
                       .arg(QString::fromStdString(msg->name))
                       .arg(QString::fromStdString(msg->msg));

  if (QString::fromStdString(msg->name) == "/autonom_maneuver_node") {
        ui->kamikaze_logs->append(logMessage);
        ui->kamikaze_logs->moveCursor(QTextCursor::End);
    }
  else {
    ui->general_log->append(logMessage);
    ui->general_log->moveCursor(QTextCursor::End);
  }
}

