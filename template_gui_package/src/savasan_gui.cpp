#include "savasan_gui.h"
#include "ui_savasan_gui.h"
#include <QDebug>
#include <QPalette>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <QtConcurrent/QtConcurrent>
#include <rosgraph_msgs/Log.h>
#include <mavros_msgs/CommandInt.h>
#include <QDialog>
#include <ros/package.h>
#include <QDir>
#include <QStringList>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <vector>



SavasanGui::SavasanGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::SavasanGui)
{
  ui->setupUi(this);
  ui->online_led->setText("  Offline");


  QPalette palette = ui->online_led->palette();
  palette.setColor(QPalette::WindowText, Qt::red);  // You can change Qt::red to any color you want
  ui->online_led->setPalette(palette);

  // Connect the button to the slot
  connect(ui->pushButton_3, &QPushButton::clicked, this, &SavasanGui::onStartKamikazeButtonClicked);

  connect(ui->startGpsTracking, &QPushButton::clicked, this, &SavasanGui::on_startGpsTracking_clicked);
  connect(ui->stopGpsTracking, &QPushButton::clicked, this, &SavasanGui::on_stopGpsTracking_clicked);
  connect(ui->startYolo, &QPushButton::clicked, this, &SavasanGui::on_startYolo_clicked);
  connect(ui->stopYolo, &QPushButton::clicked, this, &SavasanGui::on_stopYolo_clicked);
  connect(ui->startCompetition, &QPushButton::clicked, this, &SavasanGui::on_startCompetition_clicked);
  connect(ui->stopCompetition, &QPushButton::clicked, this, &SavasanGui::on_stopCompetition_clicked);
  connect(ui->startRecording, &QPushButton::clicked, this, &SavasanGui::on_startRecording_clicked);
  connect(ui->stopRecording, &QPushButton::clicked, this, &SavasanGui::on_stopRecording_clicked);
  connect(ui->UsePID, &QPushButton::clicked, this, &SavasanGui::on_UsePID_stateChanged);

  populateMissionComboBox();

  int recording_started = 0;
  int yolo_started = 0;
  int tracking_started = 0;

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
  ros::param::set("azimuth_angle", azimut);
  ros::param::set("dive_angle", diveAngle);
  ros::param::set("circle_offset", offset);
  ros::param::set("approach_distance", approachDistance);

  // Optional: Log the parameters to the console for debugging
  ROS_INFO("Set ROS parameters: azimut_degree=%d, dive_angle=%d, offset=%d, approach_distance=%d",
           azimut, diveAngle, offset, approachDistance);

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/start_kamikaze");


  QtConcurrent::run([client]() mutable {
    std_srvs::Trigger srv;
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

  if (QString::fromStdString(msg->name) == "/kamikaze_node") {
        ui->kamikaze_logs->append(logMessage);
        ui->kamikaze_logs->moveCursor(QTextCursor::End);
    }
  else if (QString::fromStdString(msg->name) == "/yolov8_node") {
        ui->tracker_logger->append(logMessage);
        ui->tracker_logger->moveCursor(QTextCursor::End);
  }
  else if(QString::fromStdString(msg->name) == "/comm_node"){
        ui->tracker_logger->append(logMessage);
        ui->tracker_logger->moveCursor(QTextCursor::End);
        ui->kamikaze_logs->append(logMessage);
        ui->kamikaze_logs->moveCursor(QTextCursor::End);
  }
  else {
    ui->general_log->append(logMessage);
    ui->general_log->moveCursor(QTextCursor::End);
  }
}


void sendcommandint(int command, int param1, int param2, int param3, int param4,int x,int y,int z){
    ros::NodeHandle nh2;
    ros::ServiceClient client2 = nh2.serviceClient<mavros_msgs::CommandInt>("/mavros/cmd/command_int");

      mavros_msgs::CommandInt srv;
      srv.request.broadcast = 0;
      srv.request.frame=3;
      srv.request.command = command; // MAV_CMD_DO_CHANGE_SPEED (use the appropriate integer command ID)
      srv.request.current = 0;
      srv.request.autocontinue = 0;
      srv.request.param1 = param1;    // Speed type (0 = airspeed, 1 = groundspeed, 2 = climb speed, 3 = descent speed)
      srv.request.param2 = param2;  // Speed value (integer)
      srv.request.param3 = param3;    // Throttle (-1 means no change)
      srv.request.param4 = param4;     // Unused
      srv.request.x = x;
      srv.request.y = y;
      srv.request.z = z;


      if (client2.call(srv)) {
            ROS_INFO("command sent successfully.");
        } else {
            ROS_ERROR("Failed to send command.");
        };
}


void SavasanGui::on_startGpsTracking_clicked()
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/start_navigation");

  std_srvs::Trigger srv;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Navigation node started successfully: %s", srv.response.message.c_str());
    } else {
      ROS_WARN("Failed to start Navigation node: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Service call to Start Navigation failed.");
  }}


void SavasanGui::on_stopGpsTracking_clicked()
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/stop_navigation");

  std_srvs::Trigger srv;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Navigation node stopped successfully: %s", srv.response.message.c_str());
    } else {
      ROS_WARN("Failed to stop Navigation node: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Service call to stop Navigation failed.");
  }}


void SavasanGui::on_startYolo_clicked()
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/start_yolov8_tracking");

  std_srvs::Trigger srv;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("YOLO node started successfully: %s", srv.response.message.c_str());
    } else {
      ROS_WARN("Failed to start YOLO node: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Service call to Start YOLO failed.");
  }}

void SavasanGui::on_stopYolo_clicked()
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/stop_yolov8_tracking");

  std_srvs::Trigger srv;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("YOLO node stopped successfully: %s", srv.response.message.c_str());
    } else {
      ROS_WARN("Failed to stop YOLO node: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Service call to Stop YOLO failed.");
  }}

void SavasanGui::on_startCompetition_clicked()
{


  ros::param::set("savasan_gui_node/competition_online", true);
  ui->online_led->setText("  Online");

  QPalette palette = ui->online_led->palette();
  palette.setColor(QPalette::WindowText, Qt::green);  // You can change Qt::red to any color you want
  ui->online_led->setPalette(palette);
  ROS_INFO("Competition started.");
}

void SavasanGui::on_stopCompetition_clicked()
{
  ros::param::set("savasan_gui_node/competition_online", false);
  ui->online_led->setText("  Offline");

  QPalette palette = ui->online_led->palette();
  palette.setColor(QPalette::WindowText, Qt::red);  // You can change Qt::red to any color you want
  ui->online_led->setPalette(palette);
  ROS_INFO("competition stopped.");
}

void SavasanGui::on_UsePID_stateChanged(int arg1)
{
  ros::param::set("savasan_gui_node/usePID", arg1);
  if(arg1){
      ROS_INFO("PID control is online");
  }
  else {
      ROS_INFO("PID control is offline");
  }

}

void SavasanGui::on_startRecording_clicked()
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("image_proccessor/start_recording");

  std_srvs::Trigger srv;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Camera recording started successfully: %s", srv.response.message.c_str());
    } else {
      ROS_WARN("Failed to start camera recording: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Service call to start camera recording failed.");
  }}


void SavasanGui::on_stopRecording_clicked()
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("image_proccessor/stop_recording");

  std_srvs::Trigger srv;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Camera recording Stopped successfully: %s", srv.response.message.c_str());
    } else {
      ROS_WARN("Failed to stop camera recording: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Service call to stop camera recording failed.");
  }}

void SavasanGui::on_spinBox_valueChanged(int arg1)
{
  ros::param::set("/gps_navigator/target_id", arg1);
  ROS_INFO("Target Set.");
}

// Function to populate the combo box with mission files from a specific folder
void SavasanGui::populateMissionComboBox() {
    // Get the absolute path to the ROS package (replace "your_package_name" with the actual package name)
    std::string package_path = ros::package::getPath("savasan_general");

    // Set the relative path to the mission folder within that package
    QString missionFolderPath = QString::fromStdString(package_path) + "/missions/";

    // Clear any existing items
    ui->load_mission_combo->clear();

    // Get a list of .json files in the specified folder
    QDir dir(missionFolderPath);
    QStringList filters;
    filters << "*.json";  // Adjust filter as necessary
    dir.setNameFilters(filters);

    QFileInfoList fileList = dir.entryInfoList();
    foreach (const QFileInfo &fileInfo, fileList) {
        ui->load_mission_combo->addItem(fileInfo.fileName(), fileInfo.absoluteFilePath());
    }
}

void SavasanGui::on_load_mission_clicked()
{
    // Get the file path from the combo box
    QString json_file_path = ui->load_mission_combo->currentText();

    std::string package_path = ros::package::getPath("savasan_general");

    // Set the relative path to the mission folder within that package
    QString missionFolderPath = QString::fromStdString(package_path) + "/missions/" + json_file_path;
    qWarning() << missionFolderPath;



    // Open the JSON file
    QFile file(missionFolderPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Could not open JSON file.";
        return;
    }

    // Read the file content
    QByteArray file_content = file.readAll();
    file.close();

    // Parse the JSON content
    QJsonDocument json_doc = QJsonDocument::fromJson(file_content);
    if (!json_doc.isArray()) {
        qWarning() << "Invalid JSON format.";
        return;
    }

    QJsonArray json_array = json_doc.array();

    // Prepare a ROS service client to push the waypoints
    ros::NodeHandle nh;
    ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    mavros_msgs::WaypointPush waypoint_push_srv;

    // Convert JSON waypoints to mavros_msgs::Waypoint and add to the service request
    for (int i = 0; i < json_array.size(); ++i) {
        QJsonObject waypoint_obj = json_array[i].toObject();

        mavros_msgs::Waypoint wp;
        wp.frame = waypoint_obj["frame"].toInt();
        wp.command = waypoint_obj["command"].toInt();
        wp.is_current = waypoint_obj["is_current"].toBool();
        wp.autocontinue = waypoint_obj["autocontinue"].toBool();
        wp.param1 = waypoint_obj["param1"].toDouble();
        wp.param2 = waypoint_obj["param2"].toDouble();
        wp.param3 = waypoint_obj["param3"].toDouble();
        wp.param4 = waypoint_obj["param4"].toDouble();
        wp.x_lat = waypoint_obj["x_lat"].toDouble();
        wp.y_long = waypoint_obj["y_long"].toDouble();
        wp.z_alt = waypoint_obj["z_alt"].toDouble();

        waypoint_push_srv.request.waypoints.push_back(wp);
    }

    // Call the service to upload the mission
    if (waypoint_push_client.call(waypoint_push_srv)) {
        if (waypoint_push_srv.response.success) {
            qDebug() << "Mission uploaded successfully!";
        } else {
            qWarning() << "Failed to upload the mission.";
        }
    } else {
        qWarning() << "Service call failed.";
    }
}
