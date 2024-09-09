#ifndef SAVASAN_GUI_H
#define SAVASAN_GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <mavros_msgs/CommandInt.h>

namespace Ui {
class SavasanGui;
}

class SavasanGui : public QWidget
{
  Q_OBJECT

public:
  explicit SavasanGui(QWidget *parent = nullptr);
  ~SavasanGui();

  void populateMissionComboBox();

  bool uploadMission(const std::string& json_file_path);

private slots:
  void onStartKamikazeButtonClicked();
  void handleRosLog(const rosgraph_msgs::Log::ConstPtr &msg);

  void on_startGpsTracking_clicked();

  void on_stopGpsTracking_clicked();

  void on_startYolo_clicked();

  void on_stopYolo_clicked();

  void on_startCompetition_clicked();

  void on_stopCompetition_clicked();

  void on_UsePID_stateChanged(int arg1);

  void on_startRecording_clicked();

  void on_stopRecording_clicked();

  void on_spinBox_valueChanged(int arg1);

  void on_load_mission_clicked();

private:
  Ui::SavasanGui *ui;
  ros::Subscriber log_subscriber;
  ros::ServiceClient speed_command_client;
};



#endif // SAVASAN_GUI_H
