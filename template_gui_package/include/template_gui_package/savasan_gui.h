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

private slots:
  void onStartKamikazeButtonClicked();
  void handleRosLog(const rosgraph_msgs::Log::ConstPtr &msg);
  void onAltitudeSliderChanged(int value);  // Slot for altitude slider
  void onSpeedSliderChanged(int value);

  void on_verticalSlider_valueChanged(int value);

  void on_verticalSlider_sliderMoved(int position);

private:
  Ui::SavasanGui *ui;
  ros::Subscriber log_subscriber;
  ros::ServiceClient speed_command_client;
};



#endif // SAVASAN_GUI_H
