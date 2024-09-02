#ifndef SAVASAN_GUI_H
#define SAVASAN_GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

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

private:
  Ui::SavasanGui *ui;
  ros::Subscriber log_subscriber;
};

#endif // SAVASAN_GUI_H
