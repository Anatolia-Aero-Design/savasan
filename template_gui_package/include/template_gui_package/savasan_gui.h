#ifndef SAVASAN_GUI_H
#define SAVASAN_GUI_H

#include <QWidget>
#include <ros/ros.h>

namespace Ui {
class SavasanGui;
}

class SavasanGui : public QWidget
{
  Q_OBJECT

public:
  explicit SavasanGui(QWidget *parent = nullptr);
  ~SavasanGui();

private:
  Ui::SavasanGui *ui;
};

#endif // SAVASAN_GUI_H
