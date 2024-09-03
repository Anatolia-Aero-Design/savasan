#include <QApplication>
#include "savasan_gui.h"



int main(int argc, char *argv[])
{

  ros::init(argc, argv, "Savasan_GUI",ros::init_options::AnonymousName);
  QApplication a(argc, argv);

  SavasanGui w;

  // set the window title as the node name
  w.setWindowTitle(QString::fromStdString("Anatolia Aero Design Mission Controller"));


  w.show();


  return a.exec();
}
