#include <ros/ros.h>
#include <QtWidgets/QApplication>
#include "mainwindow.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auvsi2019");

  QApplication app(argc, argv);

  MainWindow w;
  w.showMaximized();
  ROS_INFO("AUVSI 2019 App started");
  w.show();
  
  return app.exec();
}
