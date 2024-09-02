#include "savasan_gui.h"
#include "ui_savasan_gui.h"
#include <QDebug>

SavasanGui::SavasanGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::SavasanGui)
{
  ui->setupUi(this);
}

SavasanGui::~SavasanGui()
{
  delete ui;
}
