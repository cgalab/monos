#ifndef WEIGHTDIALOG_H
#define WEIGHTDIALOG_H

#include <QMainWindow>
#include <CGAL/Qt/DemosMainWindow.h>
#include <QGraphicsScene>
#include <QLabel>

#include <memory>

namespace Ui {
  class MainWindow;
  class WeightDialog;
}

class WeightDialog : public CGAL::Qt::DemosMainWindow {
    Q_OBJECT

  public:
    explicit WeightDialog(const std::string& title);
    ~WeightDialog();

  private slots:
    void showEvent(QShowEvent *);
    void mousePressEvent(QMouseEvent *event);

    void on_actionQuit_triggered() { close(); };

  private:
    std::string 					 	title;
    std::unique_ptr<Ui::WeightDialog>  	ui;
    QGraphicsScene 					 	scene;
};

#endif // WEIGHTDIALOG_H
