#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <CGAL/Qt/DemosMainWindow.h>
#include <QGraphicsScene>
#include <QLabel>

#include "ArcGraphicsItem.h"
#include "InputGraphicsItem.h"
#include "Config.h"
#include "Monos.h"

#include "weightdialog.h"

class MainWindow : public CGAL::Qt::DemosMainWindow {
    Q_OBJECT

  public:
    explicit MainWindow(const std::string& title, Monos& _monos);
    ~MainWindow();
  private:
    bool first_show_event = true;
    bool did_finish = false;

    bool onLowerChain = true;
    bool lowerChainDone = false, upperChainDone = false, bothChainsDone = false;
    bool mergeDone = false;

  private slots:
    void showEvent(QShowEvent *);
    void mousePressEvent(QMouseEvent *event);

    void on_actionQuit_triggered() { monos.write(); close(); };
    void on_actionVisToggleInput_triggered() { updateVisibilities(); };
    void on_actionVisToggleInputLabels_triggered() { updateVisibilities(); };
    void on_actionVisToggleArcs_triggered() { updateVisibilities(); };

    void on_actionResize_triggered();
    void on_actionToggleFullscreen_triggered();

    void on_actionDefineWeight_triggered();
    void on_actionResetAll_triggered();

    void on_actionEventStep_triggered();
    void on_actionTimeForwardAfterChains_triggered();
    void on_actionFinishComputation_triggered();


  private:
    std::string title;
    std::unique_ptr<Ui::MainWindow> ui;
    QGraphicsScene scene;
    QLabel* time_label;
    NT drawing_time_offset_increment;

    Monos& monos;

    WeightDialog* weightDialog;

    std::shared_ptr<InputGraphicsItem> input_gi;
    std::shared_ptr<ArcGraphicsItem> skeleton_gi;

    void updateVisibilities();
    void update_time_label();
    void time_changed();
    void simulation_has_finished();
};


#endif // MAINWINDOW_H
