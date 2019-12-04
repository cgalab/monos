/**
 *  Copyright 2018, 2019 Peter Palfraader
 *            2018, 2019 Günther Eder - geder@cs.sbg.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <CGAL/Qt/DemosMainWindow.h>
#include <QGraphicsScene>
#include <QLabel>

#include <QMimeData>

#include "ui_mainwindow.h"

#include "ArcGraphicsItem.h"
#include "InputGraphicsItem.h"
#include "Config.h"
#include "Monos.h"

class MainWindow : public CGAL::Qt::DemosMainWindow {
    Q_OBJECT

	enum class STATE : ul {STARTLOWER=0,LOWER,FINISHLOWER,STARTUPPER,UPPER,FINISHUPPER,INITMERGE,MERGE,FINISHED};


  public:
    explicit MainWindow(const std::string& title, Monos& _monos);
    ~MainWindow();
  private:
    int merge_counter = 0;
//    bool first_show_event = true;
//    bool did_finish = false;
//
//    bool onLowerChain = true;
//    bool firstStart = true, lowerChainDone = false, upperChainDone = false, bothChainsDone = false;
//    bool mergeDone = false;

    STATE state = STATE::STARTLOWER;

    bool isChainState() const {
    	return   state == STATE::STARTLOWER ||
    			 state == STATE::LOWER ||
				 state == STATE::FINISHLOWER ||
				 state == STATE::STARTUPPER ||
				 state == STATE::UPPER ||
				 state == STATE::FINISHUPPER;
    }

  private slots:
    void showEvent(QShowEvent *);
    void mousePressEvent(QMouseEvent *event);

    void on_actionQuit_triggered() { monos.write(); close(); };
    void on_actionVisToggleInput_triggered() { updateVisibilities(); };
    void on_actionVisToggleInputLabels_triggered() { updateVisibilities(); };
    void on_actionVisToggleArcs_triggered() { updateVisibilities(); };
    void on_actionVisToggleArcLabels_triggered() { updateVisibilities(); };
    void on_actionVisToggleNodeLabels_triggered() { updateVisibilities(); };
    void on_actionVisToggleInputEdgesLabels_triggered() { updateVisibilities(); }

	void on_actionResize_triggered();
    void on_actionToggleFullscreen_triggered();

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

    std::shared_ptr<InputGraphicsItem> input_gi;
    std::shared_ptr<ArcGraphicsItem>   skeleton_gi;

    void updateVisibilities();
    void update_time_label();
    void time_changed();
    void simulation_has_finished();
};


#endif // MAINWINDOW_H
