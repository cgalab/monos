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

#include "mainwindow.h"

#include "ui_mainwindow.h"

#include "BGLGraph.h"

MainWindow::MainWindow(const std::string& title, Monos& _monos) :
	CGAL::Qt::DemosMainWindow(),
	title(title),
	ui(new Ui::MainWindow),
	monos(_monos) {

	ui->setupUi(this);
	setWindowTitle(QString::fromStdString(title));
	scene.setItemIndexMethod(QGraphicsScene::NoIndex);
	ui->gV->setScene(&scene);
	ui->gV->setMouseTracking(true);
	ui->gV->scale(1, -1);
	ui->gV->setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
	ui->gV->setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
	setAcceptDrops(true);

	//addNavigation(ui->gV);

	/* add navigation */
	navigation = new MyGraphicsViewNavigation();
	ui->gV->viewport()->installEventFilter(navigation);
	ui->gV->installEventFilter(navigation);
	QObject::connect(navigation, SIGNAL(mouseCoordinates(QString)),
			xycoord, SLOT(setText(QString)));
	this->view = ui->gV;

	/* read input file */
	monos.readInput();
	/* general init. of monos */
	monos.init();

	input_gi = std::make_shared<InputGraphicsItem>(monos.getBasicInput());
	scene.addItem(input_gi.get());

	skeleton_gi = std::make_shared<ArcGraphicsItem>(&monos.wf->nodes, &monos.wf->arcList);
	scene.addItem(skeleton_gi.get());

	auto input_size = input_gi->boundingRect().size();
	auto size_avg = (input_size.width() + input_size.height() ) /2.0;

	time_label = new QLabel("", this);
	update_time_label();
	time_label->setAlignment(::Qt::AlignHCenter);
	time_label->setMinimumSize(time_label->sizeHint());
	ui->statusBar->addWidget(new QLabel("C-S-<right button: center; C-<right button>: drag; C-<left buttom>: select-zoom"), 1);
	ui->statusBar->addWidget(new QLabel(this), 1);
	ui->statusBar->addWidget(time_label, 0);
	ui->statusBar->addWidget(xycoord, 0);

	time_changed();
	on_actionResize_triggered();
	on_actionResize_triggered();
}

MainWindow::~MainWindow() {}

void MainWindow::updateVisibilities() {
	input_gi->setVisibleLabels(ui->actionVisToggleInputLabels->isChecked());
	input_gi->setVisibleEdgeLabels(ui->actionVisToggleInputEdgesLabels->isChecked());
	input_gi->setVisible(ui->actionVisToggleInput->isChecked());

	scene.removeItem(skeleton_gi.get());
	scene.addItem(skeleton_gi.get());
	skeleton_gi->setVisibleLabels(ui->actionVisToggleInputLabels->isChecked());
	skeleton_gi->setVisible(ui->actionVisToggleInput->isChecked());
    skeleton_gi->setVisible(ui->actionVisToggleArcs->isChecked());
	skeleton_gi->setVisibleArcLabels(ui->actionVisToggleArcLabels->isChecked());
	skeleton_gi->setVisibleNodeLabels(ui->actionVisToggleNodeLabels->isChecked());
}

void MainWindow::on_actionToggleFullscreen_triggered() {
	if (this->isFullScreen()) {
		this->showNormal();
	} else {
		this->showFullScreen();
	}
	on_actionResize_triggered();
}

void MainWindow::on_actionResize_triggered() {
	auto br = input_gi->boundingRect();
	//br |= skeleton_gi->boundingRect();

	ui->gV->setSceneRect(br);
	ui->gV->fitInView(br, Qt::KeepAspectRatio);
}


void MainWindow:: showEvent(QShowEvent *) {}

void MainWindow::mousePressEvent(QMouseEvent *event) {
	(void) event;
}

void MainWindow::update_time_label() {
	scene.update(scene.sceneRect());
	skeleton_gi->update(scene.sceneRect());
	if(state == STATE::MERGE) {
		time_label->setText(QString(" -merging- "));
	} else if(state == STATE::FINISHED) {
		time_label->setText(QString(" -finished- "));
	} else {
		auto t = CGAL::to_double( monos.wf->getTime() );
		time_label->setText(QString("t: %1 ").arg(t));
	}
}

void MainWindow::time_changed() {
	skeleton_gi->modelChanged();
	skeleton_gi->setBoundingRect(input_gi->boundingRect());
	update_time_label();
}


void MainWindow::on_actionTimeForwardAfterChains_triggered() {
	if(!monos.config.isValid()) {return;}

	while(isChainState()) {
		on_actionEventStep_triggered();
		time_changed();
	}

	on_actionResize_triggered();
	time_changed();
}

void MainWindow::on_actionFinishComputation_triggered() {
	if(!monos.config.isValid()) {return;}

	on_actionTimeForwardAfterChains_triggered();

	while(state != STATE::FINISHED) {
		on_actionEventStep_triggered();
		time_changed();
	}

	time_changed();
	on_actionResize_triggered();
}

void MainWindow::on_actionEventStep_triggered() {
	if(!monos.config.isValid()) {return;}

	switch(state) {
	case STATE::STARTLOWER:
		LOG(INFO) << "STARTLOWER";
		{
		auto& chain = monos.wf->getChain(ChainType::LOWER);
		monos.wf->InitSkeletonQueue(chain);
		}
		state = STATE::LOWER;
		break;
	case STATE::LOWER:
		LOG(INFO) << "LOWER";
		{
		auto& chain = monos.wf->getChain(ChainType::LOWER);
		if(!monos.wf->SingleDequeue(chain)) {
			state = STATE::FINISHLOWER;
		}
		}
		break;
	case STATE::FINISHLOWER:
		LOG(INFO) << "FINISHLOWER";
		{
		auto& chain = monos.wf->getChain(ChainType::LOWER);
		monos.wf->FinishSkeleton(chain);
		monos.wf->nextState();
		}
		state = STATE::STARTUPPER;
		break;
	case STATE::STARTUPPER:
		LOG(INFO) << "STARTUPPER";
		{
		auto& chain = monos.wf->getChain(ChainType::UPPER);
		monos.wf->InitSkeletonQueue(chain);
		}
		state = STATE::UPPER;
		break;
	case STATE::UPPER:
		LOG(INFO) << "UPPER";
		{
		auto& chain = monos.wf->getChain(ChainType::UPPER);
		monos.wf->SingleDequeue(chain);
		if(!monos.wf->SingleDequeue(chain)) {
			state = STATE::FINISHUPPER;
		}
		}
		break;
	case STATE::FINISHUPPER:
		LOG(INFO) << "FINISHUPPER";
		{
		auto& chain = monos.wf->getChain(ChainType::UPPER);
		monos.wf->FinishSkeleton(chain);
		monos.wf->nextState();
		}
		state = STATE::INITMERGE;
		break;
	case STATE::INITMERGE:
		LOG(INFO) << "INITMERGE";
		monos.s->initMerge();
		state = STATE::MERGE;
		break;
	case STATE::MERGE:
		LOG(INFO) << "MERGE";
		if(!monos.s->SingleMergeStep()) {
			monos.s->finishMerge();
			state = STATE::FINISHED;
		}
		break;
	case STATE::FINISHED:
		LOG(INFO) << "FINISHED";
		break;
	}


//	if(!upperChainDone || !lowerChainDone) {
//		auto type = onLowerChain ? ChainType::LOWER : ChainType::UPPER;
//		auto& chain     = monos.wf->getChain(type);
//
//		while(!monos.wf->eventTimes.empty() && !monos.wf->SingleDequeue(chain));
//
//		if(monos.wf->eventTimes.empty()) {
//			if(onLowerChain) {
//				lowerChainDone = true;
//				LOG(INFO) << "Lower Chain Finished!";
//			} else {
//				upperChainDone = true;
//				LOG(INFO) << "Upper Chain Finished!";
//			}
//
//			if(lowerChainDone && !upperChainDone) {
//				onLowerChain = false;
//				type = onLowerChain ? ChainType::LOWER : ChainType::UPPER;
//				chain     = monos.wf->getChain(type);
//				monos.wf->InitSkeletonQueue(chain);
//			}
//		}
//	} else if(lowerChainDone && upperChainDone && !bothChainsDone) {
//		auto type = onLowerChain ? ChainType::LOWER : ChainType::UPPER;
//		auto& chain     = monos.wf->getChain(type);
//		monos.wf->FinishSkeleton(chain);
//		bothChainsDone = true;
//		monos.s->initMerge();
//	}


	on_actionResize_triggered();
	time_changed();
}


void MainWindow::simulation_has_finished() {
	if (state == STATE::FINISHED) return;
	updateVisibilities();
}


