#include "mainwindow.h"
#include "weightdialog.h"

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

	skeleton_gi = std::make_shared<ArcGraphicsItem>(&monos.wf->nodes, &monos.wf->arcList, &monos.data->lines);
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
	if(bothChainsDone && !mergeDone) {
		time_label->setText(QString(" -merging- "));
	} else if(mergeDone) {
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
	if(!monos.data->isMonotone) {return;}

	while(!bothChainsDone) {
		on_actionEventStep_triggered();
		time_changed();
	}

	on_actionResize_triggered();
	time_changed();
}

void MainWindow::on_actionFinishComputation_triggered() {
	if(!monos.config.isValid()) {return;}
	if(!monos.data->isMonotone) {return;}

	on_actionTimeForwardAfterChains_triggered();

	while(!mergeDone) {
		on_actionEventStep_triggered();
		time_changed();
	}

	time_changed();
	on_actionResize_triggered();
}

void MainWindow::on_actionEventStep_triggered() {
	if(!monos.config.isValid()) {return;}
	if(!monos.data->isMonotone) {return;}

	if(firstStart) {
		auto type = onLowerChain ? ChainType::LOWER : ChainType::UPPER;
		auto& chain     = monos.wf->getChain(type);
		auto& skeleton  = monos.wf->getSkeleton(type);

		if (!monos.wf->InitSkeletonQueue(chain,skeleton)) {
			LOG(WARNING) << "Error Init SkeletonQueue!";
		}
		firstStart = false;
	}

	if(!upperChainDone || !lowerChainDone) {
		auto type = onLowerChain ? ChainType::LOWER : ChainType::UPPER;
		auto& chain     = monos.wf->getChain(type);
		auto& skeleton  = monos.wf->getSkeleton(type);

		while(!monos.wf->eventTimes.empty() && !monos.wf->SingleDequeue(chain,skeleton));

		if(monos.wf->eventTimes.empty()) {
			if(onLowerChain) {
				lowerChainDone = true;
				LOG(INFO) << "Lower Chain Finished!";
			} else {
				upperChainDone = true;
				LOG(INFO) << "Upper Chain Finished!";
			}

			if(lowerChainDone && !upperChainDone) {
				monos.wf->FinishSkeleton(chain,skeleton);
				onLowerChain = false;
				type = onLowerChain ? ChainType::LOWER : ChainType::UPPER;
				chain     = monos.wf->getChain(type);
				skeleton  = monos.wf->getSkeleton(type);
				monos.wf->InitSkeletonQueue(chain,skeleton);
			}
		}
	} else if(lowerChainDone && upperChainDone && !bothChainsDone) {
		auto type = onLowerChain ? ChainType::LOWER : ChainType::UPPER;
		auto& chain     = monos.wf->getChain(type);
		auto& skeleton  = monos.wf->getSkeleton(type);
		monos.wf->FinishSkeleton(chain,skeleton);
		bothChainsDone = true;
//		monos.s->initMerge();
	}

	if(bothChainsDone && !mergeDone) {
		LOG(INFO) << " -------------------- || Merge-Step: " << ++merge_counter << "|| -------------------- " ;

//		if(!monos.s->SingleMergeStep()) {
//			monos.s->finishMerge();
//			mergeDone = true;
//			if(!monos.data->lines.empty()) {
//				monos.data->lines.clear();
//			}
//		}
	}

	on_actionResize_triggered();
	time_changed();
}


void MainWindow::simulation_has_finished() {
	if (did_finish) return;
	did_finish = true;
	updateVisibilities();
}

void MainWindow::dragEnterEvent(QDragEnterEvent *e) {
    if (e->mimeData()->hasUrls()) {
        e->acceptProposedAction();
    }
}

void MainWindow::dropEvent(QDropEvent *e) {
//    foreach (const QUrl &url, e->mimeData()->urls()) {
//        QString fileName = url.toLocalFile();
//        LOG(INFO) << "Dropped file:" << fileName.toStdString();
//        if (fileName.endsWith(".obj") || fileName.endsWith(".graphml")) {
//
//        	monos.reinitialize(fileName.toStdString(),true);
//
//        	input_gi = std::make_shared<InputGraphicsItem>(&monos.data->getBasicInput(), &monos.data->getPolygon(), &monos.data->getWeights(), &monos.data->getVertices());
//        	scene.addItem(input_gi.get());
//
//        	skeleton_gi = std::make_shared<ArcGraphicsItem>(&monos.wf->nodes, &monos.wf->arcList, &monos.data->lines);
//        	scene.addItem(skeleton_gi.get());
//
//        	on_actionResize_triggered();
//        }
//    }
}
