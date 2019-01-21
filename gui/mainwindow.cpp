#include "mainwindow.h"
#include "weightdialog.h"

#include "ui_weightDialog.h"
#include "ui_mainwindow.h"

#include "gml/GMLGraph.h"

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
	//addNavigation(ui->gV);

	/* add navigation */
	navigation = new MyGraphicsViewNavigation();
	ui->gV->viewport()->installEventFilter(navigation);
	ui->gV->installEventFilter(navigation);
	QObject::connect(navigation, SIGNAL(mouseCoordinates(QString)),
			xycoord, SLOT(setText(QString)));
	this->view = ui->gV;

	/* general init. of monos */
	monos.init();

	/* start with lower chain, rest by steppint through */
	if(!monos.initSkeletonQueue(onLowerChain)) {
		LOG(WARNING) << "Error Init SkeletonQueue!";
	}

	//  GMLGraph graph;
	//  graph = GMLGraph::create_from_graphml(is);
	//s.add_graph(graph);
	//s.initialize();

	input_gi = std::make_shared<InputGraphicsItem>(&monos.data.getBasicInput());
	scene.addItem(input_gi.get());

	skeleton_gi = std::make_shared<ArcGraphicsItem>(&monos.wf.nodes, &monos.wf.arcList);
	scene.addItem(skeleton_gi.get());


	auto input_size = input_gi->boundingRect().size();
	auto size_avg = (input_size.width() + input_size.height() ) /2.0;
	//  s.wp.set_increment(size_avg/5000.);
	//  drawing_time_offset_increment = size_avg/5000.;

	//  kinetic_triangulation_gi = std::make_shared<KineticTriangulationGraphicsItem>(&s.get_kt(), size_avg/200.);
	//  scene.addItem(kinetic_triangulation_gi.get());

	time_label = new QLabel("", this);
	update_time_label();
	time_label->setAlignment(::Qt::AlignHCenter);
	time_label->setMinimumSize(time_label->sizeHint());
	ui->statusBar->addWidget(new QLabel("C-S-<right button: center; C-<right button>: drag; C-<left buttom>: select-zoom"), 1);
	ui->statusBar->addWidget(new QLabel(this), 1);
	ui->statusBar->addWidget(time_label, 0);
	ui->statusBar->addWidget(xycoord, 0);

	//  s.wp.do_initial_skips(skip_all, skip_to, NT(skip_until_time));
	time_changed();
}

MainWindow::~MainWindow()
{
}

void
MainWindow::updateVisibilities() {
	input_gi->setVisibleLabels(ui->actionVisToggleInputLabels->isChecked());
	input_gi->setVisible(ui->actionVisToggleInput->isChecked());

	scene.removeItem(skeleton_gi.get());
	scene.addItem(skeleton_gi.get());
	skeleton_gi->setVisibleLabels(ui->actionVisToggleInputLabels->isChecked());
	skeleton_gi->setVisible(ui->actionVisToggleInput->isChecked());
    skeleton_gi->setVisible(ui->actionVisToggleArcs->isChecked());

	/*
  triangulation_gi->setVisible(ui->actionVisToggleTriangulation->isChecked());
	 */
	//  kinetic_triangulation_gi->setVisible(ui->actionVisToggleKineticTriangulation->isChecked() ||
	//                                       ui->actionVisToggleWavefront->isChecked() ||
	//                                       ui->actionVisToggleArcs->isChecked());
	//  kinetic_triangulation_gi->setVisibleEdges(ui->actionVisToggleKineticTriangulation->isChecked());
	//  kinetic_triangulation_gi->setVisibleConstraints(ui->actionVisToggleWavefront->isChecked());
	//  kinetic_triangulation_gi->setVisibleLabels(ui->actionVisToggleKineticTriangulationLabels->isChecked());
	//  kinetic_triangulation_gi->setVisibleArcs(ui->actionVisToggleArcs->isChecked());

  /*offset_gi->setVisible(ui->actionVisToggleOffset->isChecked());
	 */
}

void
MainWindow::on_actionToggleFullscreen_triggered() {
	if (this->isFullScreen()) {
		this->showNormal();
	} else {
		this->showFullScreen();
	}
	on_actionResize_triggered();
}

void
MainWindow::on_actionResize_triggered() {
	auto br = input_gi->boundingRect();
	//br |= skeleton_gi->boundingRect();
	/*
  if (instance_triangulation_gi) {
    br |= instance_triangulation_gi->boundingRect();
  };
  br |= skeleton_gi->boundingRect();
  br |= offset_gi->boundingRect();
	 */

	ui->gV->setSceneRect(br);
	ui->gV->fitInView(br, Qt::KeepAspectRatio);
}

void MainWindow::on_actionDefineWeight_triggered() {
	std::string wt = "Set Weight";
	weightDialog = new WeightDialog(wt);
	weightDialog->setGeometry(this->x(), this->y(),184,134);
	weightDialog->show();
}

void MainWindow::on_actionResetAll_triggered() {
	monos.reset();
	onLowerChain = true;
	lowerChainDone = upperChainDone = bothChainsDone = mergeDone = false;

	/* start with lower chain, rest by steppint through */
	if(!monos.initSkeletonQueue(onLowerChain)) {
		LOG(WARNING) << "Error Init SkeletonQueue!";
	}

	on_actionResize_triggered();
}

void
MainWindow::
showEvent(QShowEvent *) {
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
	(void) event;

	// Kernel::Point_2 p( ui->gV->getMousePoint().x(),  ui->gV->getMousePoint().y() );
	// XXX s.kinetic_triangulation->showInfoAt(p);
}

void
MainWindow::
update_time_label() {
	scene.update(scene.sceneRect());
	skeleton_gi->update(scene.sceneRect());
	if(bothChainsDone && !mergeDone) {
		time_label->setText(QString(" -merging- "));
	} else if(mergeDone) {
		time_label->setText(QString(" -finished- "));
	} else {
	  auto t = CGAL::to_double( monos.wf.getTime() );
	  time_label->setText(QString("t: %1 ").
	    arg(t)
    //arg(s.wp.event_ctr()).
	//    arg((kinetic_triangulation_gi->drawing_time_offset() < 0) ? "-" : "+" ).
	//    arg(abs(CGAL::to_double(kinetic_triangulation_gi->drawing_time_offset())), 9, 'f', 5)
	    );
	}
}

void
MainWindow::
time_changed() {
	skeleton_gi->modelChanged();
	//  if (s.wp.simulation_is_finished()) {
	//    simulation_has_finished();
	//  }
	update_time_label();
	//  kinetic_triangulation_gi->setTime(s.wp.get_time());
	//  kinetic_triangulation_gi->highlighted_clear();
	//  if (!s.wp.simulation_is_finished()) {
	//    const std::shared_ptr<const EventQueueItem> next = s.wp.peak();
	//    kinetic_triangulation_gi->highlighted_add(next->get_priority().t);
	//  }
}


void
MainWindow::on_actionTimeForwardAfterChains_triggered() {

	if(!bothChainsDone) {
		do {
			if(!upperChainDone || !lowerChainDone) {
				if(!monos.computeSingleSkeletonEvent(onLowerChain)) {
					if(onLowerChain) {
						lowerChainDone = true;
					} else {
						upperChainDone = true;
					}

					if(lowerChainDone && !upperChainDone) {
						monos.finishSkeleton(onLowerChain);
						onLowerChain = false;
						monos.initSkeletonQueue(onLowerChain);
					}
				}
			}
		} while(!upperChainDone);

		if(lowerChainDone && upperChainDone && !bothChainsDone) {
			LOG(INFO) << "both done! TODO: finish 2nd skel end + merge";
			monos.finishSkeleton(onLowerChain);
			bothChainsDone = true;
			monos.s.initMerge();
		}

	}

	time_changed();
	on_actionResize_triggered();
}

void
MainWindow::on_actionFinishComputation_triggered() {
	//  s.wp.reset_time_to_last_event(); // backspace -- reset to last event time

	on_actionTimeForwardAfterChains_triggered();

	while(monos.s.SingleMergeStep());

	mergeDone = true;
	monos.s.finishMerge();

	time_changed();
	on_actionResize_triggered();
}

void
MainWindow::on_actionEventStep_triggered() {
	//  s.wp.advance_step(); // n - Move forward in time to the next event and handle it
	if(!upperChainDone || !lowerChainDone) {
		if(!monos.computeSingleSkeletonEvent(onLowerChain)) {
			if(onLowerChain) {
				lowerChainDone = true;
			} else {
				upperChainDone = true;
			}

			if(lowerChainDone && !upperChainDone) {
				monos.finishSkeleton(onLowerChain);
				onLowerChain = false;
				monos.initSkeletonQueue(onLowerChain);
			}
		}
	}

	if(lowerChainDone && upperChainDone && !bothChainsDone) {
		LOG(INFO) << "both done! TODO: finish 2nd skel end + merge";
		monos.finishSkeleton(onLowerChain);
		bothChainsDone = true;
		monos.s.initMerge();
	}

	if(bothChainsDone && !mergeDone) {
		if(!monos.s.SingleMergeStep()) {
			mergeDone = true;
			monos.s.finishMerge();
		}
	}

	time_changed();
}


void
MainWindow::simulation_has_finished() {
	if (did_finish) return;
	did_finish = true;
	//  ui->actionVisToggleWavefront->setChecked(false);
	//  ui->actionVisToggleKineticTriangulation->setChecked(false);
	updateVisibilities();
}

