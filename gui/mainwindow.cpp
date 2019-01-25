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

	input_gi = std::make_shared<InputGraphicsItem>(&monos.data.getBasicInput());
	scene.addItem(input_gi.get());

	skeleton_gi = std::make_shared<ArcGraphicsItem>(&monos.wf.nodes, &monos.wf.arcList, &monos.data.lines);
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

void MainWindow::on_actionDefineWeight_triggered() {
	std::string wt = "Weight";
	weightDialog = new WeightDialog(wt);
	weightDialog->setGeometry(this->x(), this->y(),184,134);
	weightDialog->show();
	auto spinBox = weightDialog->ui->edgeSelect;
	spinBox->setRange(0, monos.data.getPolygon().size()-1);
	spinBox->setSingleStep(1);
	spinBox->setValue(0);
	updateWeightValue(0);
	connect(weightDialog->ui->pushButtonOK,SIGNAL(clicked()),this,SLOT(on_actionDefineWeightDialogClosed()));
	connect(weightDialog->ui->weightInput, SIGNAL(returnPressed()),this,SLOT(on_actionDefineWeightDialogClosed()));
	connect(weightDialog->ui->edgeSelect, QOverload<int>::of(&QSpinBox::valueChanged),
	    [=](int i){ updateWeightValue(i); });
}



void MainWindow::updateWeightValue(int idx) {
	Exact weight = monos.data.w(idx);
	weightDialog->ui->weightInput->setText(QString::fromStdString(std::to_string(weight.doubleValue())));
}

void MainWindow::on_actionDefineWeightDialogClosed() {
	bool ok 	 = false;
	uint edgeIdx = weightDialog->ui->edgeSelect->value();
	auto weight  = weightDialog->ui->weightInput->text().toDouble(&ok);

	if(ok) {
		monos.data.setEdgeWeight(edgeIdx,Exact(weight));
	}
}

void MainWindow::on_actionResetAll_triggered() {
	monos.reset();

	onLowerChain = firstStart = true;
	lowerChainDone = upperChainDone = bothChainsDone = mergeDone = false;

	on_actionResize_triggered();
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
		auto t = CGAL::to_double( monos.wf.getTime() );
		time_label->setText(QString("t: %1 ").arg(t));
	}
}

void MainWindow::time_changed() {
	skeleton_gi->modelChanged();
	update_time_label();
}


void MainWindow::on_actionTimeForwardAfterChains_triggered() {

	while(!bothChainsDone) {
		on_actionEventStep_triggered();
	}

	time_changed();
	on_actionResize_triggered();
}

void MainWindow::on_actionFinishComputation_triggered() {

	on_actionTimeForwardAfterChains_triggered();

	while(monos.s.SingleMergeStep());

	monos.s.finishMerge();
	mergeDone = true;

	time_changed();
	on_actionResize_triggered();
}

void MainWindow::on_actionEventStep_triggered() {

	if(firstStart) {
		if (!monos.initSkeletonQueue(onLowerChain)) {
			LOG(WARNING) << "Error Init SkeletonQueue!";
		}
		firstStart = false;
	}

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
		monos.finishSkeleton(onLowerChain);
		bothChainsDone = true;
		monos.s.initMerge();
	}

	if(bothChainsDone && !mergeDone) {
		if(!monos.s.SingleMergeStep()) {
			monos.s.finishMerge();
			mergeDone = true;
		}
	}

	time_changed();
}


void MainWindow::simulation_has_finished() {
	if (did_finish) return;
	did_finish = true;
	updateVisibilities();
}

