#include "weightdialog.h"
#include "ui_weightDialog.h"

WeightDialog::WeightDialog(const std::string& title) :
	CGAL::Qt::DemosMainWindow(),
	title(title),
	ui(new Ui::WeightDialog) {

	ui->setupUi(this);
	setWindowTitle(QString::fromStdString(title));
	scene.setItemIndexMethod(QGraphicsScene::NoIndex);
//	ui->gV->setScene(&scene);
//	ui->gV->setMouseTracking(true);
//	ui->gV->scale(1, -1);
//	ui->gV->setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
//	ui->gV->setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
	//addNavigation(ui->gV);

	/* add navigation */
//	navigation = new MyGraphicsViewNavigation();
//	ui->gV->viewport()->installEventFilter(navigation);
//	ui->gV->installEventFilter(navigation);
//	QObject::connect(navigation, SIGNAL(mouseCoordinates(QString)),
//			xycoord, SLOT(setText(QString)));
//	this->view = ui->gV;

	connect(ui->pushButtonOK,SIGNAL(clicked()),this,SLOT(on_actionQuit_triggered()));

}

WeightDialog::~WeightDialog() {}

void WeightDialog::showEvent(QShowEvent *) {}

void WeightDialog::mousePressEvent(QMouseEvent *event) {}
