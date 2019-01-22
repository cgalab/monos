#include "weightdialog.h"
#include "ui_weightDialog.h"

WeightDialog::WeightDialog(const std::string& title)
	: CGAL::Qt::DemosMainWindow()
	, ui(new Ui::WeightDialog)
	, title(title)
	{

	ui->setupUi(this);
	setWindowTitle(QString::fromStdString(title));
	scene.setItemIndexMethod(QGraphicsScene::NoIndex);
//	ui->gV->setScene(&scene);
//	ui->gV->setMouseTracking(true);
//	ui->gV->scale(1, -1);
//	ui->gV->setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
//	ui->gV->setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
	//addNavigation(ui->gV);

	connect(ui->pushButtonOK,SIGNAL(clicked()),this,SLOT(on_actionQuit_triggered()));

}

WeightDialog::~WeightDialog() {}

void WeightDialog::showEvent(QShowEvent *) {}

void WeightDialog::mousePressEvent(QMouseEvent *event) {}
