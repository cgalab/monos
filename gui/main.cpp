#include "mainwindow.h"

#include "tools.h"

#include <QApplication>
#include <fstream>
#include <iostream>

#include "Config.h"
#include "Monos.h"

int main(int argc, char *argv[]) {

	setupEasylogging(argc, argv);
	QApplication a(argc, argv);

	Config config(argc, argv, true);
	if(!config.isValid()) {return 0;}

	Monos monos(config);

	std::string title =
#ifdef CMAKE_BUILD_TYPE
	"Monos GUI (" CMAKE_BUILD_TYPE ")"
#else
	"Monos GUI - " + monos.config.getFileNameNoPath();
#endif
	;

	MainWindow w(title, monos);
	w.show();

	return a.exec();
	return 0;
}
