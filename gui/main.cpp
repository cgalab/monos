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

	Args argPair({argc,argv});
	Monos monos(argPair,true);

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
