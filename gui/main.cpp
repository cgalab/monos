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

	static std::list<std::string> args;
	for(auto i = 1; i < argc; ++i) { args.push_back(std::string(argv[i])); }

	Monos monos(args);

#ifdef CMAKE_BUILD_TYPE
	"MonosGUI (" CMAKE_BUILD_TYPE ")";
#else
	"MonosGUI";
#endif

	std::string title = "Monos";
	MainWindow w(title, monos);
	w.show();

	return a.exec();
	return 0;
}
