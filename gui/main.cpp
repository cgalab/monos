#include "mainwindow.h"

#include "tools.h"

#include <QApplication>
#include <fstream>
#include <iostream>

#include "Config.h"
#include "Offsetter.h"

int main(int argc, char *argv[]) {

	setupEasylogging(argc, argv);
	QApplication a(argc, argv);

	static std::list<std::string> args;
	for(auto i = 1; i < argc; ++i) { args.push_back(std::string(argv[i])); }

	Config config(args);
	Offsetter offset(config);

#ifdef CMAKE_BUILD_TYPE
	"OffsetGUI (" CMAKE_BUILD_TYPE ")";
#else
	"OffsetGUI";
#endif

	std::string title = "Offsetter";
	MainWindow w(title, offset);
	w.show();

	return a.exec();
	return 0;
}
