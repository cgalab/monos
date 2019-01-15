//#include "../../gui/mainwindow.h"
//
//#include "tools.h"
//
//#include <QApplication>
//#include <fstream>
//#include <iostream>
//
//#include "Config.h"
//#include "Monos.h"
//

#include "tools.h"

#include <fstream>
#include <iostream>

#include "Config.h"
#include "Monos.h"

int main(int argc, char *argv[]) {
	setupEasylogging(argc, argv);

	static std::list<std::string> args;
	for(auto i = 1; i < argc; ++i) { args.push_back(std::string(argv[i])); }

	/* start */
	Monos monos(args);

	monos.run();

	return 0;
}


/*
int main(int,char*[]) {
  BGLGraph graph;

  std::ifstream dot("../x");
  graph = BGLGraph::create_from_graphml(dot);
  // graph.print();


  SkeletonStructure s;
  s.add_graph(graph);
  s.build_kt();

  return 0;
}
*/
//
//int main(int argc, char *argv[]) {
//  //el::Configurations defaultConf;
//  //defaultConf.setToDefault();
//  //defaultConf.setGlobally(el::ConfigurationType::Format, "%datetime %level %func(): %msg");
//  //el::Loggers::reconfigureLogger("default", defaultConf);
//
//  //START_EASYLOGGINGPP(argc, argv);
//
//	setupEasylogging(argc, argv);
//	QApplication a(argc, argv);
//
//  //int skip_to = 0;
//  //bool skip_all = 0;
////  std::string skoffset;
////  std::string skip_until_time;
////
////  while (1) {
////    int option_index = 0;
////    int r = getopt_long(argc, argv, "hs:SO:R:T:", long_options, &option_index);
////
////    if (r == -1) break;
////    switch (r) {
////      case 'h':
////        usage(argv[0], 0);
////        break;
////
////      case 'O':
////        skoffset = std::string(optarg);
////        break;
////
////      /*
////      case 's':
////        skip_to = atoi(optarg);
////        break;
////
////      case 'S':
////        skip_all = true;
////        break;
////        */
////
////      case 'T':
////        skip_until_time = std::string(optarg);
////        break;
////
////      case 'R':
//////        my_srand(atoi(optarg));
////        break;
////
////      default:
////        std::cerr << "Invalid option " << (char)r << std::endl;
////        exit(1);
////    }
////  }
////
////  if (argc - optind > 1) {
////    usage(argv[0], 1);
////  }
////
////  bool use_stdin = true;
////  std::ifstream filestream;
//  std::string title = "Monos";
////  if (argc - optind == 1) {
////    std::string fn(argv[optind]);
////    if (fn != "-") {
////      title += " [" + fn + "]";
////      filestream.open(fn);
////      if (! filestream.is_open()) {
////        //LOG(ERROR) << "Failed to open " << fn << ": " << strerror(errno);
////        exit(1);
////      }
////      use_stdin = false;
////    }
////  }
////  std::istream &in = use_stdin ? std::cin : filestream;
////  (void)in;
//
//	static std::list<std::string> args;
//	for(auto i = 1; i < argc; ++i) { args.push_back(std::string(argv[i])); }
//
//	/* start */
//	Monos monos(args);
//
////  MainWindow w(title, in, skip_to, skip_all, skip_until_time, skoffset);
//	MainWindow w(title, monos);
//	w.show();
//
//  return a.exec();
//  return 0;
//}
