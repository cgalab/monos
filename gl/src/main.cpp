#include "gml/GMLGraph.h"
//#include "SkeletonStructure.h"
#include "tools.h"

#include <QApplication>
#include <fstream>
#include <iostream>
#include <getopt.h>

static struct option long_options[] = {
  { "help"        , no_argument      , 0, 'h'},
  { "skip"        , required_argument, 0, 's'},
  { "skip-all"    , no_argument      , 0, 'S'},
  { "skip-to-time", required_argument, 0, 'T'},
  { "sk-offset"   , required_argument, 0, 'O'},
  { "random-seeed", required_argument, 0, 'R'},
  { 0, 0, 0, 0}
};

[[noreturn]]
static void
usage(const char *progname, int err) {
  FILE *f = err ? stderr : stdout;

  fprintf(f,"Usage: %s [options] <POLYFILE>\n", progname);
  fprintf(f,"  Options: --skip=<n>                 Skip to event n at start.\n");
  fprintf(f,"           --skip-all                 Skip until end.\n");
  fprintf(f,"           --skip-to-time=<time>      Skip until <time>.\n");
  fprintf(f,"           --sk-offset=<offset-spec>  Draw offsets.\n");
  fprintf(f,"           --random-seed=<seed>       Seed for RNG (for debugging).\n");
  fprintf(f,"\n");
  fprintf(f,"  offset-spec = <one-block> [ ',' <one-block> ]\n");
  fprintf(f,"  one-block   = <one-offset> [ '+' one-block ]\n");
  fprintf(f,"  one-offset  = [<cnt> '*' ] <time>\n");
  fprintf(f,"  example: '0.01 + 3*0.025, 0.15' or '10 * 0.025'\n");
  exit(err);
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

int main(int argc, char *argv[]) {
  //el::Configurations defaultConf;
  //defaultConf.setToDefault();
  //defaultConf.setGlobally(el::ConfigurationType::Format, "%datetime %level %func(): %msg");
  //el::Loggers::reconfigureLogger("default", defaultConf);

  //START_EASYLOGGINGPP(argc, argv);
  QApplication a(argc, argv);

  //int skip_to = 0;
  //bool skip_all = 0;
  std::string skoffset;
  std::string skip_until_time;

  while (1) {
    int option_index = 0;
    int r = getopt_long(argc, argv, "hs:SO:R:T:", long_options, &option_index);

    if (r == -1) break;
    switch (r) {
      case 'h':
        usage(argv[0], 0);
        break;

      case 'O':
        skoffset = std::string(optarg);
        break;

      /*
      case 's':
        skip_to = atoi(optarg);
        break;

      case 'S':
        skip_all = true;
        break;
        */

      case 'T':
        skip_until_time = std::string(optarg);
        break;

      case 'R':
//        my_srand(atoi(optarg));
        break;

      default:
        std::cerr << "Invalid option " << (char)r << std::endl;
        exit(1);
    }
  }

  if (argc - optind > 1) {
    usage(argv[0], 1);
  }

  bool use_stdin = true;
  std::ifstream filestream;
  std::string title = "offsetter";
  if (argc - optind == 1) {
    std::string fn(argv[optind]);
    if (fn != "-") {
      title += " [" + fn + "]";
      filestream.open(fn);
      if (! filestream.is_open()) {
        //LOG(ERROR) << "Failed to open " << fn << ": " << strerror(errno);
        exit(1);
      }
      use_stdin = false;
    }
  }
  std::istream &in = use_stdin ? std::cin : filestream;
  (void)in;

  //MainWindow w(title, in, skip_to, skip_all, skip_until_time, skoffset);
  //w.show();

  return a.exec();
  return 0;
}
