#!/usr/bin/env python3

import os
import tempfile
import subprocess
import argparse
import getpass
import socket
import time
import resource

def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i + n]


# returns 1 if timeout 0 otherwise
def run_single_monos(args, polygons, timeout):
    cmds = [[os.path.abspath(args.monos), '--t', f] for f in polygons]
   
    procs = [subprocess.Popen(cmd, stdin=None, stderr=None, shell=False) for cmd in cmds]
    for proc in procs:
        try:
            if timeout != 0:
                outs, errs = proc.communicate(timeout=timeout)
            else:
                outs, errs = proc.communicate()
        except subprocess.CalledProcessError as e:
            print(e)
            proc.kill()
            outs, errs = proc.communicate()
        except subprocess.TimeoutExpired:
            proc.kill()
            outs, errs = proc.communicate()

    return 0

def run_monos(args, tempDir, polygon, polygonPath):
    if polygonPath:
        absPath = os.path.abspath(polygonPath)
        files = [absPath + '/' + f for f in os.listdir(absPath) if f.lower().endswith(('.obj', '.gml', '.graphml'))] 
        
        # get list of list in chunks of num-threads
        files = chunks(files,args.threads)

        for f in files:
            run_single_monos(args,f,args.timeout)
    elif polygon and not polygonPath and polygon.lower().endswith(('.obj', '.gml', '.graphml')):
        run_single_monos(args,[os.path.abspath(polygon)],args.timeout)
    else:
        print("no input provided")

def main():
    runner_info = '%s@%s'%(getpass.getuser(), socket.gethostname())

    parser = argparse.ArgumentParser(description='run monos for benchmarking')

    parser.add_argument('monos', help='path to monos executable')
    parser.add_argument('--polygon', action="store", dest="polygon", default="", help='a single input polygon')
    parser.add_argument('--polygon-path', action="store", dest="polygonPath", default="", help='path to the input polygons')

    parser.add_argument('--timeout', dest='timeout', type=int, action="store", default=0, help='timeout in seconds to kill monos')
    parser.add_argument('--threads', dest='threads', type=int, action="store", default=1, help='use <number> parallel instances')

    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tempDir:
        run_monos(args, tempDir, args.polygon, args.polygonPath)


if __name__ == "__main__":
    main()
