#!/usr/bin/env python3

import os
import tempfile
import subprocess
import argparse
import getpass
import socket
import time
import resource

# returns 1 if timeout 0 otherwise
def run_single_monos(args, polygons, timeout):
    cmds = [[os.path.abspath(args.monos), '--t', f] for f in polygons ]
    print(cmds)
    
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


#    try:
#        start = time.time()
#        if timeout != 0:
#            procs = subprocess.Popen(cmd, stdin=None, stderr=None, shell=False, timeout=timeout)
#            #o = subprocess.check_output(cmd, stdin=None, stderr=None, shell=False, timeout=timeout)
#        else:
#            o = subprocess.Popen(cmd, stdin=None, stderr=None, shell=False)
#            #o = subprocess.check_output(cmd, stdin=None, stderr=None, shell=False)
#        elapsed = time.time() - start
#    except subprocess.CalledProcessError as e:
#        print(e)
#        return 1
#    except subprocess.TimeoutExpired:
#        pass
#        return 1
#    

    return 0

def run_monos(args, tempDir, polygon, polygonPath):
    retVal = 0
    if polygonPath:
        absPath = os.path.abspath(polygonPath)
        print(absPath)
        files = [f for f in os.listdir(absPath) if f.lower().endswith(('.obj', '.gml', '.graphml'))] 
        for f in files:
            retVal = run_single_monos(args,absPath + '/' + f,args.timeout)
    elif polygon and not polygonPath and polygon.lower().endswith(('.obj', '.gml', '.graphml')):
        retVal = run_single_monos(args,os.path.abspath(polygon),args.timeout)
    else:
        print("no input provided")

    #print("to beat    : %15d"%(to_beat, ))

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
