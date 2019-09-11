#!/usr/bin/env python3

import os
import tempfile
import argparse
import getpass
import socket
import time
import resource
import threading
from multiprocessing import Pool
from subprocess import CalledProcessError, STDOUT, check_output

monos_path = ""
timeout = 0
#lock = threading.Lock()

def get_polygons(polygon_path, polygon):
    files = []
    if polygon_path:
        abs_path = os.path.abspath(polygon_path)
        files = [abs_path + '/' + f for f in os.listdir(abs_path) if f.lower().endswith(('.obj', '.gml', '.graphml'))] 
    elif polygon and polygon.lower().endswith(('.obj', '.gml', '.graphml')):
        files = [polygon]
    else:
        print("no input provided")
    return files

def get_command_for_file(monos_path,polygon):
    return [os.path.abspath(monos_path), '--t', polygon] 

def run_monos_instance(polygon):
    cmd = get_command_for_file(monos_path,polygon)
    try:
        output = check_output(cmd, stderr=STDOUT, timeout=timeout)
        #lock.acquire()
        print(output)
        #lock.release()
    except CalledProcessError:
        #lock.acquire()
        print("0,0,",polygon)
        #lock.release()

    return 0

def start_benchmark(args, tempDir):
    timeout = args.timeout
    monos_path = args.monos

    polygons = get_polygons(args.polygon_path, args.polygon)
    #process_info = [PInfo(args.monos,poly,args.timeout,lock) for poly in polygons]

    pool = Pool(processes=args.num_threads)
    pool.map(run_monos_instance, polygons)


def main():
    runner_info = '%s@%s'%(getpass.getuser(), socket.gethostname())

    parser = argparse.ArgumentParser(description='run monos for benchmarking')

    parser.add_argument('monos', help='path to monos executable')
    parser.add_argument('--polygon',        action="store", dest="polygon",               default="", help='a single input polygon')
    parser.add_argument('--polygon-path',   action="store", dest="polygon_path",          default="", help='path to the input polygons')
    parser.add_argument('--timeout',        action="store", dest='timeout',     type=int, default=0,  help='timeout in seconds to kill monos')
    parser.add_argument('--threads',        action="store", dest='num_threads', type=int, default=1,  help='use <number> parallel instances (default: 1)')

    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tempDir:
        start_benchmark(args, tempDir)


if __name__ == "__main__":
    main()
