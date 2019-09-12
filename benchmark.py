#!/usr/bin/env python3

import os
import tempfile
import argparse
import getpass
import socket
import time
import resource
from subprocess import CalledProcessError,TimeoutExpired, check_output

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

def run_monos_instance(polygon, monos_path, timeout):
    cmd = get_command_for_file(monos_path,polygon)
    try:
        output = check_output(cmd, stdin=None, stderr=None, shell=False, timeout=timeout)
        print(output.decode("utf-8").rstrip())
    except CalledProcessError:
        print("0,0,",polygon)
    except TimeoutExpired:
        print("0,0,",polygon)

def start_benchmark(args, tempDir):
    polygons = get_polygons(args.polygon_path, args.polygon)
    timeout = args.timeout

    for polygon in polygons:
        if timeout == 0:
            a = polygon.rfind('p')
            a = polygon[0:a].rfind('p')
            b = polygon.rfind('.')
            timeout = 5*int(polygon[a+1:b])
            if timeout < 0 and timeout > 1000000:
                timeout = 0
        run_monos_instance(polygon, args.monos, timeout)


def main():
    runner_info = '%s@%s'%(getpass.getuser(), socket.gethostname())

    parser = argparse.ArgumentParser(description='run monos for benchmarking')

    parser.add_argument('monos', help='path to monos executable')
    parser.add_argument('--polygon',        action="store", dest="polygon",               default="", help='a single input polygon')
    parser.add_argument('--polygon-path',   action="store", dest="polygon_path",          default="", help='path to the input polygons')
    parser.add_argument('--timeout',        action="store", dest='timeout',     type=int, default=0,  help='timeout in seconds to kill monos')

    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tempDir:
        start_benchmark(args, tempDir)


if __name__ == "__main__":
    main()
