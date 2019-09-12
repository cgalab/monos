#!/usr/bin/env python3

from multiprocessing import Pool

def f(x,y):
    return x*y

if __name__ == '__main__':
    p = Pool(5)
    print(p.map(f, [(1,1), (2,2), (2,3)]))
    
