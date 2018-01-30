import os
import sys
import cv2
import numpy as np
import gflags

def process(a):
    img = cv2.imread(a, 0)
    
    ret,thresh = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    area = img.size - cv2.countNonZero(thresh)
    
    '''
        Canny returns white borders
    '''
    edges = cv2.Canny(img, 100, 200)
    perimeter = cv2.countNonZero(edges)
    
    ratio = float(area)/float(perimeter)
    print ratio   


if __name__ == '__main__':
    argv = gflags.FLAGS(sys.argv)
    mp = argv[1]
    name = os.getcwd() + "/maps/" + mp
    process(name)