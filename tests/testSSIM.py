# import the necessary packages
from skimage.measure import compare_ssim as ssim
import matplotlib.pyplot as plt
import numpy as np
import cv2

def compare_images(imageA, imageB, title):
    # compute the mean squared error and structural similarity
    # index for the images
    s = ssim(imageA, imageB, )
                     
    # setup the figure
    fig = plt.figure(title)
    plt.suptitle("SSIM: %.2f" % ( s))
                                 
    # show first image
    ax = fig.add_subplot(1, 2, 1)
    plt.imshow(imageA, cmap = plt.cm.gray)
    plt.axis("off")
                                                 
    # show the second image
    ax = fig.add_subplot(1, 2, 2)
    plt.imshow(imageB, cmap = plt.cm.gray)
    plt.axis("off")
                                                                 
    # show the images
    plt.show()


a = cv2.imread("maps/simple1.pgm")
b = cv2.imread("maps/simple_same.pgm")
c = cv2.imread("maps/problem.pgm")

a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
b = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
c = cv2.cvtColor(c, cv2.COLOR_BGR2GRAY)

b = b[0:234, 0:233]
c = c[0:234, 0:233]

compare_images(a,b,"Same")
compare_images(b,c,"big difference")
