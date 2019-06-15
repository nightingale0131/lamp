#!/usr/bin/env python
import cv2
import rospy, rospkg
import imutils
import numpy as np
import visilibity as vis
import numpy.linalg as LA

def save_print(polygon):
    end_pos_x = []
    end_pos_y = []
    print ('Points of Polygon: ')
    for i in range(polygon.n()):
        x = polygon[i].x()
        y = polygon[i].y()
        
        end_pos_x.append(x)
        end_pos_y.append(y)
                
        print( x,y) 
        
    return end_pos_x, end_pos_y 

def save_print_contour(polygon):
	cnt = []
	#print ('Points of Polygon: ')
	for i in range(polygon.n()):
		x = int(polygon[i].x())
		y = int(polygon[i].y())
		cnt.append([[x,y]])
		#print( x,y) 
	return cnt 

epsilon = 0.0000001

rospack = rospkg.RosPack()
pkgdir = rospack.get_path('policy')
mapdir = pkgdir + '/maps/'
pgm0 = mapdir + 'test.pgm'
yaml0 = mapdir + 'simple1.yaml'
image = cv2.imread(pgm0)
image[np.where((image == [0,0,0]).all(axis = 2))] = [255,255,255]
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#gray = cv2.bitwise_not(gray)
#blurred = cv2.GaussianBlur(gray, (5, 5), 0)
thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)[1]
#thresh = thresh[10:-10,10:-10]

cv2.imshow("sample", thresh)
#cv2.waitKey(0)

cnts = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

'''
for c in cnts:
	cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
 
	# show the output image
	cv2.imshow("Image", image)
	cv2.waitKey(0)
'''
print(np.size(cnts[0]))
observer = vis.Point(100,100)
obstacles = []
maxArea = 0
maxidx = 0
for idx, c in enumerate(cnts):
	if cv2.contourArea(c) > maxArea:
		maxArea = cv2.contourArea(c)
		maxidx = idx
'''for idx in range(len(cnts)):
	minpt = 9999
	minidx = 0
	for i, p in enumerate(cnts[idx]):
		if LA.norm(p[0]) < minpt:
			minpt = LA.norm(p[0])
			minidx = i
	cnts[idx] = cnts[idx][minidx:]+cnts[idx][:minidx]'''
for idx, c in enumerate(cnts):
	points = []
	for p in c:
		points.append(vis.Point(float(p[0][0]),float(p[0][1])))
	obstacles.append(vis.Polygon(points))
env = vis.Environment(obstacles)
observer.snap_to_boundary_of(env, epsilon)
observer.snap_to_vertices_of(env, epsilon)
isovist = vis.Visibility_Polygon(observer, env, epsilon)

# Print the point of the visibility polygon of 'observer' and save them 
# in two arrays in order to draw the polygon later
isocnt = save_print_contour(isovist)
# Add the first point again because the function to draw, draw a line from
# one point to the next one and to close the figure we need the last line
# from the last point to the first one

isocnt.append([[int(isovist[0].x()),int(isovist[0].y())]])
isocnt = np.asarray(isocnt)
#print(cnts[0])
print(isocnt)

cv2.drawContours(image, [isocnt],-1,(0, 255, 0), 2)
cv2.imshow("Image", image)
cv2.waitKey(0)
