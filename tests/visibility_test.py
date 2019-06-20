#!/usr/bin/env python
import cv2
import rospy, rospkg
import imutils
import numpy as np
import visilibity as vis
import numpy.linalg as LA
from policy.gridgraph import GridGraph, LiveGridGraph

def ccw(Ax,Ay,Bx,By,Cx,Cy):
    return (Cy-Ay)*(Bx-Ax) > (By-Ay)*(Cx-Ax)

def line_segments_intersect(line1, line2): #https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
	(x1,y1) = line1[0]
	(x2,y2) = line1[1]
	(x3,y3) = line2[0]
	(x4,y4) = line2[1]
	if max(x1,x2) < min(x3,x4) or min(x1,x2) > max(x3,x4):
		return False
	if x1-x2==0 and x3-x4==0:
		if x1 != x3: return False 
		else: return True
	elif x1-x2==0:
		return ccw(x1,y1,x3,y3,x4,y4) != ccw(x2,y2,x3,y3,x4,y4) and ccw(x1,y1,x2,y2,x3,y3) != ccw(x1,y1,x2,y2,x4,y4)
	elif x3-x4==0:
		return ccw(x1,y1,x3,y3,x4,y4) != ccw(x2,y2,x3,y3,x4,y4) and ccw(x1,y1,x2,y2,x3,y3) != ccw(x1,y1,x2,y2,x4,y4)
	else:
		A1 = (y1-y2)/(x1-x2)
		A2 = (y3-y4)/(x3-x4)
		b1 = y1-A1*x1
		b2 = y3-A2*x2
		if A1==A2: return False 
		else: return ccw(x1,y1,x3,y3,x4,y4) != ccw(x2,y2,x3,y3,x4,y4) and ccw(x1,y1,x2,y2,x3,y3) != ccw(x1,y1,x2,y2,x4,y4)

def point_in_polygon(polygon, point):
	return cv2.pointPolygonTest(polygon, point, measureDist = False) > 0

def visible_set(gridGraph, observationPoint):
	occGrid = gridGraph.occ_grid.copy()
	#cv2.imshow('image', occGrid)
	#cv2.waitKey(0)
	occGrid[np.where(occGrid == 0)] = 255	
	thresh = cv2.threshold(occGrid, 150, 255, cv2.THRESH_BINARY_INV)[1]
	
	#cv2.imshow('image', thresh)
	#cv2.waitKey(0)

	cnts = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	ox = int(observationPoint[0]-gridGraph.origin[0])/gridGraph.img_res
	oy = int((-observationPoint[1]+gridGraph.origin[1])/gridGraph.img_res)+gridGraph.imgheight
	print(ox)
	print(oy)
	observer = vis.Point(ox,oy)
	obstacles = []
	for idx, c in enumerate(cnts):
		points = []
		for p in c:
			points.append(vis.Point(float(p[0][0]),float(p[0][1])))
		obstacles.append(vis.Polygon(points))
	env = vis.Environment(obstacles)
	observer.snap_to_boundary_of(env, epsilon)
	observer.snap_to_vertices_of(env, epsilon)
	isovist = vis.Visibility_Polygon(observer, env, epsilon)
	isocnt = save_print_contour(isovist)
	isocnt.append([[int(isovist[0].x()),int(isovist[0].y())]])
	isocnt = np.asarray(isocnt)
	image = occGrid.copy() #for visualization
	cv2.drawContours(image, [isocnt],-1,0, 2)
	visible_set = []
	for edge in list(gridGraph.graph.edges()):
		(node1,node2) = edge
		(xloc1, yloc1) = gridGraph.graph.node[node1]['pos']
		(xloc2, yloc2) = gridGraph.graph.node[node2]['pos']
		x1 = int((xloc1 - gridGraph.origin[0])/gridGraph.img_res)
		y1 = int(((-yloc1 + gridGraph.origin[1])/gridGraph.img_res)+gridGraph.imgheight)
		x2 = int((xloc2 - gridGraph.origin[0])/gridGraph.img_res)
		y2 = int(((-yloc2 + gridGraph.origin[1])/gridGraph.img_res)+gridGraph.imgheight)
		visible = point_in_polygon(isocnt,(x1,y1)) and point_in_polygon(isocnt,(x2,y2))
		#visible = True
		for i in range(1,len(isocnt)):
			if line_segments_intersect((isocnt[i-1][0],isocnt[i][0]),((x1,y1),(x2,y2))): visible = False
		if visible: 
			visible_set.append(edge)
			cv2.line(image,(x1,y1),(x2,y2),0)
	cv2.imshow("Image", image)
	cv2.waitKey(0)
	return visible_set

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
pgm0 = mapdir + 'simple1.pgm'
yaml0 = mapdir + 'simple1.yaml'
start = (0.0, 0.0)
#goal = (4.0, -4.0) #for robohub
goal = (-8.0, 4.5)
#goal = (0.0,8.5)
# goal = (-6.0, 3.7)

map0 = GridGraph(pgm0, yaml0, goal, graph_res=1.5, robot_width=0.5)


vis_set = visible_set(map0, (0,0))
