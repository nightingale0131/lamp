#!/usr/bin/env python
import cv2
import rospy, rospkg
import imutils
import numpy as np
import visilibity as vis
import numpy.linalg as LA
import policy.utility as util

def ccw(Ax,Ay,Bx,By,Cx,Cy):
    return (Cy-Ay)*(Bx-Ax) > (By-Ay)*(Cx-Ax)

def line_segments_intersect(line1, line2): #https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
	(x1,y1) = line1[0]
	(x2,y2) = line1[1]
	(x3,y3) = line2[0]
	(x4,y4) = line2[1]
	if max(x1,x2) < min(x3,x4) or min(x1,x2) > max(x3,x4):
		return False
        if util.isclose(0, (x1-x2)) and util.isclose(0, (x3-x4)): 
		if not util.isclose(x1,x3): return False 
		else: return True
        elif util.isclose(0, (x1-x2)):
		return ccw(x1,y1,x3,y3,x4,y4) != ccw(x2,y2,x3,y3,x4,y4) and ccw(x1,y1,x2,y2,x3,y3) != ccw(x1,y1,x2,y2,x4,y4)
        elif util.isclose(0, (x3-x4)):
		return ccw(x1,y1,x3,y3,x4,y4) != ccw(x2,y2,x3,y3,x4,y4) and ccw(x1,y1,x2,y2,x3,y3) != ccw(x1,y1,x2,y2,x4,y4)
	else:
		A1 = (y1-y2)/(x1-x2)
		A2 = (y3-y4)/(x3-x4)
		b1 = y1-A1*x1
		b2 = y3-A2*x2
		if util.isclose(A1,A2): return False 
		else: return ccw(x1,y1,x3,y3,x4,y4) != ccw(x2,y2,x3,y3,x4,y4) and ccw(x1,y1,x2,y2,x3,y3) != ccw(x1,y1,x2,y2,x4,y4)

def point_in_polygon(polygon, point):
	return cv2.pointPolygonTest(polygon, point, measureDist = False) > 0

#https://stackoverflow.com/a/20679579
def line_eqn(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection_point(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if not util.isclose(0, D):
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        # print("Line: {} \nEdge: {}".format(L1, L2))
        return False

def find_obstacles(gridgraph):
    # returns list of obstacles (list of polygons)
    # threshold is a little low because I was trying to exclude unknown areas as obstacles
    grid = gridgraph.occ_grid.copy()

    grid[np.where(grid == 0)] = 255
    thresh = cv2.threshold(grid, 125, 255, cv2.THRESH_BINARY_INV)[1]

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # set contours as visilibity polygons
    obstacles = []
    for c in cnts:
        points = []
        for p in c:
            points.append(vis.Point(float(p[0][0]),float(p[0][1])))

        obstacle = vis.Polygon(points)
        obstacle.eliminate_redundant_vertices(1)
        obstacles.append(obstacle)

    return obstacles

def visible_set(gridGraph, observationPoint, obstacles):
    # obstacles - list of visilibity polygons

    # setup environment and observer location
    ox = int((observationPoint[0]-gridGraph.origin[0])/gridGraph.img_res)
    oy = int((-observationPoint[1]+gridGraph.origin[1])/gridGraph.img_res)+gridGraph.imgheight
    # print(ox)
    # print(oy)
    observer = vis.Point(ox,oy)
    env = vis.Environment(obstacles)
    observer.snap_to_boundary_of(env, epsilon)
    observer.snap_to_vertices_of(env, epsilon)

    # get visibility polygon
    isovist = vis.Visibility_Polygon(observer, env, epsilon)
    isocnt = save_print_contour(isovist)

    # prep visualization
    # image = gridGraph.occ_grid.copy()
    # image[np.where(image == 0)] = 255 # easier to see
    # cv2.drawContours(image, [isocnt],-1, 0, 2)

    # find visible edges
    visible_set = []
    for edge in list(gridGraph.graph.edges()):
        (node1,node2) = edge
        (xloc1, yloc1) = gridGraph.graph.node[node1]['pos']
        (xloc2, yloc2) = gridGraph.graph.node[node2]['pos']
        x1 = int((xloc1 - gridGraph.origin[0])/gridGraph.img_res)
        y1 = int(((-yloc1 + gridGraph.origin[1])/gridGraph.img_res)+gridGraph.imgheight)
        x2 = int((xloc2 - gridGraph.origin[0])/gridGraph.img_res)
        y2 = int(((-yloc2 + gridGraph.origin[1])/gridGraph.img_res)+gridGraph.imgheight)

        pt1_visible = point_in_polygon(isocnt,(x1,y1))
        pt2_visible = point_in_polygon(isocnt,(x2,y2))
        visible = pt1_visible and pt2_visible
        at_least_one_endpt_visible = pt1_visible or pt2_visible

        if at_least_one_endpt_visible:
            for i in range(1,len(isocnt)):
                if line_segments_intersect((isocnt[i-1][0],isocnt[i][0]),((x1,y1),(x2,y2))): 
                    visible = False				
                    visIntersect = intersection_point(line_eqn(isocnt[i-1][0],isocnt[i][0]),line_eqn((x1,y1),(x2,y2)))
                    # if intersection_point returns False here, it means
                    # the lines are parallel and overlap each other
                    for obstacle in obstacles:
                        for j in range(1,obstacle.n()):
                            if line_segments_intersect(((obstacle[j-1].x(),obstacle[j-1].y()),(obstacle[j].x(),obstacle[j].y())),((x1,y1),(x2,y2))): 
                                obsIntersect = intersection_point(line_eqn((obstacle[j-1].x(),obstacle[j-1].y()),(obstacle[j].x(),obstacle[j].y())),line_eqn((x1,y1),(x2,y2)))
                                if (visIntersect == False) or (obsIntersect == False) or util.euclidean_distance(visIntersect, obsIntersect) < 0.1/gridGraph.img_res:
                                    visible = True
        if visible: 
            visible_set.append(edge)
            # cv2.line(image,(x1,y1),(x2,y2),0)

    # cv2.circle(image, (ox, oy), 5, (0,0,255), thickness=-1)
    # cv2.imshow("Image", image)
    # cv2.waitKey(0)
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
    # converts visilibity polygon to cv2 printable contour
	cnt = []
	#print ('Points of Polygon: ')
	for i in range(polygon.n()):
		x = int(polygon[i].x())
		y = int(polygon[i].y())
		cnt.append([[x,y]])

                if i == 0: init_pt = [[x,y]]
		#print( x,y) 

        cnt.append(init_pt)     # close off polygon
        cnt = np.asarray(cnt)   # transform into np array
	return cnt 

epsilon = 0.0000001
