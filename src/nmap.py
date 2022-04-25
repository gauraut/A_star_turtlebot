import os
import math
import numpy as np
from numpy import array
import cv2
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation 

fig, ax = plt.subplots()

ymax = 999
xmax = 999

def eq(x1,y1,x2,y2,x,y,f):
	m = (y2-y1)/(x2-x1)
	if (f == 1):
		c = (m*x) - y <= (m*x1) - y1 
	else:
		c = (m*x) - y >= (m*x1) - y1 
	return c

def create_map():
	m = np.zeros((1000,1000))
	am = np.zeros((1000,1000,3))
	hl = 40.4145
	for y in range(m.shape[0]):
		for x in range(m.shape[1]):
			if (((y - 800) ** 2) + ((x - 200) ** 2) <= ((100) ** 2) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if (((y - 200) ** 2) + ((x - 200) ** 2) <= ((100) ** 2) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if ( (x >= 25) and (x <= 175) and (y >= 425) and (y <= 575) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if ( (x >= 375) and (x <= 625) and (y >= 425) and (y <= 575) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if ( (x >= 725) and (x <= 875) and (y >= 600) and (y <= 800) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
	return m,am


def detect(m, am, x,y):
	global cll
	for cl in range(0,cll):
		if (1000 - (y+cl) > 0):
			if (m[ymax-(y+cl)][x] == 1):
				#print("1")
				return True
		if (1000 - (y-cl) <= ymax):
			if (m[ymax-(y-cl)][x] == 1):
				#print("2")
				return True
		if ( x+cl < xmax ):
			if (m[ymax-y][x+cl] == 1):
				#print("3")
				return True
		if ( x-cl > 0 ):
			if (m[ymax-y][x-cl] == 1):
				#print("4")
				return True
		if (1000 - (y+cl) > 0) and ( x+cl < xmax ):
			if (m[ymax-(y+cl)][x+cl] == 1):
				#print("1")
				return True
		if (1000 - (y+cl) > 0) and ( x-cl > 0 ):
			if (m[ymax-(y+cl)][x-cl] == 1):
				#print("2")
				return True
		if (1000 - (y-cl) <= ymax) and ( x+cl < xmax ):
			if (m[ymax-(y-cl)][x+cl] == 1):
				#print("2")
				return True
		if (1000 - (y-cl) <= ymax) and ( x-cl > 0 ):
			if (m[ymax-(y-cl)][x-cl] == 1):
				#print("2")
				return True
	global rr
	#print(rr)
	if (1000 - (y+rr) < 0) or (1000 - (y-rr)  >= ymax) or ( x-rr < 0 ) or ( x+rr > xmax ):
		return True
	return False


class Node:
	def __init__(self, data, orientaion, cost, parent, gcost, pxy):
		self.d = data
		self.o = orientaion
		self.c = cost
		self.p = parent
		self.g = gcost
		self.pxy = pxy


def action(goal, CurrentNode,UL,UR,m,am):
	x,y = CurrentNode.d

	xd,yd,theta,gc = plot_curve(x,y,CurrentNode.o,UL,UR)
	if ((1000-round(yd)) > 0) and (round(xd) < xmax) and ((1000-round(yd)) <= 1000) and (round(xd) > 0):
		if (m[ymax-round(yd)][round(xd)] == 1):
			return None
		if (detect(m, am, round(xd),round(yd))):
			return None
		else:
			cost = CurrentNode.c
			gc = getcost(goal, xd,yd)
			orientation = theta
			if(orientation >= 360):
				orientation = orientation - 360
			if(orientation < 0):
				orientation = orientation + 360

			child_node = Node([(xd),(yd)],orientation,cost+1.4,CurrentNode,gc,CurrentNode.d)
			return child_node
	else:
		return None 

def plot_curve(Xn,Yn,Thetai,UL,UR):
	t = 0
	r = 0.33 
	L = 28.7 
	dt = 0.1

	Thetan = 3.14 * Thetai / 180
	D=0
	while t<1:
		t = t + dt
		Xs = Xn
		Ys = Yn
		Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
		Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
		Thetan += (r / L) * (UR - UL) * dt
		D += (math.pow(Xn,2)+math.pow(Yn,2))
		plt.plot([Xs, Xn], [Ys, Yn], color="blue")
		
	Thetan = 180 * (Thetan) / 3.14
	
	return Xn, Yn, Thetan, D

def move(goal, direction, node, L,m,am):
	UL,UR = direction
	return action(goal, node,UL,UR,m,am)

def DS(node, goal, L,ga,m,am):
	Q = [node]
	CL = []
	OL,cOL,OLc,P = [],[],[],[]
	OL.append(node.d)
	OLc.append(node.d)
	cOL.append(node.c)
	P.append(node.pxy)

	action_set=[[50,50], [50,0],[0,50],[50,100],[100,50],[100,100],[0,100],[100,0]]
	p=0
	while Q:
		l = cOL.index(min(cOL))
		cn = Q.pop(l)
		o=cOL.pop(l)
		o=OL.pop(l)
		ox,oy=o
		CL.append(cn.d)
		print(cn.d[0],cn.d[1])

		cv2.imshow("Output", am)
		cv2.waitKey(1)
		if (((cn.d[1] - goal[1]) ** 2) + ((cn.d[0] - goal[0]) ** 2) <= ((3*L) ** 2)):
			return cn,CL,OLc,P
		for a in action_set:
			NewNode = move(goal, a,cn,L,m,am) 
			if NewNode != None:
				if (NewNode.d not in CL):
					if(NewNode.d not in OL):
						Q.append(NewNode)
						OL.append(NewNode.d)
						OLc.append(NewNode.d)
						P.append(NewNode.pxy)
						cOL.append(NewNode.g)
					else:
						T = Q[OL.index(NewNode.d)]
						if(T.g > NewNode.g):
							T.p = NewNode.p
							T.pxy = NewNode.pxy




def reverse_path(node,m,am):
	path = []
	path = [node]
	c=0
	while node.p != None:
		x,y = node.d
		c=c+1
		m[ymax-round(y)][round(x)] = 1
		node = node.p
		path.append(node)
	path.reverse()
	return path


def Vi(P,C):
	fourcc = cv2.VideoWriter_fourcc(*'mp4v')
	out = cv2.VideoWriter("Output.mp4", fourcc, 1500, (1000,1000))
	for x,y in C:
		am[ymax-round(y)][round(x)] = [255,255,255]
		out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)
	for n in P:
		x,y = n.d 
		am[ymax-round(y)][round(x)] = [0,0,255]
		out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)
	cv2.imshow("Final Path", am)
	cv2.waitKey(0)
	out.release()
	cv2.destroyAllWindows()


def getcost(goal, x,y):
	xg,yg = goal
	ec = (x-xg)**2 + (y-yg)**2
	return ec


def get_input(args,m,am):
	global cll
	global rr
	rr = 22
	cll = 10
	L = 10

	cll = cll + rr  

	x = int((float(args[0]) + 5)*100)
	y = int((float(args[1]) + 5)*100)
	sa = int(args[2])
	ga = 0
		
	print("Enter goal's x-coordinate:")
	xg = int((float(args[3]) + 5)*100)
	if (xg - rr < 0) or (xg + rr > xmax):
		print('Invalid x-coordinate.')
		xg = xmax - rr
		
	print("Enter goal's y-coordinate:")
	yg = int((float(args[4]) + 5)*100)
	if (yg - rr < 0) or (yg + rr > ymax):
		print('Invalid y-coordinate.')
		yg = ymax - rr 
	
	if detect(m,am,xg,yg):
		print('Invalid graph size.')
		xg = xmax - rr
		yg = ymax - rr
		
	return [x,y],[xg,yg],L,sa,ga


def graph(O,OP,P):
	fig, ax = plt.subplots()
	plt.grid()
	plt.xlim(0,400)
	plt.ylim(0,300)
	
	for c,p in zip(O,OP):
		x,y = c
		px,py = p
		ax.quiver(px,py,x-px,y-py,units='xy' ,scale=1, width = 0.5,headwidth = 10,headlength = 10)
	for z in P:
		x,y = z.d
		px,py = z.pxy
		ax.quiver(px,py,x-px,y-py,units='xy' ,color= 'r',scale=1,headwidth = 10,headlength = 10)
	ax.set_aspect('equal')
	plt.show()


def Vig(O,OP,P,m,am):
	fourcc = cv2.VideoWriter_fourcc(*'mp4v')
	out = cv2.VideoWriter("Output.mp4", fourcc, 20, (1000,1000))

	for c,p in zip(O,OP):
		x,y = c
		px,py = p
		cv2.arrowedLine(am, (round(px), round(ymax-py)), (round(x), round(ymax-y)),(0,155,255), 1, 1, 0, 0.1)
		out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)
	for z in P:
		x,y = z.d
		px,py = z.pxy
		cv2.arrowedLine(am, (round(px), round(ymax-py)), (round(x), round(ymax-y)),(0,255,0), 1, 1, 0, 0.1)
		out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)

	out.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	m,am = create_map()
	global cll
	global rr
	start = []
	global goal
	start,goal,L,sa,ga=get_input(m,am)
	root = Node(start, sa, 0 , None, 0, start)
	F,C,O,Pxy = DS(root,goal,L,ga,m,am)
	p=reverse_path(F,m,am)
	Vig(O,Pxy,p,m,am)
	plt.grid()

	ax.set_aspect('equal')

	plt.title('How to plot a vector in matplotlib?',fontsize=10)

	plt.show()
	plt.close()