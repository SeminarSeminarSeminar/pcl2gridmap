import numpy as np
import math
import cv2
import os 
import sys

HEADER_LINE_NUM = 11
THRESHOLD1 = 8
THRESHOLD2 = 32
SCALE = 4
OFFSET = 10
file_path = '/home/alpha/Desktop/Project/pcl2gridmap/build/map_inliners.pcd'

fp = open(file_path, "r")

''' read pcd file'''
print("reading pcd file...")
pointNum = 0
for i in range(HEADER_LINE_NUM):
	line = fp.readline()
	tokens = line.split()
	if(tokens[0] == 'POINTS'):
		pointNum = int(tokens[1])

points = np.zeros((pointNum,3))
for i in range(pointNum):
	line = fp.readline()
	tokens = line.split()
	points[i] = list(map(float, tokens))

x_max = np.max(points[:,0]) # all x
x_min = np.min(points[:,0]) 
z_max = np.max(points[:,2]) # all y
z_min = np.min(points[:,2])

grid_size=32

scale_x = grid_size/(x_max-x_min)
scale_z = grid_size/(z_max-z_min)
bias_x = -x_min*scale_x
bias_z = -z_min*scale_z

gridsize=(grid_size+1, grid_size+1)
grid_map = np.zeros(gridsize)
for i in range(0,pointNum):
	grid_pos_x = int(scale_x*points[:,0][i]+bias_x)
	grid_pos_z = int(scale_z*points[:,2][i]+bias_z)
	grid_map[grid_pos_z][grid_pos_x]=255 #grid_map[grid_pos_z][grid_pos_x]+int(1)

for row in range(0, grid_size+1):
	for col in range(0, grid_size+1):
		print(str(int(grid_map[row][col]))+" ,",end='')
	print("\n")
cv2.imshow("window", grid_map)
cv2.waitKey(0)
'''
gridsize = np.array([int(z_max//SCALE + abs(x_min)//SCALE + 2*OFFSET)
						, int(x_max//SCALE + abs(x_min)//SCALE + 2*OFFSET)])
grid = np.zeros((gridsize))

print("(x max, x min) : ",x_max, x_min)
print("(z max, z min) : ",z_max, z_min)
print("grid size : ",gridsize)

# generate grid 
print("generating grid...")
for i in range(points.shape[0]):
	x,y,z= points[i].astype(int)
	x=int(x//SCALE + abs(x_min)//SCALE)
	z=int(z//SCALE + abs(z_min)//SCALE)
	grid[z+OFFSET][x+OFFSET]+=1

for r in range(len(grid)):
	for c in range(len(grid[r])):
		if grid[r][c] < THRESHOLD1:
			grid[r][c] = 0
		elif grid[r][c] < THRESHOLD2:
			grid[r][c] = 0.5
		else:
			grid[r][c] = 1

gridmap = np.concatenate((gridsize, grid.flatten())) 
gridmap.tofile(file_path +".bin")

grid = cv2.resize(grid, (500,500), interpolation = cv2.INTER_LINEAR)
cv2.imshow("window", grid)
cv2.waitKey(0)
'''