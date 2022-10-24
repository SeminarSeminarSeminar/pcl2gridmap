import numpy as np
import math
import cv2
import os 
import sys
from PIL import Image
from matplotlib import pyplot as plt

HEADER_LINE_NUM = 11
THRESHOLD1 = 8
THRESHOLD2 = 32
SCALE = 1
OFFSET = 10

def convert_pcl2grid(points):
	print("convert pcl to grid...")
	x_max = np.max(points[:,0])
	x_min = np.min(points[:,0])
	z_max = np.max(points[:,2])
	z_min = np.min(points[:,2])

	gridsize = np.array([int(z_max//SCALE + abs(z_min)//SCALE + 2*OFFSET), int(x_max//SCALE + abs(x_min)//SCALE +2*OFFSET),3])
	counter = np.zeros([int(z_max//SCALE + abs(z_min)//SCALE + 2*OFFSET), int(x_max//SCALE + abs(x_min)//SCALE+2*OFFSET)])
	grid = np.zeros((gridsize),dtype=np.uint8)

	for i in range(points.shape[0]):
		x,y,z= points[i].astype(int)
		x=int(x//SCALE + abs(x_min)//SCALE)
		z=int(z//SCALE + abs(z_min)//SCALE)
		counter[z+OFFSET][x+OFFSET]+=1
	
	for r in range(len(grid)):
		 for c in range(len(grid[r])):
			 if counter[r,c] < THRESHOLD1:
				 grid[r,c] = np.array([0,0,0])
			 elif counter[r,c] < THRESHOLD2:
				 grid[r,c] = np.array([128,128,128])
			 else:
				 grid[r,c] = np.array([255,255,255])
	
	return grid

def load_map(file_path):	
	print("reading pcd file...")
	fp = open(file_path, "r")

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

	return points

def load_frame(file_path):
	print("reading frame file...")
	fp = open(file_path, "r")

	frame_pos = []
	while(True):
		line = fp.readline()
		if not line: break
		tokens = line.split()
		frame_id = tokens[0]
		frame_pos.append([tokens[4], tokens[4+4], tokens[4*3]])
	frame_pos = np.array(frame_pos,np.float32)
	frame_pos = np.reshape(frame_pos, (-1, 3))
	return frame_pos


def main():
	map_file_path = sys.argv[1]
	frame_file_path = sys.argv[2]

	pcl_map = load_map(map_file_path)
	grid_map = convert_pcl2grid(pcl_map)

	frame_pos = load_frame(frame_file_path)
	x_max = np.max(pcl_map[:,0])
	x_min = np.min(pcl_map[:,0])
	z_max = np.max(pcl_map[:,2])
	z_min = np.min(pcl_map[:,2])

	frame_x_max = np.max(frame_pos[:,0])
	frame_x_min = np.min(frame_pos[:,0])
	frame_y_max = np.max(frame_pos[:,1])
	frame_y_min = np.min(frame_pos[:,1])
	frame_z_max = np.max(frame_pos[:,2])
	frame_z_min = np.min(frame_pos[:,2])


	print("map", x_max, x_min, z_max, z_min)
	print("frame", frame_x_max, frame_x_min, frame_y_max, frame_y_min, frame_z_max, frame_z_min)
	for point in frame_pos:
		x,y,z = point
		x = int(x//SCALE + abs(x_min)//SCALE)
		z = int(z//SCALE + abs(z_min)//SCALE)
		grid_map[z+OFFSET, x+OFFSET] = [255,0,0]
	
	img = Image.fromarray(grid_map)
	plt.imshow(grid_map)
	plt.show()

if __name__ == "__main__":
	main()

