#!/usr/bin/python2

import roslib
roslib.load_manifest('ros_brain', 0)

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import pygame
from pygame.locals import *
import numpy as np
# from rospy.numpy_msg import numpy_msg
# from rospy_tutorials.msg import Floats

# input: size of the square (int), and number of divisions (int)
# return: a list of cells' center point coordinates (np.array)
def init_grid(size, res):
	cell_size = size[0] / res
	cell_center = cell_size / 2
	cells = np.array([0, 0])
	cells = []
	for i in range(res):
		x = (i * cell_size + cell_center)
		for j in range(res):
			y = (j * cell_size + cell_center)
			cell = np.array([x, y])
			cells = np.append(cells, cell)
	cells = cells.reshape(-1, 2)
	return cells

# computes the distance between the mouse position and the grid cells
def compute_distances(pt, cloud):
	dists = -2 * np.dot(pt, cloud.T) + np.sum(cloud**2, axis=1) + \
			np.sum(pt**2, axis=1)[:, np.newaxis]
	dists = np.sqrt(dists)
	return dists

# create and configure a Float64MultiArray message
def prepare_msg(map):
	new_msg = Float64MultiArray()
	for dim_spec in map:
		new_dim = MultiArrayDimension()
		new_dim.label = dim_spec[0]
		new_dim.size = dim_spec[1]
		new_dim.stride = 0
		new_msg.layout.dim.append(new_dim)
	return new_msg

def mouse_reader():
	global compute_distances, init_grid
	debug = False
	pub = rospy.Publisher('sensor_data', Float64MultiArray, queue_size=20)
	rospy.init_node('mouse_reader', anonymous=True)
	rate = rospy.Rate(10)
	rospy.loginfo('publishing mouse position in rostopic sensor_data')
	cells = init_grid(size, res)
	my_msg = prepare_msg([['x',res],['y',res]])
	running = True

	while not rospy.is_shutdown() and running==True:
		for event in pygame.event.get():
			if event.type == QUIT:
				running = False
			if event.type == KEYDOWN and event.key == K_ESCAPE:
				rospy.loginfo('Quit')
				running = False

			# reading mouse coordinates
			mouse_x, mouse_y = pygame.mouse.get_pos()
			mouse = np.array([mouse_x, mouse_y])
			mouse = mouse.reshape(-1, 2)

			# distance of mouse to the cells
			dists = compute_distances(mouse, cells)
			dists = dists.reshape(res, res).T

			# remapping distances between 0 to 1
			bound_max = dists.max()
			dists_remapped = dists / bound_max

			# publishing to ros
			del my_msg.data[:]
			for dims in dists_remapped: #'flatten' the numpy array
				for val in dims:
					my_msg.data.append(val)

		if debug:
			rospy.loginfo('now:')
			for val in my_msg.data: rospy.loginfo(val)
		pub.publish(my_msg)
		rate.sleep()

	pygame.display.quit()

if __name__ == '__main__':
	try:
		size = (400, 400)
		res = 3
		pygame.init()
		pygame.display.set_mode(size)
		pygame.display.set_caption('Tracking Area')
		mouse_reader()
	except rospy.ROSInterruptException:
		pygame.display.quit()