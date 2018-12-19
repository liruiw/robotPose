#!/usr/bin/env python
import vtk
import numpy as np
import cv2
import sys
sys.path.append("/usr/local/lib/python2.7/site-packages/")
import pymeshfix
import vtk
import math
from vtk.util import numpy_support
import cv2
from project_util import *
import convertPoly
class Camera_VTK():
	"""
	class to synthesize robot models
	"""
	def __init__(self, name='baxter', visualize=False): #K, near, far, extrinsics can be inputs.
		self.w = 640
		self.h = 480
		self.K = np.eye(3)
		self.K[0,0] = 525 #513.3605800901698
		self.K[1,1] = 525 #516.1600538269478
		self.K[0,2] = 319.5 #313.308307869799
		self.K[1,2] = 239.5 #261.3256090654382
		self.near = 0.001 #1cm
		self.far = 1000 #m
		self.f = np.array( [self.K[0,0], self.K[1,1]] ) # focal lengths
		self.c = self.K[:2,2] # principal point
		self.ren = vtk.vtkRenderer()
		self.renWin = vtk.vtkRenderWindow()
		self.vis = visualize
		self.iren = vtk.vtkRenderWindowInteractor()
		self.actor_list = []
		self.axes_list =  []
		self.name = name
		if self.name == 'baxter':
			self.base_link = 'right_arm_mount'
			self.file_list = ['S0', 'S1', 'E0', 'E1', 'W0', 'W1', 'W2']#['E0','S0','W0','E1','S1','W1','W2','G1','G2']
		elif self.name == 'panda_arm':
			self.base_link='panda_link0'
			self.file_list = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7']
		self.transform_list = [] # new*old^-1 would be the relative transform
		self.init_vtk()
		self.param=[int(cv2.IMWRITE_PNG_COMPRESSION), 5]
		self.scene_flow_list = {} #index represent the relative flow


	def init_vtk(self):
		"""
		Initialize VTK actors and rendering pipeline
		"""
		self.renWin.SetSize(self.w, self.h)
		self.renWin.SetOffScreenRendering(1) # turn on in test
		self.renWin.AddRenderer(self.ren)
		if self.vis:
			self.renWin.SetOffScreenRendering(0) # turn on in test
			self.iren.SetRenderWindow(self.renWin)
			self.iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
		self.ren.ResetCamera()
		self.set_intrinsics()
		self.initialize_transform()
		self.initialize_actor()
		#self.initialize_light()

	def initialize_actor(self): #render initial stl files
		for i in range(len(self.transform_list)):
			file = self.file_list[i%len(self.file_list)]
			name = self.append_abs_name(file)
			actorList = convertPoly.read_dae(name)
			self.actor_list.append(actorList)
			axes = vtk.vtkAxesActor()
			self.axes_list.append(axes)
			axes.SetTotalLength([0.2,0.2,0.2])
		self.add_actor()

	def initialize_light(self, position=[-1, -1.5, 1]):
	    light = vtk.vtkLight()
	    light.SetLightTypeToSceneLight()
	    light.SetFocalPoint(-0.4, 0.3, 1.4)
	    light.SetPosition(position[0],position[1],position[2])
	    light2 = vtk.vtkLight()
	    light2.SetLightTypeToSceneLight()
	    light2.SetFocalPoint(-0.3, 0.3, 2.2)
	    light2.SetPosition(position[0],position[1],position[2])
	    # light.SetIntensity(1)
	    # light.SetConeAngle(40)
	    self.ren.AddLight(light)
	    self.ren.AddLight(light2)		

	def add_actor(self): #one part is composed of multiple actors
		for actors in self.actor_list:
			for actor in actors:
				self.ren.AddActor(actor)
		for actor in self.axes_list:
			self.ren.AddActor(actor)

	def initialize_transform(self): #put identity transform for each of the file
		R = np.eye(3)
		t = np.zeros(3)
		Id = np.empty((4, 4))
		Id[:3, :3] = R
		Id[:3, 3] = t
		Id[3, :] = [0, 0, 0, 1]
		for i in range(len(self.file_list)):
			self.transform_list.append(np.copy(Id))

	def set_intrinsics(self): #needs to be checked
		cam = self.ren.GetActiveCamera()

		cam.SetClippingRange(self.near, self.far)
		cam.SetPosition(0, 0, 0) #need to reset focal point
		# cam.SetFocalPoint(1, 0, 0) #in robot coordinates, adjusted
		# cam.SetViewUp(0, 0, 1)
		cam.SetViewUp(0, -1, 0)
		cam.SetFocalPoint(0, 0, 1) #here modify to be original
		w,h = self.w, self.h
		# Set window center for offset principal point
		wcx = -2.0*(self.c[0] - self.w / 2.0) / self.w
		wcy = 2.0*(self.c[1] - self.h / 2.0) / self.h
		cam.SetWindowCenter(wcx, wcy)

		# Set vertical view angle as a indirect way of setting the y focal distance
		angle = 180 / np.pi * 2.0 * np.arctan2(self.h / 2.0, self.f[1]) 
		cam.SetViewAngle(angle)

		# Set the image aspect ratio as an indirect way of setting the x focal distance
		m = np.eye(4)
		aspect = self.f[1]/self.f[0]
		m[0,0] = 1.0/aspect
		t = vtk.vtkTransform()
		t.SetMatrix(m.flatten())

		cam.SetUserTransform(t)


	def append_abs_name(self,model_name):
 		return '%s_models/%s.DAE'%(self.name,model_name)

	def render_pose(self,transformList, id=-1):
		# Set basic camera parameters in VTK added coordinates		
		for i in range(len(self.transform_list)):
			if id == -1:
				old_transform = self.transform_list[i]
				new_transform = ros2np(transformList[i])
				new_vtk_transform = compute_vtk_transform(new_transform)
				self.transform_list[i] = np.copy(new_transform)
				self.setUserTransform(self.actor_list[i],new_vtk_transform)
				self.axes_list[i].SetUserTransform(new_vtk_transform)
				self.axes_list[i].VisibilityOff()
				for actors in self.actor_list[i]:
					actors.VisibilityOn()

			else:
				if i not in id:
					self.axes_list[i].VisibilityOff()
					for actors in self.actor_list[i]:
						actors.VisibilityOff()
				else:
					old_transform = self.transform_list[i]
					new_transform = ros2np(transformList[i])
					new_vtk_transform = compute_vtk_transform(new_transform)
					self.transform_list[i] = np.copy(new_transform)
					self.setUserTransform(self.actor_list[i],new_vtk_transform)
					self.axes_list[i].SetUserTransform(new_vtk_transform)
					self.axes_list[i].VisibilityOff()
					for actors in self.actor_list[i]:
						actors.VisibilityOn()
					
		self.renWin.Render()
		if self.vis:
			self.iren.Start()#this thing stuck the flow, is an event loop
		return self.render_image()
	
	def setUserTransform(self, actors, transform):
		for actor in actors:
			actor.SetUserTransform(transform)


	def render_image(self):
		#display the depth and color screenshots from the camera perspective, and return np arrays
		winToIm = vtk.vtkWindowToImageFilter()
		winToIm.SetInput(self.renWin)
		winToIm.Update()
		vtk_image = winToIm.GetOutput()
		width, height, _ = vtk_image.GetDimensions()
		vtk_array = vtk_image.GetPointData().GetScalars()
		components = vtk_array.GetNumberOfComponents()
		arr = cv2.flip(numpy_support.vtk_to_numpy(vtk_array).reshape(height, width, components), 0)
		arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
		winToDepth = vtk.vtkWindowToImageFilter()
		winToDepth.SetInputBufferTypeToZBuffer()
		winToDepth.SetInput(self.renWin)
		winToDepth.Update()
		vtk_depth = winToDepth.GetOutput()
		width, height, _ = vtk_depth.GetDimensions()
		depth_array = vtk_depth.GetPointData().GetScalars()
		components = depth_array.GetNumberOfComponents()
		z_arr = cv2.flip(numpy_support.vtk_to_numpy(depth_array).reshape(height, width, components), 0)
		depth_arr = z_buffer_to_depth(z_arr, self.near, self.far)
		zm = depth_arr[np.nonzero(depth_arr)]
		# cv2.imshow("test_image", arr)
		# cv2.waitKey(1)
		# cv2.imshow("test_depth", z_arr) #switch the showing to the caller
		#if True: #cv2.waitKey(1) == 32: # just for testing
			#import os
			#cv2.imwrite(file_name, depth_arr, self.param)
			#cv2.imwrite(file_name, arr, self.param)
		return arr, depth_arr
