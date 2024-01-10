import torch
import math 
import pickle
import numpy as np
import scipy.io as sio
import os
# from matplotlib import pyplot as plt

def DeepCFD_output_eval(sx, sy, lpx, lpy):
	# sx and sy are the source coordinates 
	# px and py are the coordinates of the point of interest
	
	device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	
	# map number 
	#print(device)
	f = open('/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/webots/data/map_nbr.txt', 'r')
	m = int(f.read())
	f.close()
	# print("map nbr {}".format(m))

	# load the saved model
	# DeepCFD = torch.load("myDeepCFD_model_Cavg_3input_noWind.pt") 
	DeepCFD = torch.load("DDPM.pt")
	if torch.cuda.is_available():
		DeepCFD.cuda()
	DeepCFD.eval()
	
	# when no source position is given
	if(sx==-1 and sy==-1):
		adr = "/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/test_maps/map{}/test_maps_x_for_MCMC.mat".format(m)
		mat_x = sio.loadmat(adr)
		array_x = mat_x.get('py_x')
		double_x = torch.from_numpy(array_x)
		x = double_x.float()
		x = x[:,:3,:,:]
		r = torch.from_numpy(mat_x.get('X')).float()
		c = torch.from_numpy(mat_x.get('Y')).float()
	else:
		# make the input tensors using sx sy
		adr = "/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/test_maps/map{}/test_map{}_flowdef_sdf_emptysourcepos.mat".format(m,m)
		mat_x = sio.loadmat(adr)
		array_x = mat_x.get('py_x')
		double_x = torch.from_numpy(array_x)
		x = double_x.float()
		# x = x[:,1:4,:,:]
		# print("making source dist map")
		r = torch.from_numpy(mat_x.get('X')).float()
		c = torch.from_numpy(mat_x.get('Y')).float()
		x[0,2,:,:] = 1/np.sqrt( (sx-r)**2 + (sy-c)**2 )

	# get the output maps
	out = DeepCFD(x.to(device))
	out = out.cpu().detach().numpy()
	
	# get the values at (px, py)
	if(isinstance(lpx, float) or isinstance(lpx, int)):
		# print("int")
		idx_r = int(round((lpx-min(r[0]).item()) / (r[0,1]-r[0,0]).item()))
		idx_c = int(round((lpy-min(c[:,0]).item()) / (c[1,0]-c[0,0]).item()))
		c_avg = [out[:, 0, idx_c,idx_r].tolist()]
		c_std = [0] #out[0, 1, idx_r,idx_c]
	elif(isinstance(lpx, list) or isinstance(lpx, tuple) or isinstance(lpx, np.ndarray)):
		# print("list or tuple of size", len(lpx))
		c_avg = [[]]*len(lpx)
		c_std = [[]]*len(lpx)
		for i in range(len(lpx)):
			# get the values at (px, py)
			idx_r = int(round((lpx[i]-min(r[0]).item()) / (r[0,1]-r[0,0]).item()))
			idx_c = int(round((lpy[i]-min(c[:,0]).item()) / (c[1,0]-c[0,0]).item()))
			c_avg[i]=out[:, 0, idx_c,idx_r].tolist()
			c_std[i]=[0] #out[0, 1, idx_r,idx_c]
			# print("(",lpx[i], lpy[i],") -> (", idx_r, idx_c, ") -> ", out[0, 0, idx_r,idx_c])
	else:
		print("apply_DeepCFD: Bad input format.")
	return c_avg, c_std

if __name__ == "__main__":
	c_avg, c_std = DeepCFD_output_eval(3.224360,2.772563,4.511471,2.830724)
	print("c_avg = ", c_avg,"c_std = ", c_std)