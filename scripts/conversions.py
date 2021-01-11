#!/usr/bin/python
#! -*- encoding: utf-8 -*-

import os
import subprocess
import sys
import shutil
import liblas


class pointCloudConvertions():

	def __init__(self):

		print ("setting paths")
		self.actual=os.getcwd()
		os.chdir("../")
		self.PATH_TO_PROJECT=os.getcwd()
		self.PATH_TO_3LIBRARIES=self.PATH_TO_PROJECT+"/src_build/bin"
		

		self.input_dataset=self.PATH_TO_PROJECT+"/dataset_input/"
		self.output_path=self.PATH_TO_PROJECT+"/output/"
		os.chdir(self.actual)
		#self.output_dir=self.input_dir+"/reconstruction"

	def las2pcd(self,lasFile,pcdName):
		print ("Compute conversion las to pcd")
		#shutil.copy2(matches_dir+"/sfm_data.json", PATH_TO_PROJECT+"/sfm_data.json")
		pLas2pcd = subprocess.Popen([os.path.join(self.PATH_TO_3LIBRARIES,
						"las2pcd"),
						self.input_dataset+"las/"+lasFile,self.output_path+"pcd/"+pcdName])
		pLas2pcd.wait()	


	def laz2las(self,lazFile,lasName):
		print ("Compute conversion laz to las")
		#shutil.copy2(matches_dir+"/sfm_data.json", PATH_TO_PROJECT+"/sfm_data.json")
		pLaz2las = subprocess.Popen(["wine",os.path.join(self.PATH_TO_3LIBRARIES,
						"laszip.exe"),"-i",self.input_dataset+"laz/"+lazFile,
						"-o",self.output_path+"las/"+lasName])
		pLaz2las.wait()	


	def xyz2las(self,xyzFile,lasName):
		print ("Compute conversion xyz to las")
		#shutil.copy2(matches_dir+"/sfm_data.json", PATH_TO_PROJECT+"/sfm_data.json")
		pxyz2las = subprocess.Popen(["wine",os.path.join(self.PATH_TO_3LIBRARIES,
						"txt2las.exe"),"-i",
						self.input_dataset+"xyz/"+xyzFile,"-o",self.output_path+"las/"+lasName,"-parse xyz"])
		pxyz2las.wait()	
		
				
 
if __name__ == "__main__":

	conv=pointCloudConvertions()
	if len(sys.argv)>1:
		if sys.argv[1]=="-h":
			print("---------------------------help---------------------------------")
			print("execute as: python conversions.py convertionType lasFile pcdName")
			print("convertionType: las2pcd")
			print("----------------------------------------------------------------")

		elif sys.argv[1]=="--help":
			print("---------------------------help---------------------------------")
			print("execute as: python conversions.py convertionType lasFile pcdName")
			print("convertionType: las2pcd")
			print("----------------------------------------------------------------")

		elif sys.argv[1]=="las2pcd":
			if len(sys.argv)==4:
				conv.las2pcd(sys.argv[2],sys.argv[3])
			else:
				print("press -h or --help")

		elif sys.argv[1]=="laz2las":
			if len(sys.argv)==3:
				conv.laz2las(sys.argv[2],sys.argv[3])
			else:
				print("press -h or --help")

		else:
			print("press -h or --help")

	else:
		print("press -h or --help")


