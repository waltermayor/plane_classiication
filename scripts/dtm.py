#!/usr/bin/python
#! -*- encoding: utf-8 -*-

import os
import subprocess
import sys
import shutil
import liblas


class dtm():

	def __init__(self):

		print ("setting paths")
		os.chdir("../")
		self.PATH_TO_PROJECT=os.getcwd()
		self.PATH_TO_3LIBRARIES=self.PATH_TO_PROJECT+"/src_build/bin"
		
		self.input_dataset=self.PATH_TO_PROJECT+"/dataset_input/"
		self.output_path=self.PATH_TO_PROJECT+"/output/"
		#self.output_dir=self.input_dir+"/reconstruction"

	
	def dtmOp3(self,lazFile):

		#wine las2dem -i b1.las -o demx.laz -keep_class 2 -step 0.5

		p1 = subprocess.Popen(["wine",os.path.join(self.PATH_TO_3LIBRARIES,
						"las2dem.exe"),"-i",self.input_dataset+"laz/"+lazFile,
						"-o",self.input_dataset+"laz/"+"dtm.laz","-keep_class","2","-step","0.5"])

		p1.wait()

		print ("Compute conversion laz to las")
		p2 = subprocess.Popen(["wine",os.path.join(self.PATH_TO_3LIBRARIES,
						"laszip.exe"),"-i",self.input_dataset+"laz/"+"dtm.laz",
						"-o",self.input_dataset+"las/"+"dtm.las"])
		p2.wait()	


		print ("Compute conversion las to pcd")
		p3 = subprocess.Popen([os.path.join(self.PATH_TO_3LIBRARIES,
						"las2pcd"),
						self.input_dataset+"las/"+"dtm.las",self.input_dataset+"pcd/"+"dtm.pcd"])
		p3.wait()	

		p4 = subprocess.Popen(["wine",os.path.join(self.PATH_TO_3LIBRARIES,
						"lasview.exe"),
						"-i",self.input_dataset+"las/"+"dtm.las"])
		p4.wait()


		
	
if __name__ == "__main__":

	topo=dtm()

	#topo.dtmOp2(sys.argv[1])
	topo.dtmOp3(sys.argv[1])
	


