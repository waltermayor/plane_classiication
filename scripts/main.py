#!/usr/bin/python
#! -*- encoding: utf-8 -*-

import os
import subprocess
import sys
import shutil
import liblas
import dtm
import conversions


print ("setting paths")
current=os.getcwd()
os.chdir("../")
PATH_TO_PROJECT=os.getcwd()
PATH_TO_3LIBRARIES=PATH_TO_PROJECT+"/src_build/bin"

input_dataset=PATH_TO_PROJECT+"/dataset_input/"
output_path=PATH_TO_PROJECT+"/output/"
os.chdir(current)

			
if __name__ == "__main__":

	topo=dtm.dtm()
	#conver=conversions.pointCloudConvertions()
	topo.dtmOp3(sys.argv[1])
	#conver.las2pcd("dtm.las","dtm.pcd")
	p2 = subprocess.Popen([os.path.join(PATH_TO_3LIBRARIES,
						"classificationMap"),input_dataset+"pcd/"+"dtm.pcd"])
	p2.wait()


