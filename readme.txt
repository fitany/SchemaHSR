Hardware:
	Toyota Human Support Robot
Dependencies:
	Python 2.7
	Numpy
	PyQt5
	Rospy
Run the following commands to load COCO weights
	cd hsr_yolov3
	wget https://pjreddie.com/media/files/yolov3.weights
Experiment:
	main_ui.py - entry script for running the interfaces
	network.py - script for running neural network, majority of relevant code is here
Analysis:
	extractOutputs.py - parse saved weights from experiment runs into python objects
	plotPerformance.py - load python objects to plot performance
	plot_trajectories.py - plot robot trajectories from individual runs

hsr_yolov3 folder is largely adapted from https://github.com/marvis/pytorch-yolo2

