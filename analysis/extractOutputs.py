from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QDialog, QGridLayout, QGroupBox, QPushButton, QLabel, QLineEdit, QSizePolicy
import numpy as np
import sys
sys.path.insert(0, '..')
from network import *

app = QApplication(sys.argv)
schemaNetwork = SchemaNetwork()

#schemaNetwork.loadWeights()
#testPA(self,cue,has_hipp,prompting=False)

# Experiment 1A
correct_schema = np.asarray([[6,8],[21,2],[18,0]])
exp1a_results = {}
for trial in range(5): # trials
	performances = np.zeros(5)
	for subject in range(1,6): # subjects
		weights = '../saved_weights/subj'+str(subject)+'_exp1a_trial'+str(trial)
		print 'loading '+weights
		schemaNetwork.loadWeightsFromFilename(weights)
		performances[subject-1] = schemaNetwork.testSchema(correct_schema,subject,'classroom')
	exp1a_results[trial] = np.mean(performances),np.std(performances)/np.sqrt(5)
print exp1a_results

# Experiment 1B
correct_schema = np.asarray([[11,5],[21,2],[18,0]])
performances = np.zeros(5)
for subject in range(1,6): # subjects
	weights = '../saved_weights/subj'+str(subject)+'_exp1b_trial1'
	print 'loading '+weights
	schemaNetwork.loadWeightsFromFilename(weights)
	performances[subject-1] = schemaNetwork.testSchema(correct_schema,subject,'classroom')
exp1b_result = np.mean(performances),np.std(performances)/np.sqrt(5)
print exp1b_result

# Experiment 2
correct_schema = np.asarray([[31,0],[42,6],[32,3]])
exp2_results = {}
for trial in range(5): # trials
	performances = np.zeros(5)
	for subject in range(1,6): # subjects
		weights = '../saved_weights/subj'+str(subject)+'_exp2_trial'+str(trial)
		print 'loading '+weights
		schemaNetwork.loadWeightsFromFilename(weights)
		performances[subject-1] = schemaNetwork.testSchema(correct_schema,subject,'breakroom')
	exp2_results[trial] = np.mean(performances),np.std(performances)/np.sqrt(5)
print exp2_results
correct_schema = np.asarray([[6,8],[21,2],[18,0]])
performances = np.zeros(5)
for subject in range(1,6): # subjects
	weights = '../saved_weights/subj'+str(subject)+'_exp2_trial'+str(trial)
	print 'loading '+weights
	schemaNetwork.loadWeightsFromFilename(weights)
	performances[subject-1] = schemaNetwork.testSchema(correct_schema,subject,'classroom')
exp2_classroom_result = np.mean(performances),np.std(performances)/np.sqrt(5)
print exp2_classroom_result
correct_schema = np.asarray([[11,5],[21,2],[18,0]])
performances = np.zeros(5)
for subject in range(1,6): # subjects
	weights = '../saved_weights/subj'+str(subject)+'_exp2_trial'+str(trial)
	print 'loading '+weights
	schemaNetwork.loadWeightsFromFilename(weights)
	performances[subject-1] = schemaNetwork.testSchema(correct_schema,subject,'classroom')
exp2_novel_result = np.mean(performances),np.std(performances)/np.sqrt(5)
print exp2_novel_result

# Experiment 3 - Heat Map
schemaNetwork.loadWeightsFromFilename('../saved_weights/subj1_exp2_trial4')
# Banana
schemaNetwork.current_context_items =['banana']
heatmap_banana = schemaNetwork.testPA(1,True,True)
print heatmap_banana
# Book
schemaNetwork.current_context_items =['book']
heatmap_book = schemaNetwork.testPA(7,True,True)
print heatmap_book

# Save Outputs
data = [exp1a_results, exp1b_result, exp2_results, exp2_classroom_result, exp2_novel_result, heatmap_banana, heatmap_book]
outfile = open('analysis_results','wb+')
pickle.dump(data,outfile)
outfile.close()