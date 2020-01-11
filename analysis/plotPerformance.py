import matplotlib.pyplot as plt
import numpy as np
import pickle

plt.rcParams.update({'font.size': 20})

infile = open('analysis_results','rb')
data = pickle.load(infile)
infile.close()

exp1a_results = data[0]
exp1b_result = data[1]
exp2_results = data[2]
exp2_classroom_result = data[3]
exp2_novel_result = data[4]
heatmap_banana = data[5]
heatmap_book = data[6]

exp1a_results_behavioral = {0:(159.43,18.40),4:(90.53,2.84)}
exp1b_result_behavioral = (85.61,14.33)
exp2_results_behavioral = {0:(166.03,13.68),4:(107.29,3.85)}
exp2_classroom_result_behavioral = (98.36,11.95)
exp2_novel_result_behavioral = (89.606,9.72)

# Experiment 1 Plot
fig, ax1 = plt.subplots()
x=[0,1,2,3,4,5]
y=[]
errs=[]
for key in sorted(exp1a_results.keys()):
	y.append(exp1a_results[key][0])
	errs.append(exp1a_results[key][1])
y.append(exp1b_result[0])
errs.append(exp1b_result[1])
ax1.errorbar(x, y, yerr=errs, fmt='b',elinewidth=3, linewidth=3)
ax1.plot([4.5,4.5],[0,1.1],'black')
ax1.text(2,.03,'Exp 1a')
ax1.text(4.52,.03,'Exp 1b')
ax1.set_xlabel('Trial')
# Make the y-axis label, ticks and tick labels match the line color.
ax1.set_ylabel('% Activation of correct output neuron', color='b')
ax1.tick_params('y', colors='b')
plt.ylim([0,1.1])

ax2 = ax1.twinx()
x=[0,4,5]
y=[]
errs=[]
for key in sorted(exp1a_results_behavioral.keys()):
	y.append(exp1a_results_behavioral[key][0])
	errs.append(exp1a_results_behavioral[key][1])
y.append(exp1b_result_behavioral[0])
errs.append(exp1b_result_behavioral[1])
ax2.errorbar(x, y, yerr=errs, fmt='r',elinewidth=3, linewidth=3, ecolor='red')
ax2.set_ylabel('Time to retrieve (s)', color='r')
ax2.tick_params('y', colors='r')
plt.xlim([-.5,5.6])
fig.tight_layout()
plt.title('Performance on Experiment 1')
plt.show()

# Experiment 2 Plot
fig, ax1 = plt.subplots()
x=[0,1,2,3,4,5,6]
y=[]
errs=[]
for key in sorted(exp2_results.keys()):
	y.append(exp2_results[key][0])
	errs.append(exp2_results[key][1])
y.append(exp2_classroom_result[0])
errs.append(exp2_classroom_result[1])
y.append(exp2_novel_result[0])
errs.append(exp2_novel_result[1])
ax1.errorbar(x, y, yerr=errs, fmt='b',elinewidth=3, linewidth=3)
ax1.plot([4.5,4.5],[0,1.1],'black')
ax1.plot([5.5,5.5],[0,1.1],'black')
ax1.text(2,.03,'Exp 2')
ax1.text(4.8,.03,'CR')
ax1.text(5.54,.03,'Novel')
ax1.set_xlabel('Trial')
# Make the y-axis label, ticks and tick labels match the line color.
ax1.set_ylabel('% Activation of correct output neuron', color='b')
ax1.tick_params('y', colors='b')
plt.ylim([0,1.1])

ax2 = ax1.twinx()
x=[0,4,5,6]
y=[]
errs=[]
for key in sorted(exp2_results_behavioral.keys()):
	y.append(exp2_results_behavioral[key][0])
	errs.append(exp2_results_behavioral[key][1])
y.append(exp2_classroom_result_behavioral[0])
errs.append(exp2_classroom_result_behavioral[1])
y.append(exp2_novel_result_behavioral[0])
errs.append(exp2_novel_result_behavioral[1])
ax2.errorbar(x, y, yerr=errs, fmt='r',elinewidth=3, linewidth=3, ecolor='red')
ax2.set_ylabel('Time to retrieve (s)', color='r')
ax2.tick_params('y', colors='r')
plt.xlim([-.5,6.5])
fig.tight_layout()
plt.title('Performance on Experiment 2')
plt.show()

# Experiment 3 Plot
# Heatmaps
'''
plt.subplot(2,2,1)
plt.imshow(heatmap_banana[:25].reshape([5,5],order='C').T)
plt.axis('off')
plt.subplot(2,2,2)
plt.imshow(heatmap_banana[25:].reshape([5,5],order='C').T)
plt.axis('off')
plt.subplot(2,2,3)
plt.imshow(heatmap_book[:25].reshape([5,5],order='C').T)
plt.axis('off')
plt.subplot(2,2,4)
plt.imshow(heatmap_book[25:].reshape([5,5],order='C').T)
plt.axis('off')
plt.show()
'''