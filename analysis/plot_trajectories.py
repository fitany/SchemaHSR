import matplotlib.pyplot as plt
def parseTrajectory(filename):
	f = open(filename)
	x = []
	y = []
	for line in f:
		elts = line.split(',')
		x.append(float(elts[0]))
		y.append(float(elts[1]))
	return x,y
x1, x2 = -6.5,6.5
y1, y2 = -8.0,8.0

plt.imshow(plt.imread('../lab-map/map_cropped_tagged.pgm'),extent=[x1, x2, y1, y2], aspect='auto', cmap='gray')
#x,y = parseTrajectory('../trajectories/subj4_teddy_trial0.txt')
#plt.plot(x,y,'y')
x,y = parseTrajectory('../trajectories/subj4_bottle_trial4.txt')
plt.plot(x,y,'r')
plt.axis('scaled')
plt.ylim([-8,4])
#plt.ylim([-1.5,4])
#plt.xlim([-3.4,3.6])
plt.show()

plt.imshow(plt.imread('../lab-map/map_cropped_tagged.pgm'),extent=[x1, x2, y1, y2], aspect='auto', cmap='gray')
#x,y = parseTrajectory('../trajectories/subj5_cup_trial0.txt')
#plt.plot(x,y,'y')
x,y = parseTrajectory('../trajectories/subj3_apple_trial4.txt')
for i in range(len(x)):
	x[i]-=.2

plt.plot(x,y,'g')
plt.axis('scaled')
plt.ylim([-8,4])
#plt.ylim([-7,-3])
#plt.xlim([-5.2,.2])
plt.show()

plt.imshow(plt.imread('../lab-map/map_cropped_tagged.pgm'),extent=[x1, x2, y1, y2], aspect='auto', cmap='gray')
x,y = parseTrajectory('../trajectories/subj1_banana.txt')
plt.plot(x,y,'y')
x,y = parseTrajectory('../trajectories/subj1_book.txt')
plt.plot(x,y,'b')
plt.axis('scaled')
plt.ylim([-8,4])
plt.show()