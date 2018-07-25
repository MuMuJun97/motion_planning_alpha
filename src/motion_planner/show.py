import matplotlib.pyplot as plt

def showpath(filename1, filename2, filename3):
	with open(filename1) as f1:
		x = []
		y = []
		for line in f1.readlines():
			x.append(float(line.strip().split(", y:")[0][3:]))
			y.append(float(line.strip().split(", y:")[1]))
		for i in range(len(x)-1):
			plt.plot([x[i],x[i+1]], [y[i],y[i+1]])
	with open(filename2) as f2:
		x = []
		y = []
		for line in f2.readlines():
			x.append(float(line.strip().split(", y: ")[0][3:]))
			y.append(float(line.strip().split(", y: ")[1]))
		for i in range(len(x)):
			plt.scatter([x[i]], [y[i]], c='r', marker='v')
	with open(filename3) as f3:
		x = []
		y = []
		for line in f3.readlines():
			x.append(float(line.strip().split(" ")[0]))
			y.append(float(line.strip().split(" ")[1]))
		for i in range(len(x)-1):
			plt.plot([x[i],x[i+1]], [y[i],y[i+1]], 'r')
	plt.savefig('path')

if __name__ == '__main__':
	showpath("data.txt", "pathnode.txt", "referpath.txt")

