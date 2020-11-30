
import matplotlib.pyplot as plt
import math
import sys

class Points:
	def __init__(self):
		self.src_x = []
		self.src_y = []
		self.dst_x = []
		self.dst_y = []
		self.lat_err = []
		
	def load(self,file_name):
		with open(file_name,'r') as f:
			lines = f.readlines()
		
		is_src = True
		
		for line in lines:
			if(line[0] == '-'):
				is_src = False
				continue
			elif(len(line) == 0):
				break
				
			msg = line.split()
			if(len(msg) == 0):
				break
			if(is_src):
				self.src_x.append(float(msg[0]))
				self.src_y.append(float(msg[1]))
			else:
				self.dst_x.append(float(msg[0]))
				self.dst_y.append(float(msg[1]))
				self.lat_err.append(float(msg[2]))
		
		print(len(self.src_x), len(self.dst_x))
			
	def plot(self):
		plt.plot(self.src_x, self.src_y, "r")
		plt.plot(self.dst_x, self.dst_y, "b")
		plt.show()
	
	
	def __disBetweenPoints(self,i,j):
		x = self.x[i] - self.x[j]
		y = self.y[i] - self.y[j]
		return math.sqrt(x*x+y*y)
			

def main(argv):
	raw_file = argv[1]
		
	path_points = Points()
	path_points.load(raw_file)
	
	path_points.plot()


if __name__ == '__main__':
	main(sys.argv)
