import numpy as np
import csv
import matplotlib.pyplot as plt

def readFromFile(fileName):
	data = []
	with open(fileName, 'rb') as f:
	    reader = csv.reader(f)
	    for row in reader:
	        data.append(row)
	return data

def plotRuntimes(data):
	plt.plot(data, 'r--')
	plt.show()

plotRuntimes(readFromFile('results_A.csv'))