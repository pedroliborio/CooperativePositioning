import numpy as np
import matplotlib.pyplot as plt

count = int(0)

tunnels = ['RIO450', 'DPT', 'DMAT', 'RCLT' , 'YBT']
colors = ['g', 'c', 'm', 'r' , 'b']

ways = ['ExitEntrance','EntranceExit']

for tunnel in tunnels:

	for way in ways:
		if(way == "ExitEntrance" and tunnel == "RIO450"):
			continue
		filePath = tunnel+way+".txt"
		print(filePath)
		listx, listy = np.loadtxt(filePath, delimiter='\t', unpack = True)
		print(len(listx))
		print(len(listy))

		plt.plot(listx, listy,label=tunnel+way, color = colors[count])#, linewidth=5)

	count+=1
	plt.xlabel("Longitude (x)")
	#plt.xticks(np.arange(1, 1000, 10))
	#plt.xscale('log')
	#plt.yscale('log')
	plt.ylabel("Latitude (y)")
	plt.legend(loc='upper left', bbox_to_anchor=(0.175, 1.016), fancybox=True, shadow=True, fontsize='small')

	fig = plt.gcf()
	#fig.suptitle('Error Per Cycle',y=0.98)
	plt.show()
	plt.draw()
	plt.grid(True)
	fig.savefig('graphs.png',dpi=200)

