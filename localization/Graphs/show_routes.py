import numpy as np
import matplotlib.pyplot as plt

count = int(0)

tunnels = ['RIO450', 'DPT', 'DMAT', 'RCLT' , 'YBT']
colors = ['g', 'c', 'm', 'k' , 'b']

ways = ['EntranceExit', 'ExitEntrance']

for tunnel in tunnels:

	for way in ways:
		if(way == "ExitEntrance" and tunnel == "RIO450"):
			continue
		
		filePath = "Graphs/out/"+tunnel+way+".txt"
		listx, listy = np.loadtxt(filePath, delimiter='\t', unpack = True)
		plt.plot(listx, listy,label=tunnel, color = colors[count],linewidth=3,zorder=1)#,s =30)
		plt.scatter(listx, listy,color = 'k',linewidth=3, s =30, zorder=2)

		filePathGPS = "alloutagesxy/"+tunnel+way+".txt"
		listxGPS, listyGPS = np.loadtxt(filePathGPS, delimiter='\t', unpack = True, skiprows=1, usecols=(1,2))
		plt.scatter(listxGPS, listyGPS,c='r',zorder=3)

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
	fig.savefig(tunnel+'.png',dpi=200)
	count+=1

	


