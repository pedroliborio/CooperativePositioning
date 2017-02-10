import numpy as np
import matplotlib.pyplot as plt

count = int(0)
count2 =int(0)

tunnels = ['RIO450', 'DPT', 'DMAT', 'RCLT' , 'YBT']
colors = ['g', 'c', 'm', 'r' , 'b']

ways = ['EntranceExit', 'ExitEntrance']

for tunnel in tunnels:

	for way in ways:
		if(way == "ExitEntrance" and tunnel == "RIO450"):
			continue
		filePath = "../outagesxy/"+tunnel+way+".txt"
		print(filePath)
		listError = np.loadtxt(filePath, delimiter='\t', unpack = True, usecols=(3,), skiprows=1)
		f = open(tunnel+way+".txt","w")
		f.write( str(np.mean(listError))+'\t'+str(np.std(listError))+'\n' )
		plt.errorbar(count2, np.mean(listError), np.std(listError), label=tunnel+way, color = colors[count], fmt='--o')#, linewidth=5)
		count2+=1

	count+=1
plt.xlabel("Longitude (x)")
plt.xticks(np.arange(-1,10))
#plt.xscale('log')
#plt.yscale('log')
plt.ylabel("Latitude (y)")
plt.legend(loc='upper left', bbox_to_anchor=(0.175, 1.016), fancybox=True, shadow=True, fontsize='small')

fig = plt.gcf()
#fig.suptitle('Error Per Cycle',y=0.98)
#plt.show()
plt.draw()
plt.grid(True)
fig.savefig('mean_std.png',dpi=200)