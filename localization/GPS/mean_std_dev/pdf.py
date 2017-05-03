# import matplotlib.pyplot as plt
# import numpy as np

#filePath = "../outagesxy/"+tunnel+way+".txt"
# filePath = "../outagesxy/DMATEntranceExit.txt"
# listError = np.loadtxt(filePath, delimiter='\t', unpack = True, usecols=(3,), skiprows=1)

# mu = 6.19143868296
# sigma = 2.88459214044
# count, bins, ignored = plt.hist(listError, 60, normed=True)

# plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) *
#                np.exp( - (bins - mu)**2 / (2 * sigma**2) ),
#          linewidth=2, color='r')
# plt.show()


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
		#f = open(tunnel+way+".txt","w")
		#f.write( str(np.mean(listError))+'\t'+str(np.std(listError))+'\n' )
		mu = np.mean(listError)
		sigma = np.std(listError)

		if way == "EntranceExit":
			line = '-'
		else:
			line = '-.'

		count, bins, ignored = plt.hist(listError, 60, normed=True, edgecolor= colors[count2], facecolor="None")

		

		plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) *
               np.exp( - (bins - mu)**2 / (2 * sigma**2) ),
         linewidth=2, color=colors[count2],label=tunnel+way, ls=line)

	plt.xlabel("Error (m)")
	#plt.xticks(np.arange(-1,10))
	#plt.xscale('log')
	#plt.yscale('log')
	plt.ylabel("PDF")
	plt.legend(loc='upper left', bbox_to_anchor=(0.82, 1.016), fancybox=True, shadow=True, fontsize='small')

	fig = plt.gcf()
	#fig.suptitle('Error Per Cycle',y=0.98)
	plt.show()
	plt.draw()
	plt.grid(True)
	fig.savefig('pdf'+tunnel+'.png',dpi=200)

	plt.clf()
	#plt.errorbar(count2, np.mean(listError), np.std(listError), label=tunnel+way, color = colors[count], fmt='--o')#, linewidth=5)
	count2+=1

