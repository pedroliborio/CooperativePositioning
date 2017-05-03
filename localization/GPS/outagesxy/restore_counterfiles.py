import numpy as np

tunnels = ['RIO450', 'DPT', 'DMAT', 'RCLT' , 'YBT']
ways = ['EntranceExitCount', 'ExitEntranceCount']

for tunnel in tunnels:

	for way in ways:
		if(way == "ExitEntranceCount" and tunnel == "RIO450"):
			continue
		path = tunnel+way+'.txt'
		counter, outages  = np.loadtxt(path, delimiter=' ', unpack = True, usecols=(0,1))
		file = open(path,'w')
		file.write('1'+' '+str(int(outages))+'\n');