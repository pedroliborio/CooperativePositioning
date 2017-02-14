import numpy as np

tunnels = ['RIO450', 'DPT', 'DMAT', 'RCLT' , 'YBT']
ways = ['EntranceExit', 'ExitEntrance']

for tunnel in tunnels:

	for way in ways:
		if(way == "ExitEntrance" and tunnel == "RIO450"):
			continue
		path = tunnel+way+'.txt'
		listError = np.loadtxt(path, delimiter='\t', unpack = True, usecols=(3,), skiprows=1)
		file = open(tunnel+way+'Count.txt','w')
		file.write('1'+'\t'+str(int((len(listError)/2)))+'\n');