import numpy as np

tunnels = ['RIO450', 'DPT', 'DMAT', 'RCLT' , 'YBT']
ways = ['EntranceExit', 'ExitEntrance']

for tunnel in tunnels:

	for way in ways:
		if(way == "ExitEntrance" and tunnel == "RIO450"):
			continue
		path = tunnel+way+'.txt'
		
		listLon, listLat = np.loadtxt(path, delimiter='\t', unpack = True, usecols=(1,2), skiprows=1)
		file = open('bare_outages/'+tunnel+way+'.txt','w')
		file.write('if(outagesFile=="'+tunnel+way+'"){\n'+ 'double listLonLat['+str(len(listLon))+']['+str(len(listLon))+'] = {\n')
		for i in range(0, len(listLon)):
			if(i == (len(listLon)-1)):
				file.write('\t{'+str(listLon[i])+', '+str(listLat[i])+'}\n')
			else:
				file.write('\t{'+str(listLon[i])+', '+str(listLat[i])+'},\n')
		file.write('};\n}')