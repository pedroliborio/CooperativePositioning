#!/usr/bin/python
from xml.dom import minidom
import sys
import argparse
import matplotlib
from matplotlib import pyplot as plt
import numpy as np

################Begin of Functions to parse XML

#Extract data from the xml format to  alist with ID
#and necessary attributes only
# def ExtractDataXML(nodeListXML, edgeListXML):
	
# 	nodeList = list()
# 	edgeList = list()
# 	edgesShape = dict()
	
# 	for edge in edgeListXML:
# 		if IsAValidEdge(edge):
# 			#Pushing edge Attributes : from, to, weight
# 			#TODO replace this line by the error of GPS
# 			edgeList.append([edge.attributes['from'].value, edge.attributes['to'].value, np.random.random_sample()])
# 			#Building the dict
# 			#edgesShape['']
# 			#Pushing node attributes: id, lat , lon
# 			node = FindNodeXML(edge.attributes['from'].value,  nodeListXML)
# 			nodeList.append([node.attributes['id'].value, float(node.attributes['lat'].value), float(node.attributes['lon'].value)])
# 			node = FindNodeXML(edge.attributes['to'].value,  nodeListXML)
# 			nodeList.append([node.attributes['id'].value, float(node.attributes['lat'].value), float(node.attributes['lon'].value)])

# 	return nodeList, edgeList


# #Search for a node with an ID
# def FindNodeXML(identifier, nodeList):
# 	for node in nodeList:
# 		if node.attributes['id'].value == identifier:
# 			return node
# 	print('Referencia de node nao encontrada!') 

# #Valid edges in .net file has a 'from' -> 'to' attribute
# def IsAValidEdge(edge):
	
# 	for attr in edge.attributes.items():
# 		if attr[0] == 'from':
# 			return True
# 	return False

# #################End of Functions to parse XML	


# def FindNode(nodeId,nodeList):
# 	for node in nodeList:
# 		if(node[0] == nodeId):
# 			return node
	
# 	print("Node not found")

def main(argv):
	tunnels = ['RIO450', 'DPT', 'DMAT', 'RCLT' , 'YBT']
	ways = ['EntranceExit','ExitEntrance']

	listEdgesRoute = list()
	
	for tunnel in tunnels:
		netname = tunnel.lower()
		if(tunnel == "RIO450"):
			netname = netname[0:3]
		
		
		#.net file with network information
		netFile = minidom.parse('../../sumoscenarios/'+ netname +'.net.xml')
		#lists with xml attributes
		edgeListNetFile = netFile.getElementsByTagName('edge')
		junctionListNetFile = netFile.getElementsByTagName('junction')
		lanesListNetFile = netFile.getElementsByTagName('lane')
		

		for way in ways:

			if(way == "ExitEntrance" and tunnel == "RIO450"):
				continue

			print(tunnel+way+'.txt')
			with open(tunnel+way+'.txt') as edgesRouteFile:
				listEdgesRoute =  edgesRouteFile.readlines()
			edgesRouteFile.close()
			
			listEdgesRoute = [x.strip('\n') for x in listEdgesRoute]


			with open("out_without_id/"+tunnel+way+".txt",'w') as outputFile:
				
				for edgeID in listEdgesRoute:
					
					for edge in edgeListNetFile:

						if(edge.attributes['id'].value == edgeID):
							#outputFile.write(edgeID+'\t')
							# fromID = edge.attributes['from'].value

							# for junction in junctionListNetFile:
								
							# 	if junction.attributes['id'].value == fromID:
							# 		x = junction.attributes['x'].value
							# 		y = junction.attributes['y'].value
							# 		outputFile.write(x+'\t'+y+'\n')

									# if junction.hasAttribute('shape'):
									# 	strShape = junction.attributes['shape'].value
									# 	listCoord = strShape.split(' ')

									# 	for coord in listCoord:
									# 		xy = coord.split(',')
									# 		x = xy[0]
									# 		y = xy[1]
									# 		outputFile.write(x+'\t'+y+'\n')		


									#break

							# if edge.hasAttribute('shape'):
							# 	strShape = junction.attributes['shape'].value
							# 	listCoord = strShape.split(' ')

							# 	for coord in listCoord:
							# 		xy = coord.split(',')
							# 		x = xy[0]
							# 		y = xy[1]
							# 		outputFile.write(x+'\t'+y+'\n')

							toID = edge.attributes['to'].value

							for junction in junctionListNetFile:

								if junction.attributes['id'].value == toID:

									lanesID = junction.attributes['incLanes'].value
									listLanes = lanesID.split(' ')

									cplistLanes = list(listLanes)

									#filter lanes
									#only lanes with the same edge as prefix
									for lane in cplistLanes:
										edgevalue = lane.split('_')[0]
										if edgevalue != edgeID :
											listLanes.remove(lane)
											
									centralLane = int(len(listLanes)/2)

									for laneXML in lanesListNetFile:
										if laneXML.attributes['id'].value == listLanes[centralLane]:

											strShape = laneXML.attributes['shape'].value
											listCoord = strShape.split(' ')
											
											#outputFile.write(str(len(listCoord))+'\n')
											for coord in listCoord:
												xy = coord.split(',')
												x = xy[0]
												y = xy[1]
												outputFile.write(x+'\t'+y+'\n')
											break



									#x = junction.attributes['x'].value
									#y = junction.attributes['y'].value
									#outputFile.write(x+'\t'+y+'\n')

									# if junction.hasAttribute('shape'):
									# 	strShape = junction.attributes['shape'].value
									# 	listCoord = strShape.split(' ')

									# 	for coord in listCoord:
									# 		xy = coord.split(',')
									# 		x = xy[0]
									# 		y = xy[1]
									# 		outputFile.write(x+'\t'+y+'\n')

									break
							
							break

			outputFile.close()

 			#print(nodeList, "\n\n")
#print(edgeList, "\n\n")

#Getting the weights for colormap normalization
# weightList = list()
# for edge in edgeList:
# 	weightList.append(edge[2])


# # norm is a class which, when called, can normalize data into the
# # [0.0, 1.0] interval.
# norm = matplotlib.colors.Normalize(
# vmin=np.min(weightList),
# vmax=np.max(weightList))

# # choose a colormap
# #c_m = matplotlib.cm.jet
# c_m = matplotlib.cm.RdYlGn_r

# # create a ScalarMappable and initialize a data structure
# s_m = matplotlib.cm.ScalarMappable(cmap=c_m, norm=norm)
# s_m.set_array([])

# for edge in edgeList:
# 	nodeFrom = FindNode(edge[0],nodeList)
# 	print("FROM",nodeFrom)
# 	type(nodeFrom)
# 	nodeTo = FindNode(edge[1],nodeList)
# 	print("TO",nodeTo)
# 	print("WEIGHT",edge[2])
# 	plt.plot([nodeFrom[2], nodeTo[2]], [nodeFrom[1],nodeTo[1]], color=s_m.to_rgba(edge[2]), linewidth=2)

# # having plotted the 11 curves we plot the colorbar, using again our
# # ScalarMappable
# plt.colorbar(s_m)

# plt.show()
if __name__ == "__main__":
    main(sys.argv)





