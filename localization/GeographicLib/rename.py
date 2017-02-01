import os

path = '/local1/liborio/VehicularNetworking/workspace/LocVanet/localization/GeographicLib/tools/'

listFiles = os.listdir(path)

for file in listFiles:
	fileName = file.split('.')[0]
	ext = file.split('.')[1]

	if ext == 'cpp':
		os.rename(path+file, path+fileName+'.cc')	
