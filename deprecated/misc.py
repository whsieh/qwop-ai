import json as json

def writeAsGPlot(fname):

	with open('datasets/' + fname + '.json') as rawdata:
		dataset = json.load(rawdata)
	rawdata.close()

	xvals = []
	yvals = []

	for point in dataset:
		xvals.append(0.1*point['t'])
		yvals.append(point['dist'])

	with open('datasets/' + fname + '.gplot','w') as fout:
		for i in range(len(xvals)):
			fout.write(str(xvals[i]) + ' ' + str(yvals[i]) + '\n')
	fout.close()

def loadJSONDataset(fLabel='ds00'):
	with open('datasets/' + fLabel + '.json') as rawdata:
		dataset = json.load(rawdata)
	rawdata.close()
	return dataset

def removeZerosFromDataset(fLabel='ds00'):
	ds,_Q_ = loadJSONDataset(fLabel),{}
	for s in ds['Q']:
		for a in ds['Q'][s]:
			if ds['Q'][s][a] != 0:
				if s not in _Q_:
					_Q_[s] = {}
				_Q_[s][a] = ds['Q'][s][a]			
	_ds_ = {'N':ds['N'],'Q':_Q_}
	return "".join(json.JSONEncoder().encode(_ds_).split())

if __name__ == '__main__':
	# writeAsGPlot('dd00')
	label = 'qn00'
	fout = open('datasets/' + label + '_.json','w')
	fout.write(removeZerosFromDataset(label))
	fout.close()
	pass