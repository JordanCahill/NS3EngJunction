#!/usr/bin/python

import sys
import os
from multiprocessing import Pool
	
def start_simulation(data):
	run = data[0]
	sub_run = data[1]
	current_data = data[2]
        os.system('./waf --command-template="%%s --totalData=%d --RngRun=%d" --run scratch/v2x-analysis' % (current_data, sub_run))

if len(sys.argv) != 5:
	print "usage: ./parallel [max_data] [step] [sub_runs] [processes]"
	sys.exit(0)

max_data = int(sys.argv[1])
step = int(sys.argv[2])
sub_runs = int(sys.argv[3])
processes = int(sys.argv[4])

# create params
params = []
run = 1
for i in range(0, max_data+step, step):
	current_data = i
	for j in range(sub_runs):
		params.append([run, j+1, current_data])
	run += 1

# run
pool = Pool(processes=processes)
pool.map(start_simulation, params)
