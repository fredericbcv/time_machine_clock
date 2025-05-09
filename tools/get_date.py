#!/usr/bin/python3

import os, requests, subprocess, time
from subprocess import Popen, PIPE, STDOUT

# curl http://heurelegalefrancaise.fr/gettime.php?timer=0
r = requests.get("http://heurelegalefrancaise.fr/gettime.php?timer=0")
if r.status_code != 200: raise SystemExit("HTTP status_code not OK")
date1 = (r.text.split()[1])
date1 = int(date1[:-5])
print(date1)

cmd = "date +%s".split()
proc = Popen(cmd,stdout=PIPE)
outs, errs = proc.communicate()
date2 = int(outs.strip())
print(date2)

for date_idx in [date1,date2]:
	# print(time.localtime(date_idx))
	print(time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.localtime(date_idx)))

# https://www.epochconverter.com/programming/c
