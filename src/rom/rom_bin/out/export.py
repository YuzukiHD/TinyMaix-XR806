#! python3

import sys, os, csv

def replase_str(file_ld, s_src, s_dst):
	#print("replase ld: ", file_ld)
	f = open(file_ld,'r')
	rc = f.read().replace(s_src, s_dst)
	f.close()
	t = open(file_ld,'w')
	t.write(rc)
	t.close()

in_file = open('func.csv', 'r')
csvreader = csv.reader(in_file)
func_list = list(csvreader)
in_file.close()
#print(func_list)
for element in func_list:
	replase_str('rom_symbol.ld', element[0], element[1])
