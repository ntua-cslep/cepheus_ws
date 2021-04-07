import os
import csv


def get_latest(filenames):
	if filenames:
		filenames.sort(reverse=True)
		return filenames[0]
	else:
		raise KeyError


def get_list_from_file(file, topic):
	f_contents = []
	f_contents.append(topic)
	with open(file) as f:
		content = f.readlines()
		for line in content:
			l = line.split(',', 1)[1]
			if l == 'field.data\n':
				continue
			# print(line.strip('\n'))
			f_contents.append(l.strip('\n'))
	return f_contents



def main():
	mypath = './src/cepheus_robot/bags'
	bag_files = []
	for (dirpath, dirnames, filenames) in os.walk(mypath):
		for filename in filenames:
			if filename.endswith('.bag'):
				bag_files.append(filename)
		break
	try:
		filename = get_latest(bag_files)
	except KeyError:
		return 1
	f_prefix = filename.split('.', 1)[0]
	txt_files = []
	for (dirpath, dirnames, filenames) in os.walk(mypath):
		for filename in filenames:
			if filename.startswith(f_prefix) and filename.endswith('.txt'):
				txt_files.append(filename)
		break

	filepaths = []
	for txt_file in txt_files:
		filepath = mypath + '/' + txt_file
		# print(filepath)
		filepaths.append(filepath)

	# get file contents
	print('Getting file contents')
	f_contents = []
	for file in filepaths:
		topic = file.split('bag.', 1)[1].split('.txt', 1)[0]
		# print(topic)
		# if topic not in ['set_ls_qd', 'set_le_qd', 'set_re_qd']:
		l = get_list_from_file(file, topic)
		print(len(l))
		f_contents.append(l)
	print('Done')
	# print (len(f_contents))
	# write to csv
	csv_filename = f_prefix + '.csv'
	with open(csv_filename, "w") as f:
		writer = csv.writer(f)
		# print(list(zip(*f_contents)))
		writer.writerows(list(zip(*f_contents)))

	# delete all txt
	# for (dirpath, dirnames, filenames) in os.walk(mypath):
	# 	for filename in filenames:
	# 		if filename.endswith('.txt'):
	# 			os.remove(mypath + '/' + filename)
	# 	break

if __name__ == "__main__":
	main()
