import sys
import csv

def caua(listRow):
    res=[]
    for key in listRow[0].keys():
        count = 0
        for row in listRow:
            if row[key] == '':
                res.append((key,count))
                count += 1
    res=dict(res)
    return res
if __name__ == '__main__':
    input_file_name = str(sys.argv[1])
    listRow=[]
    with open(input_file_name) as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            listRow.append(row)
    caua(listRow)