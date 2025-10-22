import csv
import numpy as np

x = [0, 1, 2, 4]
y = [1, 1, 1, 1]

OAdata = [[x],[y]]
uh = np.transpose(OAdata).tolist()
print(np.transpose(OAdata))
print("Start")
with open('fuck.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y'])   # Header
    for xi, yi in zip(x, y):
        writer.writerow([xi, yi])

print("complete")