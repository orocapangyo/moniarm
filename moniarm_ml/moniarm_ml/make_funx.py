import numpy as np
from scipy.optimize import curve_fit
import csv
import os

def calcPwm(x, a, b, ):
    return a * x + b

input = []
output = []
csvfile = os.path.expanduser('~/ros2_ws/src/moniarm/moniarm_ml/moniarm_ml/dataset/train/kinematics_pose.csv')

with open(csvfile, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        input.append(row.get('X+1'))
        output.append(row.get('angle0'))
        
popt, pcov = curve_fit(calcPwm, input, output)
print(popt)
