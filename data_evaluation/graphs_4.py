import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import os
from roll_functions import SelectFile
from roll_functions import ChangeDir

os.chdir("/home/pablo/Desktop/MY_THESIS/ICOlearning_AR/data_evaluation/Experiments")

data = pd.read_csv(SelectFile(), delimiter=",")

print(data.head())

n_values = np.linspace(start=1, stop=data.shape[0], num=data.shape[0])
data = data.shift(periods=1, axis=1)
data.loc[:,"Weight_roll_0"] = data.index
data.index = n_values

print(data.head())


#############################################################################################################################################################################
# Generate the graph variable

graph, ax_1= plt.subplots()
ax_1.grid(True)

ax_1.plot(data.index, data.loc[:,"Weight_pitch_0"])
ax_1.plot(data.index, data.loc[:,"Weight_pitch_1"])
ax_1.set_title("Pitch evolution")
ax_1.set_ylabel("Weight value")
ax_1.set_xlabel("Iteration")

plt.savefig("pitch_weights.png", format="png")
ax_1.clear()
ax_1.grid(True)

ax_1.plot(data.index, data.loc[:,"Weight_roll_0"])
ax_1.plot(data.index, data.loc[:,"Weight_roll_1"])
ax_1.set_title("Roll evolution")
ax_1.set_ylabel("Weight value")
ax_1.set_xlabel("Iteration")

plt.savefig("roll_weights.png", format="png")
ax_1.clear()