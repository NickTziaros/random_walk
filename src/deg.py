#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
 
 
# Creating dataset
np.random.seed(10)
 
data_25B = [1.08,1.30,0.49,1.10,1.16,1.31,0.56,0.48,0.52,1.10]

data25B = np.array(data_25B)*60

data_50B = [2.47,3.17,2.48,2.04,2.37,2.39,2.40,1.42,2.51,3.9]

data50B = np.array(data_50B)*60

data_90B = [13.01,17.46,14.09,16.56,15.33,18.53,10.30,13.17,9.58,10.49]

data90B = np.array(data_90B)*60

data_25L = [0.27,0.37,0.57,0.40,0.45,0.40,0.40,0.58,0.50,0.53]

data25L = np.array(data_25L)*60

data_50L = [2.03,1.28,2.17,2.04,1.40,1.53,1.48,1.43,2.08,2.42]

data50L = np.array(data_50L)*60

data_90L = [5.45,8.33,10.16,8.39,8.47,6.32,7.03,3.33,6.48,10.22]

data90L = np.array(data_90L)*60

data25 = [data25B, data25L]
data50 = [data50B, data50L]
data90 = [data90B, data90L]
# fig = plt.figure(figsize =(10, 7))





 
# # Creating axes instance
# ax = fig.add_axes([0, 0, 1, 1])
fig, ax = plt.subplots() 
# Creating plot
bp = ax.boxplot(data25)
ax.set_title('Time to reach 25% coverage') 
plt.xticks([1, 2], ['Fixed Step', 'Levy'])
ax.set_ylabel('Time in s')
# show plot
plt.show()
