import numpy as np
import pandas as pd
from sklearn.cluster import MeanShift
from sklearn.datasets import make_blobs
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d

#clusters = [[1,1,1],[5,5,5],[3,10,10]]
clusters = [[1,1],[5,5],[3,10]]
X, _ = make_blobs(n_samples = 150, centers = clusters, cluster_std = 0.60)


ms = MeanShift()
ms.fit(X)
cluster_centers = ms.cluster_centers_

#print(cluster_centers)
#print(cluster_centers.size)

f = open("centroids.txt", "w")
f.write(str(cluster_centers.size))
f.close()

for i in range(3):
    for j in range(2):
        print cluster_centers[i,j]

fig = plt.figure()
ax = fig.add_subplot(111)

ax.scatter(X[:,0], X[:,1], marker='o')

ax.scatter(cluster_centers[:,0], cluster_centers[:,1], marker='x', color='red', s=300, linewidth=5, zorder=10)

#plt.show()
