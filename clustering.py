import numpy as np
from sklearn.cluster import KMeans
from sklearn.cluster import MeanShift

with open("positions.txt") as f:
    lines = [i.split() for i in f.readlines()]
    for i, v in  enumerate(lines):
        lines[i] = [float(k) for k in v]

kmeans = KMeans(n_clusters=3).fit(lines)
print(kmeans.labels_)
cluster_centers = np.array(kmeans.cluster_centers_)
with open("centers.txt", "w") as c:
    for center in kmeans.cluster_centers_:
        coords = f"{center[0]} {center[1]} {center[2]}\n"
        c.write(coords)

ms = MeanShift().fit(lines)
cluster_centers = np.array(ms.cluster_centers_)
with open("centers.txt", "w") as c:
    for center in ms.cluster_centers_:
        coords = f"{center[0]} {center[1]} {center[2]}\n"
        c.write(coords)

f = open("centroids.txt", "w")
f.write(str(cluster_centers.size))
f.close()
