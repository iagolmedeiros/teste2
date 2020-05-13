import argparse
import numpy as np
import sklearn.cluster as cluster
import time
import hdbscan

def create_clusters(data, algoritm, args, kwds):
    start_time = time.time()
    fitted_algoritm = algoritm(*args, **kwds).fit(data)
    end_time = time.time()
    print('Clustering took {:.2f} s'.format(end_time - start_time))
    return fitted_algoritm

def get_clusters(data, fitted_algoritm):
    n_clusters_ = len(set(fitted_algoritm.labels_)) - (1 if -1 in fitted_algoritm.labels_ else 0)
    print(fitted_algoritm.labels_)
    data = np.array(data)
    clusters = [data[fitted_algoritm.labels_ == i] for i in range(n_clusters_)]
    return clusters

def calculate_centroid(cluster):
    n_positions = int(cluster.size/3)
    sums = np.sum(cluster,axis=0)
    centroid = np.divide(sums, n_positions)
    return centroid

parser = argparse.ArgumentParser(description='Cluster position data with different algoritms.')
parser.add_argument('algo', metavar='algoritm',
                    help='Clustering algoritm to use',
                    choices=['kmeans', 'meanshift', 'dbscan', 'hdbscan'])
args = parser.parse_args()

with open("positions.txt") as f:
    lines = [i.split() for i in f.readlines()]
    for i, v in  enumerate(lines):
        lines[i] = [float(k) for k in v]

if args.algo == 'kmeans' or args.algo == 'meanshift':
    if args.algo == 'kmeans':
        fitted_algoritm = create_clusters(lines, cluster.KMeans, (), {'n_clusters':3})
    else:
        fitted_algoritm = create_clusters(lines, cluster.MeanShift, (), {})
    cluster_centers = np.array(fitted_algoritm.cluster_centers_)

else:
    if args.algo == 'dbscan':
        fitted_algoritm = create_clusters(lines, cluster.DBSCAN, (), {'eps':100})
    else:
        fitted_algoritm = create_clusters(lines, hdbscan.HDBSCAN, (), {'min_cluster_size': 15})
    clusters = get_clusters(lines,fitted_algoritm)
    cluster_centers = np.array([calculate_centroid(cluster) for cluster in
clusters])

with open("centers.txt", "w") as c:
    for center in cluster_centers:
        coords = f"{center[0]} {center[1]} 30\n"
        c.write(coords)

f = open("centroids.txt", "w")
f.write(str(int(cluster_centers.size/3)))
f.close()
