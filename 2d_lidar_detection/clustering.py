import numpy as np
from sklearn.cluster import DBSCAN

def perform_clustering(data, epsilon, min_samples):

    data = np.array(data)

    dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
    labels = dbscan.fit_predict(data)

    return labels

lidar_data = [
    [1.0, 2.0],
    [1.5, 2.5],
    [10.0, 12.0],
    [10.5, 12.5],
    [20.0, 22.0],
    [20.5, 22.5]
]

epsilon = 1.0
min_samples = 2

cluster_labels = perform_clustering(lidar_data, epsilon, min_samples)
print(cluster_labels)