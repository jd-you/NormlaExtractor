# Normal Extractor
## Estimator
The basic idea of normal estimator is PCA(principal component analysis). It estiamte the surface normal in point cloud directly.

Paper:<http://mediatum.ub.tum.de/doc/800632/941254.pdf>

the node has two modes:
1. pcl_estimator: use the method integrated in PCL to estimate normal
2. tree_estimator: use octree to estimate normal (faster than pcl)
