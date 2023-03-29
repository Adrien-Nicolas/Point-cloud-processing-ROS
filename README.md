# Point-cloud-processing-ROS
A point cloud processing packages in C++ for ROS environment


### Parameters

You can change all of the parameters related to rqt_reconfigure for the PCL filters inside the [config folder](./pcl_helpers/cfg/)

### Voxel fitler part :

- [Voxel fitler source file](./pcl_helpers/src/voxel_filter_node.cpp)

### PassThrough filter part :

- [PassThrough filter source file](./pcl_helpers/src/passthrough_filter_node.cpp)

### StatisticalOutlierRemoval filter part :

- [StatisticalOutlierRemoval filter source file](./pcl_helpers/src/statistical_removal.cpp)

### Segmentation filter part :

- [Segmentation filter source file](./src/plane_model_segmentation.cpp)

### ProjectInliers filter part :

- [ProjectInliers filter source file](./pcl_helpers/src/project_inliner.cpp)

#### Running step :

- 1 : First filter : Voxel filter, will be take the point cloud from the camera and will be downsampled it. He will generated a new point cloud on his own topic (**/VOXEL_OUT**)

- 2 : Second filter : PassThrough filter, will be take the point cloud from the Voxel filter and will be filtered it (to get only the height range of the point cloud that we want) He will generated a new point cloud on his own topic (**/PASS_OUT**)

- 3 : Third filter : StatisticalOutlierRemoval filter, will be take the point cloud from the PassThrough filter and will be filtered it (to improve the quality of the point cloud and remove outliers) He will generated a new point cloud on his own topic (**/STAT_OUT**)

- 4 : Fourth filter : Segmentation filter, will be take the point cloud from the StatisticalOutlierRemoval filter and will be filtered it (to get only the plane of the point cloud that we want) He will generated a new point cloud on his own topic (**/SEGM_OUT**)

- 5 : Fifth filter : ProjectInliers filter, will be take the point cloud from the Segmentation filter and will be filtered it (to get only the plane of the point cloud in 2D) He will generated a new point cloud on his own topic (**/INLINE_OUT**)
