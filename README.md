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


# Example of use 


![RAW_VOXEL_PASS (1)](https://user-images.githubusercontent.com/73825898/228518018-eeb5d857-109a-45ba-b24a-515d3a1bda69.png)

1 : The first point cloud is the raw point cloud coming from the ZED camera. As you can see, the
overall result is correct, but with residual light coming from a window, and with some bumps in the
ground, the point cloud is biased, and some surfaces are not scanned as they should be. Finally the
objective is to extract this ground which is not correctly represented, to extract from it a surface on
which we could work correctly and to make it as uniform as possible.
2 : On the second point cloud, we will applied the voxel filter. The voxel grid filter is a key tool in the
filtering of raw point clouds because it allows for the reduction of the number of points in the cloud
while still preserving the overall shape of the objects in the scene. This is achieved by dividing the
point cloud into small cubic cells, or voxels, and then representing each cell by a single point that is
the mean of the points within that cell. By reducing the size of the point cloud, the voxel grid filter
allows for faster and more efficient processing, making it a valuable tool for robotic navigation and
mapping.
3 : Finally with the shaping of the raw point cloud, we’ll use pass through filter. The filter works by
allowing points in a point cloud to "pass through" to the output cloud if they meet certain criteria,
and discarding points that do not meet these criteria. This can be useful for removing points that are
outside of a certain range or for selecting points along a specific axis. For our case, We juste need
the plan with the maximum of ground, so we’ll just put a range to get it.
After a good shaping, we can add some processing filters to optimize, normalize and get the good
shape that we want : **the ground**.


![Process_filt (2)](https://user-images.githubusercontent.com/73825898/228518049-0c88b516-2f0d-4ade-af2f-5bf1796d5001.png)

1 : We’ll use the last point cloud generated to compute it and add some processing filters to get all of
that we need. For that, we’ll take as an input, the Pass-through point cloud to put on it the Statistical
Outlier Removal filter. Statistical Outlier Removal works by identifying and removing points that
are significantly different from the surrounding points in the cloud. These points are referred to
as outliers and can result from measurement errors or other sources of noise. The threshold can be
adjusted to provide a desired level of filtering. This filter is important for the following steps because
by adjusting the quality of the point cloud, the computing will be more accurate with the next filters.
2 : The next one who will interested us is the Project inliner. The Project Inlier filter is a tool used to
identify and extract points that belong to a specific plane in the scene. The filter works by projecting
the points onto a specified plane and then selecting only the points that are within a certain threshold
distance from the plane. The selected points are considered inliers and are retained in the output
point cloud. By using this filter, and in our case, we can extract the ground without any walls or
objects on it. This is very important for the final step of the processing on the point cloud, because
we have a plane surface, and so we have the possibility to get the edges and improve the general
quality.
3 : Finally, to improve quality of the last point cloud, and so get a complete surface an normalize it,
We’ll use the Convex Hull filter. The Convex Hull filter is a useful tool for extracting information
about the shape and boundaries of objects in a point cloud. By identifying the convex hull of an
object, the filter can provide a simplified representation of the object that is easier to work with,
while still retaining the important information about the object’s shape and size. By playing with
the Voxel grid to get a simpler point cloud, and have a good representation of the edges by using
convex hull filter, we can extract from the last point cloud, a ground surface from the starting point
cloud.
In conclusion, PCL (Point Cloud Library) is a powerful tool for obtaining a plane surface to work
on in computer vision and robotics applications. PCL offers various algorithms for processing 3D
point clouds and detecting plane surfaces. The resulting plane surface can be used for a variety of
purposes like navigation and localization inside an environment.
