# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter
vox = cloud.make_voxel_grid_filter()
LEAF_SIZE=.01

vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

#call the filter
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)


# PassThrough filter
passthrough = cloud_filtered.make_passthrough_filter()

filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = .6
axis_max = 1.1
passthrough.set_filter_limits(axis_min,axis_max)

cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)


# RANSAC plane segmentation
seg = cloud_filtered.make_segmenter()

seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

max_distance= .01
seg.set_distance_threshold(max_distance)

inliers,coefficients = seg.segment()


# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=True)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)

# Save pcd for table
table = cloud_filtered.extract(inliers, negative=False)
filename = 'table.pcd'
pcl.save(table, filename)
# pcl.save(cloud, filename)


# Extract outliers
outlier_filter = cloud_filtered.make_statistical_outlier_filter()

outlier_filter.set_mean_k(50)
x=1.0

# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
outlier_filter.set_std_dev_mul_thresh(x)

cloud_filtered = outlier_filter.filter()


# Save pcd for tabletop objects


