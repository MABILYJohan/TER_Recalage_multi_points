
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/io/vtk_lib_io.h>

int main (int argc, char* argv[])
{
	
	//~ pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//~ pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	char file1[20] = "1.1 clean.pcd";
	char file2[20] = "1.2.1 clean.pcd";
	
	//~ // vtk reader
	//~ vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	//~ vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	//~ reader->SetFileName(file1);
	//~ reader->Update();
	//~ std::cout << "Loaded " << target_cloud->size () << " data points from " << file1 << std::endl;
	
	//~ vtkSmartPointer<vtkSTLReader> reader2 = vtkSmartPointer<vtkSTLReader>::New();
	//~ vtkSmartPointer<vtkPolyData> polydata2 = reader2->GetOutput();
	//~ reader2->SetFileName(file2);
	//~ reader2->Update();
	//~ std::cout << "Loaded " << input_cloud->size () << " data points from " << file2 << std::endl;
	
	
	//~ char file1[20] = "room_scan1.pcd";
	//~ char file2[20] = "room_scan2.pcd";
	// Loading first scan of room.
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (file1, *target_cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file %s \n", file1);
		return (-1);
	}
	std::cout << "Loaded " << target_cloud->size () << " data points from " << file1 << std::endl;

	// Loading second scan of room from new perspective.
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (file2, *input_cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file %s \n", file2);
		return (-1);
	}
	std::cout << "Loaded " << input_cloud->size () << " data points from " << file2 << std::endl;
	
	
	/**
	 * This section filters the input cloud to improve registration time.
	 * Any filter that downsamples the data uniformly can work for this section.
	 * The target cloud does not need be filtered because 
	 *  voxel grid data structure used by the NDT algorithm 
	 *  does not use individual points, but instead uses 
	 *  the statistical data of the points contained in each 
	 *  of its data structures voxel cells.
	 * */
	// Filtering input scan to roughly 10% of original size to increase speed of registration.
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud (input_cloud);
	approximate_voxel_filter.filter (*filtered_cloud);
	std::cout << "Filtered cloud contains " << filtered_cloud->size ()
		<< " data points from " << file2 << std::endl;
	
	/**
	 * Here we create the NDT algorithm with the default values.
	 * The internal data structures are not initialized until later.
	 **/
	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	
	
	
	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon (0.01);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize (0.1);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution (1.0);


	// Setting max number of registration iterations.
	ndt.setMaximumIterations (35);


	// Setting point cloud to be aligned.
	ndt.setInputSource (filtered_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget (target_cloud);
  
  
	// Set initial alignment estimate found using robot odometry.
	Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
	Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();


	// Calculating required rigid transform to align the input cloud to the target cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align (*output_cloud, init_guess);
	

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
		<< " score: " << ndt.getFitnessScore () << std::endl;
		
		
	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());


	// Saving transformed input cloud.
	pcl::io::savePCDFileASCII ("file2_transformed.pcd", *output_cloud);


	// Initializing point cloud visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
	viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer_final->setBackgroundColor (0, 0, 0);


	// Coloring and visualizing target cloud (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	target_color (target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												  1, "target cloud");

	// Coloring and visualizing transformed input cloud (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	output_color (output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												  1, "output cloud");
		
	// Starting visualizer
	viewer_final->addCoordinateSystem (1.0, "global");
	viewer_final->initCameraParameters ();
	

	// Wait until visualizer window is closed.
	while (!viewer_final->wasStopped ())
	{
		viewer_final->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

  return (0);
}
