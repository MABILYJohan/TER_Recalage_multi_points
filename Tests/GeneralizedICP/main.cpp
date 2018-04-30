/***********************
 * TEST GeneralizedICP *
 ***********************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/time.h>   // TicToc
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp6d.h>


using namespace pcl;
using namespace pcl::io;
using namespace std;


/**
 * This function takes the reference of a 4x4 matrix and prints
 * the rigid transformation in an human readable way.
 **/
void print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


PointCloud<PointXYZ>::Ptr loadCloud (char *fileName)
{
	PointCloud<PointXYZ>::Ptr myCloud (new PointCloud<PointXYZ>);
	
	std::string file = fileName;
	std::size_t pos = file.find_last_of(".");
	std::string ext = file.substr(pos);
	std::cout << ext << std::endl;
	
	if(ext.compare(".stl") == 0) // Si c'est un fichier au format STL
	{
		// vtk reader
		vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
		vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
		reader->SetFileName(fileName);
		reader->Update();

		// convert vtk to pcl object
		io::vtkPolyDataToPointCloud(polydata, *myCloud);
	}
	else if(ext.compare(".pcd")==0) // Si c'est un fichier au format PCD
	{
		if (io::loadPCDFile<PointXYZ> (file, *myCloud) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			exit (1);
		}
	}
	else
	{
		std::cout << "Les format supportés sont STL et PCD" << std::endl;
		exit(1);
	}
	
	return myCloud;
}

PointCloud<PointXYZ> downSample_cloud (PointCloud<PointXYZ>::Ptr originalCloud)
{
	PointCloud<PointXYZ> reductCloud;
	VoxelGrid<PointXYZ> grid;
	grid.setLeafSize (1, 1, 1);
	grid.setInputCloud (originalCloud);
	grid.filter (reductCloud);
	
	return reductCloud;
}


int main (int argc, char *argv[])
{
	if (argv[1] != NULL && strcmp(argv[1], "-h")==0) {
		std::cout << " \nUsage  " << std::endl;
		printf ("%s file1.stl/pcd file2.stl/pcd\n\n", argv[0]);
		exit (0);
	}
	
	console::print_highlight ("Loading point clouds...\n");
	// Checking program arguments
	if (argc-1 < 2)
	{
		printf ("Usage :\n");
		printf ("\t\t%s file1.stl/pcd file2.stl/pcd\n", argv[0]);
		PCL_ERROR ("Provide two stl or pcd files.\n");
		return (-1);
	}
	std::cout << " loading cloud src " << std::endl;
	PointCloud<PointXYZ>::Ptr BIGcloud_source = loadCloud(argv[1]);
	std::cout << " loading cloud target " << std::endl;
	PointCloud<PointXYZ>::Ptr BIGcloud_target = loadCloud(argv[2]);
	
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	console::print_highlight ("DownSampling...\n");
	
	//~ PointCloud<PointXYZ>::Ptr cloud_source = downSample_cloud (BIGcloud_source);
	//~ PointCloud<PointXYZ>::Ptr cloud_target = downSample_cloud (BIGcloud_target);
	
	PointCloud<PointXYZ> cloud_source = downSample_cloud (BIGcloud_source);
	PointCloud<PointXYZ> cloud_target = downSample_cloud (BIGcloud_target);
	PointCloud<PointXYZ> cloud_reg;
	PointCloud<PointXYZRGBA> cloud_with_color;
	
	
	////////////////////////////////////////////////////////////////////
	// GENERALIZED //
	console::print_highlight ("Generalized iterative closest point...\n");
	
	std::cout << " Loading parameters " << std::endl;
	typedef PointXYZ PointT;
	PointCloud<PointT>::Ptr src (new PointCloud<PointT>);
	copyPointCloud (cloud_source, *src);
	PointCloud<PointT>::Ptr tgt (new PointCloud<PointT>);
	copyPointCloud (cloud_target, *tgt);
	PointCloud<PointT> output;
	
	GeneralizedIterativeClosestPoint<PointT, PointT> reg;
	reg.setInputSource (src);
	reg.setInputTarget (tgt);
	
	// Set the maximum number of iterations (criterion 1)
	int iterations = 1;
	if (argv[3]!=NULL && atoi(argv[3])>0)	iterations = atoi(argv[3]);
	reg.setMaximumIterations (iterations);
	
	// Set the transformation epsilon (criterion 2)
	double epsilon = 1e-8;
	if (argv[4]!=NULL && atof(argv[4])>0)	epsilon = atof(argv[4]);
	reg.setTransformationEpsilon (epsilon);
	
	
	// Register
	std::cout << " Align " << std::endl;
	reg.align (output);
	if (!reg.hasConverged ()) {
		PCL_ERROR ("\nreg has not converged.\n");
		return (-1);
	}
	std::cout << " \nreg has converged with score of  " << reg.getFitnessScore () << std::endl;
	
	// Check again, for all possible caching schemes
	for (int iter = 0; iter < 4; iter++)
	{
		std::cout << " \tCheck " << iter+1 << std::endl;
		bool force_cache = (bool) iter/2;
		bool force_cache_reciprocal = (bool) iter%2;
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		// Ensure that, when force_cache is not set, we are robust to the wrong input
		if (force_cache)
			tree->setInputCloud (tgt);
		reg.setSearchMethodTarget (tree, force_cache);
		
		pcl::search::KdTree<PointT>::Ptr tree_recip (new pcl::search::KdTree<PointT>);
		if (force_cache_reciprocal)
			tree_recip->setInputCloud (src);
		reg.setSearchMethodSource (tree_recip, force_cache_reciprocal);
		
		// Register
		reg.align (output);
		if (!reg.hasConverged ()) {
			std::cout << " \treg has not converged " << std::endl;
		}
		std::cout << " \treg has converged with score of  " << reg.getFitnessScore () << std::endl;
	}
	
	// Test guess matrix
	std::cout << " Add test guess matrix " << std::endl;
	Eigen::Isometry3f transform = Eigen::Isometry3f (Eigen::AngleAxisf (0.25 * M_PI, Eigen::Vector3f::UnitX ())
													* Eigen::AngleAxisf (0.50 * M_PI, Eigen::Vector3f::UnitY ())
													* Eigen::AngleAxisf (0.33 * M_PI, Eigen::Vector3f::UnitZ ()));
	transform.translation () = Eigen::Vector3f (0.1, 0.2, 0.3);
	PointCloud<PointT>::Ptr transformed_tgt (new PointCloud<PointT>);
	pcl::transformPointCloud (*tgt, *transformed_tgt, transform.matrix ()); // transformed_tgt is now a copy of tgt with a transformation matrix applied
	
	GeneralizedIterativeClosestPoint<PointT, PointT> reg_guess;
	reg_guess.setInputSource (src);
	reg_guess.setInputTarget (transformed_tgt);
	reg_guess.setMaximumIterations (iterations);
	reg_guess.setTransformationEpsilon (epsilon);
	reg_guess.align (output, transform.matrix ());
	
	if (!reg_guess.hasConverged ()) {
		PCL_ERROR ("\treg_guess has not converged.\n");
		return (-1);
	}
	std::cout << " \treg_guess has converged with score of  " << reg_guess.getFitnessScore () << std::endl;
	std::cout << std::endl;
	
	
	// Visualization
	pcl::visualization::PCLVisualizer viewer ("Generalized ICP");
	
	PointCloud<PointXYZ>::Ptr cloud_source_vizu (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_source, *cloud_source_vizu);
	PointCloud<PointXYZ>::Ptr cloud_target_vizu (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_target, *cloud_target_vizu);
	PointCloud<PointT>::Ptr output_vizu (new PointCloud<PointXYZ>);;
	copyPointCloud (output, *output_vizu);
	
	// Create two vertically separated viewports
	int v1 (0);
	int v2 (1);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	// Original point cloud (source) is black
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_source_color_h 
		(cloud_source_vizu,
		 bckgr_gray_level,
		 bckgr_gray_level,
		 bckgr_gray_level);
	viewer.addPointCloud (cloud_source_vizu, cloud_source_color_h, "cloud_source v1", v1);
	viewer.addPointCloud (cloud_source_vizu, cloud_source_color_h, "cloud_source v2", v2);
	
	// target cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_target_color_h (cloud_target_vizu, 20, 180, 20);
	viewer.addPointCloud (cloud_target_vizu, cloud_target_color_h, "cloud_target_v1", v1);
	
	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h (output_vizu, 180, 20, 20);
	viewer.addPointCloud (output_vizu, cloud_icp_color_h, "cloud_icp_v2", v2);
	
	// Adding text descriptions in each viewport
	viewer.addText ("Black: cloud source\nGreen: cloud target", 10, 15, 16, bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, "icp_info_1", v1);
	viewer.addText ("Black: cloud source\nRed: ICP aligned point cloud", 10, 15, 16, bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, "icp_info_2", v2);	
	
	// Set background color
	viewer.setBackgroundColor ((int)255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, v1);
	viewer.setBackgroundColor ((int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, v2);
	
	// Display the visualiser
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
	
	return 0;
}








