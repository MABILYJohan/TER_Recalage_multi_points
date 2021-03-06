/************
 * TEST ICPnl *
 ************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/io/pcd_io.h>

#include "utils_pcl.h"


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


int main (int argc, char *argv[])
{
	
	if (argv[1] != NULL && strcmp(argv[1], "-h")==0) {
		std::cout << " \nUsage  " << std::endl;
		printf ("%s file1.stl/pcd file2.stl/pcd [maxCorDist] [iterations] [epsilon] [difDistEpsilon]\n\n", argv[0]);
		exit (0);
	}
	
	pcl::console::print_highlight ("Loading point clouds...\n");
	// Checking program arguments
	if (argc-1 < 2)
	{
		printf ("Usage :\n");
		printf ("\t\t%s file1.stl/pcd file2.stl/pcd [maxCorDist] [iterations] [epsilon] [difDistEpsilon]\n", argv[0]);
		PCL_ERROR ("Provide two stl or pcd files.\n");
		return (-1);
	}
	std::cout << " loading cloud src " << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr BIGcloud_source = loadCloud(argv[1]);
	std::cout << " loading cloud target " << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr BIGcloud_target = loadCloud(argv[2]); 
	
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	pcl::console::print_highlight ("DownSampling...\n");
	pcl::PointCloud<pcl::PointXYZ> cloud_source = downSample_cloud (BIGcloud_source);
	pcl::PointCloud<pcl::PointXYZ> cloud_target = downSample_cloud (BIGcloud_target);
	
	pcl::console::TicToc time;
	time.tic ();
	
	pcl::console::print_highlight ("iterative closest point...\n");
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icpnl;
	
	PointCloud<PointXYZ>::Ptr src (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_source, *src);
	PointCloud<PointXYZ>::Ptr tgt (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_target, *tgt);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered (new PointCloud<PointXYZ>);;
	copyPointCloud (*src, *cloud_source_registered);
	
	// Set the transformation epsilon (criterion 2)
	double epsilon = 1e-8;
	if (argv[5]!=NULL && atof(argv[5])>0)	epsilon = atof(argv[5]);
	icpnl.setTransformationEpsilon (epsilon);

	// Set the euclidean distance difference epsilon (criterion 3)
	double difDistEpsilon = 1e-8;
	if (argv[6]!=NULL && atoi(argv[6])>0)	difDistEpsilon = atoi(argv[6]);
	icpnl.setEuclideanFitnessEpsilon (difDistEpsilon);
	
	icpnl.setInputSource (src);
	icpnl.setInputTarget (tgt);
	
	// Perform the alignment
	icpnl.align (*cloud_source_registered);
	if (!icpnl.hasConverged ()) {
		PCL_ERROR ("\nICP has not converged.\n");
		return (-1);
	}
	
	//~ std::cout << " \nICP has converged in " << time.toc () << "ms, score is  " << icpnl.getFitnessScore () << std::endl;
	print_info ("ICP has converged with score of "); print_value ("%f", icpnl.getFitnessScore ()); print_info (" in "); print_value ("%g", time.toc ()); print_info (" ms : \n");
	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4f transformation_matrix = icpnl.getFinalTransformation();
	
	std::cout << " Matrix " << std::endl;
	print4x4Matrix (transformation_matrix);
	
	//~ pcl::transformPointCloud(cloud_source, *cloud_source_registered, transformation_matrix);
	
	// Compute the Hausdorff distance
	pcl::console::print_highlight ("Hausdorff\n");
	compute (cloud_target, *cloud_source_registered);
	
	int iterations = 1;
	pcl::console::print_highlight ("Visualisation \n");
	vizu (cloud_source, cloud_target, *cloud_source_registered, iterations);
	
	return (0);
}








