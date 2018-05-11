/************
 * TEST Meta Registration *
 ************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/incremental_registration.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/io/pcd_io.h>

#include "utils_pcl.h"


/**
 * This function takes the reference of a 4x4 matrix and prints
 * the rigid transformation in an human readable way.
 **/
void print4x4Matrix (const Eigen::Matrix4d & matrix)
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
	
	pcl::console::TicToc time;
	time.tic ();
	
	pcl::console::print_highlight ("iterative closest point...\n");
	IterativeClosestPoint<PointXYZ,PointXYZ>::Ptr icp (new IterativeClosestPoint<PointXYZ,PointXYZ>);
	
	// Set the max correspondence distance to 1m (e.g., correspondences with higher distances will be ignored)
	icp->setMaxCorrespondenceDistance (10);
	
	// Set the maximum number of iterations (criterion 1)
	icp->setMaximumIterations (15);
	
	pcl::registration::IncrementalRegistration<PointXYZ> iicp;
	iicp.setRegistration(icp);
	
	int i=1;
	std::stringstream ss;
	// Perform the alignment
	while (i<4)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr BIGcloud = loadCloud(argv[i]);
		pcl::PointCloud<pcl::PointXYZ> DOWNcloud = downSample_cloud (BIGcloud);
		PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
		copyPointCloud (DOWNcloud, *cloud);
		
		
		iicp.registerCloud (cloud);
		PointCloud<PointXYZ>::Ptr tmp (new PointCloud<PointXYZ>);
		transformPointCloud (*cloud, *tmp, iicp.getAbsoluteTransform ());
		
		ss << i << ".pcd";
		pcl::io::savePCDFile (ss.str (), *tmp, true);
		ss.str("");
		
		i++;
	}
	/*
	std::cout << " \nICP has converged, score is  " << icp.getFitnessScore () << std::endl;
	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
	transformation_matrix = icp.getFinalTransformation ().cast<double>();
	
	std::cout << " Matrix " << std::endl;
	print4x4Matrix (transformation_matrix);
	
	// Compute the Hausdorff distance
	pcl::console::print_highlight ("Hausdorff\n");
	compute (*tgt, *cloud_source_registered);
	
	pcl::console::print_highlight ("Visualisation \n");
	vizu (cloud_source, cloud_target, *cloud_source_registered);*/
	
	return (0);
}








