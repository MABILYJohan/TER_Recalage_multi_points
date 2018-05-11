/************
 * TEST FPCS *
 ************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_kfpcs.h>
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
	pcl::registration::KFPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> kfpcs_ia;
	
	PointCloud<PointXYZ>::Ptr src (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_source, *src);
	PointCloud<PointXYZ>::Ptr tgt (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_target, *tgt);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered (new PointCloud<PointXYZ>);;
	copyPointCloud (*src, *cloud_source_registered);
	
	// Set the max correspondence distance to 1m (e.g., correspondences with higher distances will be ignored)
	int nr_threads = 8;
	if (argv[3]!=NULL && atoi(argv[3])>0)	nr_threads = atoi(argv[3]);
	kfpcs_ia.setNumberOfThreads (nr_threads);
	// Set the maximum number of iterations (criterion 1)
	double approx_overlap = 0.9f;
	if (argv[4]!=NULL && atof(argv[4])>0)	approx_overlap = atof(argv[4]);
	kfpcs_ia.setApproxOverlap (approx_overlap);
	// Set the transformation epsilon (criterion 2)
	double delta = 0.1f;
	if (argv[5]!=NULL && atof(argv[5])>0)	delta = atof(argv[5]);
	kfpcs_ia.setDelta (delta, false);
	// Set the euclidean distance difference epsilon (criterion 3)
	double abort_score = 0.0f;
	if (argv[6]!=NULL && atof(argv[6])>0)	abort_score = atof(argv[6]);
	kfpcs_ia.setScoreThreshold (abort_score);
	
	// Set the input source and target
	kfpcs_ia.setInputSource (cloud_source_registered);
	kfpcs_ia.setInputTarget (tgt);
	
	for (int i = 0; i < 1; i++)
	{
		// Perform the alignment
		kfpcs_ia.align (*cloud_source_registered);

	}
	std::cout << " \nICP has converged, score is  " << kfpcs_ia.getFitnessScore () << std::endl;
	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
	transformation_matrix = kfpcs_ia.getFinalTransformation ().cast<double>();
	
	std::cout << " Matrix " << std::endl;
	print4x4Matrix (transformation_matrix);
	
	// Compute the Hausdorff distance
	pcl::console::print_highlight ("Hausdorff\n");
	compute (*tgt, *cloud_source_registered);
	
	pcl::console::print_highlight ("Visualisation \n");
	vizu (cloud_source, cloud_target, *cloud_source_registered);
	
	return (0);
}








