/***********************
 * TEST GeneralizedICP *
 ***********************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/io/pcd_io.h>

#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp6d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include "utils_pcl.h"


using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef PointXYZ PointT;

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
	
	PointCloud<PointXYZ> cloud_source = downSample_cloud (BIGcloud_source);
	PointCloud<PointXYZ> cloud_target = downSample_cloud (BIGcloud_target);
	
	
	//~ ////////////////////////////////////////////////////////////////////
	//~ // GENERALIZED //
	console::print_highlight ("Generalized iterative closest point...\n");
	
	std::cout << " Loading parameters " << std::endl;
	typedef PointXYZ PointT;
	PointCloud<PointT>::Ptr src (new PointCloud<PointT>);
	copyPointCloud (cloud_source, *src);
	PointCloud<PointT>::Ptr tgt (new PointCloud<PointT>);
	copyPointCloud (cloud_target, *tgt);
	PointCloud<PointT>::Ptr output (new PointCloud<PointT>);
	//~ copyPointCloud (*src, *output);
	
	GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
	gicp.setInputSource (src);
	gicp.setInputTarget (tgt);
	
	// Set the max correspondence distance to 1m (e.g., correspondences with higher distances will be ignored)
	double maxCorDist = 1;
	if (argv[3]!=NULL && atof(argv[3])>0)	maxCorDist = atof(argv[3]);
	gicp.setMaxCorrespondenceDistance (maxCorDist);
	
	// Set the maximum number of iterations (criterion 1)
	int iterations = 1;
	if (argv[4]!=NULL && atoi(argv[4])>0)	iterations = atoi(argv[4]);
	gicp.setMaximumIterations (iterations);
	
	// Set the transformation epsilon (criterion 2)
	double epsilon = 1e-8;
	if (argv[5]!=NULL && atof(argv[5])>0)	epsilon = atof(argv[5]);
	gicp.setTransformationEpsilon (epsilon);
	
	// Set the euclidean distance difference epsilon (criterion 3)
	double difDistEpsilon = 1;
	if (argv[6]!=NULL && atoi(argv[6])>0)	difDistEpsilon = atoi(argv[6]);
	gicp.setEuclideanFitnessEpsilon (difDistEpsilon);
	
	pcl::console::TicToc time;
	time.tic ();
	
	// Register
	std::cout << " Align " << std::endl;
	gicp.align (*output);
	if (!gicp.hasConverged ()) {
		PCL_ERROR ("\ngicp has not converged.\n");
		return (-1);
	}
	//~ std::cout << " \ngicp has converged with score of  " << gicp.getFitnessScore () << std::endl;
	
	//~ bool check = false;
	//~ if (check)
	//~ {
		//~ // Check again, for all possible caching schemes
		//~ for (int iter = 0; iter < 4; iter++)
		//~ {
			//~ std::cout << " \tCheck " << iter+1 << std::endl;
			//~ bool force_cache = (bool) iter/2;
			//~ bool force_cache_reciprocal = (bool) iter%2;
			//~ pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
			//~ // Ensure that, when force_cache is not set, we are robust to the wrong input
			//~ if (force_cache)
				//~ tree->setInputCloud (tgt);
			//~ gicp.setSearchMethodTarget (tree, force_cache);
			
			//~ pcl::search::KdTree<PointT>::Ptr tree_recip (new pcl::search::KdTree<PointT>);
			//~ if (force_cache_reciprocal)
				//~ tree_recip->setInputCloud (src);
			//~ gicp.setSearchMethodSource (tree_recip, force_cache_reciprocal);
			
			//~ // Register
			//~ gicp.align (*output);
			//~ if (!gicp.hasConverged ()) {
				//~ std::cout << " \tgicp has not converged " << std::endl;
			//~ }
			//~ std::cout << " \tgicp has converged with score of  " << gicp.getFitnessScore () << std::endl;
		//~ }
	//~ }
	
	//~ // Test guess matrix
	//~ std::cout << " Add test guess matrix " << std::endl;
	//~ Eigen::Isometry3f transform = Eigen::Isometry3f (Eigen::AngleAxisf (0.25 * M_PI, Eigen::Vector3f::UnitX ())
													//~ * Eigen::AngleAxisf (0.50 * M_PI, Eigen::Vector3f::UnitY ())
													//~ * Eigen::AngleAxisf (0.33 * M_PI, Eigen::Vector3f::UnitZ ()));
	//~ transform.translation () = Eigen::Vector3f (0.1, 0.2, 0.3);
	//~ PointCloud<PointT>::Ptr transformed_tgt (new PointCloud<PointT>);
	//~ pcl::transformPointCloud (*tgt, *transformed_tgt, transform.matrix ()); // transformed_tgt is now a copy of tgt with a transformation matrix applied
	
	//~ GeneralizedIterativeClosestPoint<PointT, PointT> gicp_guess;
	//gicp_guess.setInputSource (src);
	//~ gicp_guess.setInputSource (output);
	//~ gicp_guess.setInputTarget (transformed_tgt);
	//~ gicp_guess.setMaxCorrespondenceDistance (maxCorDist);
	//~ gicp_guess.setMaximumIterations (iterations);
	//~ gicp_guess.setTransformationEpsilon (epsilon);
	//~ gicp_guess.setEuclideanFitnessEpsilon (difDistEpsilon);
	//~ gicp_guess.align (*output, transform.matrix ());
	
	//~ if (!gicp_guess.hasConverged ()) {
		//~ PCL_ERROR ("\tgicp_guess has not converged.\n");
		//~ return (-1);
	//~ }
	print_info ("GICP has converged with score of "); print_value ("%f", gicp.getFitnessScore ()); print_info (" in "); print_value ("%g", time.toc ()); print_info (" ms : \n");
	std::cout << std::endl;
	
	Eigen::Matrix4f trans_final = gicp.getFinalTransformation();
	std::cout << " FINAL " << std::endl;
	print4x4Matrix(trans_final);
	
	// apply transform to data cloud (to fit model cloud)
    console::print_highlight (" apply transform  ...\n");
    pcl::transformPointCloud(cloud_source, *output, trans_final);
	
	// Compute the Hausdorff distance
	pcl::console::print_highlight ("Hausdorff\n");
	compute_Hausdorff (cloud_target, *output);
	
	// Visualization
	vizu (cloud_source, cloud_target, *output, iterations);
	
	return 0;
}








