/**********************
 * TEST ICPWITHNORMAL *
 **********************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/io/pcd_io.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

#include <pcl/features/normal_3d.h>

#include "utils_pcl.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;



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
		printf ("%s file1.stl/pcd file2.stl/pcd [deep of Ktree] [maxCorDist] [iterations] [epsilon] [difDistEpsilon]\n\n", argv[0]);
		exit (0);
	}
	
	pcl::console::print_highlight ("Loading point clouds...\n");
	// Checking program arguments
	if (argc-1 < 2)
	{
		printf ("Usage :\n");
		printf ("%s file1.stl/pcd file2.stl/pcd [deep of Ktree] [maxCorDist] [iterations] [epsilon] [difDistEpsilon]\n\n", argv[0]);
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
	
	///////////////////////////////////////////////////////////////////
	// PARAMS //
	
	PointCloud<PointNormal>::Ptr nsrc (new PointCloud<PointNormal>);
	copyPointCloud (cloud_source, *nsrc);
	PointCloud<PointNormal>::Ptr ntgt (new PointCloud<PointNormal>);
	copyPointCloud (cloud_target, *ntgt);
	
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_nicp (new PointCloud<PointNormal>);
	copyPointCloud (*nsrc, *cloud_nicp);
	
	pcl::console::TicToc time;
	time.tic ();
	
	pcl::console::print_highlight ("Normal Estimation...\n");
	// Create the normal estimation class
	NormalEstimation<PointNormal, PointNormal> norm_est;
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	norm_est.setSearchMethod (search::KdTree<PointNormal>::Ptr (new search::KdTree<PointNormal>));
	
	int k = 10;
	if (argv[3]!=NULL && atoi(argv[3])>0)	k = atoi(argv[3]);
	norm_est.setKSearch (k);
	// Pass the input dataset
	norm_est.setInputCloud (ntgt);
	// Compute the features
	norm_est.compute (*ntgt);
	
	
	typedef registration::TransformationEstimationPointToPlane<PointNormal, PointNormal> PointToPlane;
	boost::shared_ptr<PointToPlane> point_to_plane (new PointToPlane);
	
	
	
	///////////////////////////////////////////////////////////////////
	// NICP //
	
	pcl::console::print_highlight ("iterative closest point with normals...\n");
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal > nicp;
	
	// Set the max correspondence distance to 1m (e.g., correspondences with higher distances will be ignored)
	double maxCorDist = 1;
	if (argv[4]!=NULL && atof(argv[4])>0)	maxCorDist = atof(argv[4]);
	nicp.setMaxCorrespondenceDistance (maxCorDist);
	
	// Set the maximum number of iterations (criterion 1)
	int iterations = 5;
	if (argv[5]!=NULL && atoi(argv[5])>0)	iterations = atoi(argv[5]);
	nicp.setMaximumIterations (iterations);
	
	// Set the transformation epsilon (criterion 2)
	double epsilon = 1e-8;
	if (argv[6]!=NULL && atof(argv[6])>0)	epsilon = atof(argv[6]);
	nicp.setTransformationEpsilon (epsilon);
	
	// Set the euclidean distance difference epsilon (criterion 3)
	double difDistEpsilon = 1;
	if (argv[7]!=NULL && atoi(argv[7])>0)	difDistEpsilon = atoi(argv[7]);
	nicp.setEuclideanFitnessEpsilon (difDistEpsilon);
	
	// Set the input source and target
	nicp.setInputSource (cloud_nicp);
	nicp.setInputTarget (ntgt);
	
	nicp.setTransformationEstimation (point_to_plane);
	nicp.setInputSource (nsrc);
	nicp.setInputTarget (ntgt);
	// Use a correspondence estimator which needs normals
	registration::CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>::Ptr ce (new registration::CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>);
	nicp.setCorrespondenceEstimation (ce);
	//~ // Add rejector
	//~ registration::CorrespondenceRejectorSurfaceNormal::Ptr rej (new registration::CorrespondenceRejectorSurfaceNormal);
	//~ rej->setThreshold (0); //Could be a lot of rotation -- just make sure they're at least within 0 degrees
	//~ nicp.addCorrespondenceRejector (rej);
	
	
	// Perform the alignment
	pcl::console::print_highlight ("Alignement...\n");
	nicp.align (*cloud_nicp);
	if (!nicp.hasConverged ()) {
		PCL_ERROR ("\nICP has not converged.\n");
		return (-1);
	}
	
	//~ std::cout << " \nNICP has converged in " << time.toc () << "ms, score is  " << nicp.getFitnessScore () << std::endl;
	print_info ("NICP has converged with score of "); print_value ("%f", nicp.getFitnessScore ()); print_info (" in "); print_value ("%g", time.toc ()); print_info (" ms : \n");
	
	Eigen::Matrix4f trans_final = nicp.getFinalTransformation();
	std::cout << " FINAL " << std::endl;
	print4x4Matrix(trans_final);
	
	
	////////////////////////////////////////////////////////////////////
	// OUTPUT //
	
	PointCloud<PointXYZ>::Ptr nicp_vizu (new PointCloud<PointXYZ>);
	copyPointCloud (*cloud_nicp, *nicp_vizu);

	//pcl::transformPointCloud(cloud_source, *cloud_nicp, trans_final);
	
	// Compute the Hausdorff distance
	pcl::console::print_highlight ("Hausdorff\n");
	compute_Hausdorff (cloud_target, *nicp_vizu);
	
	pcl::console::print_highlight ("Visualisation \n");
	vizu (cloud_source, cloud_target, *nicp_vizu, iterations);
	
	return (0);
}




