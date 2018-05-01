/*****************
 * TEST JointICP *
 *****************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/io/pcd_io.h>


#include <pcl/registration/joint_icp.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "utils_pcl.h"

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


void sampleRandomTransform (Eigen::Affine3f &trans, float max_angle, float max_trans)
{
	srand(0);
	// Sample random transform
	Eigen::Vector3f axis((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
	axis.normalize();
	float angle = (float)rand() / RAND_MAX * max_angle;
	Eigen::Vector3f translation((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
	translation *= max_trans;
	Eigen::Affine3f rotation(Eigen::AngleAxis<float>(angle, axis));
	trans = Eigen::Translation3f(translation) * rotation;
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
	PointCloud<PointXYZ> cloud_reg;
	PointCloud<PointXYZRGBA> cloud_with_color;
	
	PointCloud<PointXYZ>::Ptr cloud_reg_vizu (new PointCloud<PointXYZ>);
	
	////////////////////////////////////////////////////////////////////
	// JOINT //
	
	console::print_highlight ("Jointiterative closest point...\n");
	
	// Set up
	pcl::JointIterativeClosestPoint<PointXYZ, PointXYZ> reg;
	int maxIterations = 50;
	reg.setMaximumIterations (maxIterations);
	double epsilon = 1e-8;
	reg.setTransformationEpsilon (epsilon);
	double corDist = 0.25;
	reg.setMaxCorrespondenceDistance (corDist); // Making sure the correspondence distance > the max translation
	std::cout << " maxIterations(" << maxIterations <<")\n" 
			<< " epsilon(" << epsilon <<")\n" 
			<< " corDist(" << corDist <<")\n" 
			<< std::endl;
	// Add a median distance rejector
	registration::CorrespondenceRejectorMedianDistance::Ptr rej_med (new registration::CorrespondenceRejectorMedianDistance);
	rej_med->setMedianFactor (4.0);
	reg.addCorrespondenceRejector (rej_med);
	// Also add a SaC rejector
	registration::CorrespondenceRejectorSampleConsensus<PointXYZ>::Ptr rej_samp (new registration::CorrespondenceRejectorSampleConsensus<PointXYZ>);
	reg.addCorrespondenceRejector (rej_samp);
	
	Eigen::Matrix4f trans_final;
	
	size_t ntransforms = 10;
	for (size_t t = 0; t < ntransforms; t++)
	{
		std::cout << " transform " << t << std::endl;
		// Sample a fixed offset between cloud pairs
		Eigen::Affine3f delta_transform;
		// No rotation, since at a random offset this could make it converge to a wrong (but still reasonable) result
		sampleRandomTransform (delta_transform, 0., 0.10);
		// Make a few transformed versions of the data, plus noise
		size_t nclouds = 5;
		for (size_t i = 0; i < nclouds; i++)
		{
			PointCloud<PointXYZ>::ConstPtr source (cloud_source.makeShared ());
			// Sample random global transform for each pair
			Eigen::Affine3f net_transform;
			sampleRandomTransform (net_transform, 2*M_PI, 10.);
			// And apply it to the source and target
			PointCloud<PointXYZ>::Ptr source_trans (new PointCloud<PointXYZ>);
			PointCloud<PointXYZ>::Ptr target_trans (new PointCloud<PointXYZ>);
			transformPointCloud (*source, *source_trans, delta_transform.inverse () * net_transform);
			transformPointCloud (*source, *target_trans, net_transform);
			// Add these to the joint solver
			reg.addInputSource (source_trans);
			reg.addInputTarget (target_trans);
		}
		
		// Register
		reg.align (cloud_reg);
		trans_final = reg.getFinalTransformation ();
		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
			{
				trans_final (y, x);
				delta_transform (y, x);
			}
		
		if (cloud_reg.empty () == true) printf ("cloud_reg empty\n"); // By definition, returns an empty cloud
		
		// Clear
		reg.clearInputSources ();
		reg.clearInputTargets ();
	}
	
	std::cout << " FINAL " << std::endl;
	print4x4Matrix(trans_final);
	
	////////////////////////////////////////////////////////////////////
	
	
	// Visualization
	char name[20] = "JOINT ICP";
	pcl::visualization::PCLVisualizer viewer = vizu (cloud_source,
													cloud_target,
													cloud_reg,
													name);
	
	return 0;
}








