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
	PointCloud<PointXYZ>::Ptr cloud_reg (new PointCloud<PointXYZ>);
	
	////////////////////////////////////////////////////////////////////
	// JOINT //
	
	console::print_highlight ("Jointiterative closest point...\n");
	
	// Set up
	pcl::JointIterativeClosestPoint<PointXYZ, PointXYZ> jicp;
	// Set the max correspondence distance to 1m (e.g., correspondences with higher distances will be ignored)
	double maxCorDist = 1;
	if (argv[3]!=NULL && atof(argv[3])>0)	maxCorDist = atof(argv[3]);
	jicp.setMaxCorrespondenceDistance (maxCorDist);
	
	// Set the maximum number of iterations (criterion 1)
	int iterations = 1;
	if (argv[4]!=NULL && atoi(argv[4])>0)	iterations = atoi(argv[4]);
	jicp.setMaximumIterations (iterations);
	
	//~ // Set the transformation epsilon (criterion 2)
	//~ double epsilon = 1e-8;
	//~ if (argv[5]!=NULL && atof(argv[5])>0)	epsilon = atof(argv[5]);
	//~ icp.setTransformationEpsilon (epsilon);
	
	//~ // Set the euclidean distance difference epsilon (criterion 3)
	//~ double difDistEpsilon = 1;
	//~ if (argv[6]!=NULL && atoi(argv[6])>0)	difDistEpsilon = atoi(argv[6]);
	//~ icp.setEuclideanFitnessEpsilon (difDistEpsilon);
	
	std::cout << " maxIterations(" << iterations <<")\n" 
			<< " maxCorDist(" << maxCorDist <<")\n" 
			<< std::endl;
			
	//
	PointCloud<PointXYZ>::Ptr src (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_source, *src);
	PointCloud<PointXYZ>::Ptr tgt (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_target, *tgt);
	
	Eigen::Matrix4f trans_final;
	
	copyPointCloud (*src, *cloud_reg);
	jicp.addInputSource (cloud_reg);
	jicp.addInputTarget (tgt);
	//
	
	
	//~ // Add a median distance rejector
	//~ registration::CorrespondenceRejectorMedianDistance::Ptr rej_med (new registration::CorrespondenceRejectorMedianDistance);
	//~ rej_med->setMedianFactor (4.0);
	//~ jicp.addCorrespondenceRejector (rej_med);
	//~ // Also add a SaC rejector
	//~ registration::CorrespondenceRejectorSampleConsensus<PointXYZ>::Ptr rej_samp (new registration::CorrespondenceRejectorSampleConsensus<PointXYZ>);
	//~ jicp.addCorrespondenceRejector (rej_samp);
	
	//~ Eigen::Matrix4f trans_final;
	
	//~ size_t ntransforms = 10;
	//~ for (size_t t = 0; t < ntransforms; t++)
	//~ {
		//~ std::cout << " transform " << t << std::endl;
		//~ // Sample a fixed offset between cloud pairs
		//~ Eigen::Affine3f delta_transform;
		//~ // No rotation, since at a random offset this could make it converge to a wrong (but still reasonable) result
		//~ sampleRandomTransform (delta_transform, 0., 0.10);
		//~ // Make a few transformed versions of the data, plus noise
		//~ size_t nclouds = 5;
		//~ for (size_t i = 0; i < nclouds; i++)
		//~ {
			//~ PointCloud<PointXYZ>::ConstPtr source (cloud_source.makeShared ());
			//~ // Sample random global transform for each pair
			//~ Eigen::Affine3f net_transform;
			//~ sampleRandomTransform (net_transform, 2*M_PI, 10.);
			//~ // And apply it to the source and target
			//~ PointCloud<PointXYZ>::Ptr source_trans (new PointCloud<PointXYZ>);
			//~ PointCloud<PointXYZ>::Ptr target_trans (new PointCloud<PointXYZ>);
			//~ transformPointCloud (*source, *source_trans, delta_transform.inverse () * net_transform);
			//~ transformPointCloud (*source, *target_trans, net_transform);
			//~ // Add these to the joint solver
			//~ jicp.addInputSource (source_trans);
			//~ jicp.addInputTarget (target_trans);
		//~ }
		
		// Register
		jicp.align (*cloud_reg);
		trans_final = jicp.getFinalTransformation ();
		//~ for (int y = 0; y < 4; y++)
			//~ for (int x = 0; x < 4; x++)
			//~ {
				//~ trans_final (y, x);
				//~ delta_transform (y, x);
			//~ }
		
		//~ if (cloud_reg.empty () == true) printf ("cloud_reg empty\n"); // By definition, returns an empty cloud
		
		//~ // Clear
		//~ jicp.clearInputSources ();
		//~ jicp.clearInputTargets ();
	//~ }
	
	std::cout << " FINAL " << std::endl;
	print4x4Matrix(trans_final);
	
	////////////////////////////////////////////////////////////////////
	
	//~ copyPointCloud (*src, *cloud_reg);
	pcl::transformPointCloud(*src, *cloud_reg, trans_final);
	
	// Compute the Hausdorff distance
	pcl::console::print_highlight ("Hausdorff\n");
	compute (*tgt, *cloud_reg);
	
	// Visualization
	vizu (*src, *tgt, *cloud_reg, iterations);
	
	return 0;
}








