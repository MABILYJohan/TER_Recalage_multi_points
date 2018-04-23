/************
 * TEST ICP *
 ************/


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


#include <pcl/registration/joint_icp.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>


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
	grid.setLeafSize (0.5, 0.5, 0.5);
	grid.setInputCloud (originalCloud);
	grid.filter (reductCloud);
	
	return reductCloud;
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
		printf ("%s file1.stl file2.stl\n\n", argv[0]);
		exit (0);
	}
	
	console::print_highlight ("Loading point clouds...\n");
	// Checking program arguments
	if (argc-1 < 2)
	{
		printf ("Usage :\n");
		printf ("\t\t%s file1.stl file2.stl\n", argv[0]);
		PCL_ERROR ("Provide two stl files.\n");
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
	
	PointCloud<PointXYZ> cloud_source = downSample_cloud (BIGcloud_source);;
	PointCloud<PointXYZ> cloud_target = downSample_cloud (BIGcloud_target);
	PointCloud<PointXYZ> cloud_reg;
	PointCloud<PointXYZRGBA> cloud_with_color;
	
	
	////////////////////////////////////////////////////////////////////
	// NORMAL //
	
	//~ console::print_highlight ("iterative closest point...\n");
	//~ IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	//~ // Set the input source and target
	//~ icp.setInputSource (cloud_source);
	//~ icp.setInputTarget (cloud_target);
	
	//~ // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	//~ double maxCorDist = 0.05;
	//~ if (argv[3]!=NULL && atof(argv[3])>0)	maxCorDist = atof(argv[3]);
	//~ icp.setMaxCorrespondenceDistance (maxCorDist);
	
	//~ // Set the maximum number of iterations (criterion 1)
	//~ int iterations = 1;
	//~ if (argv[4]!=NULL && atoi(argv[4])>0)	iterations = atoi(argv[4]);
	//~ icp.setMaximumIterations (iterations);
	
	//~ // Set the transformation epsilon (criterion 2)
	//~ double epsilon = 1e-8;
	//~ if (argv[5]!=NULL && atof(argv[5])>0)	epsilon = atof(argv[5]);
	//~ icp.setTransformationEpsilon (epsilon);
	
	//~ // Set the euclidean distance difference epsilon (criterion 3)
	//~ double difDistEpsilon = 1;
	//~ if (argv[6]!=NULL && atoi(argv[6])>0)	difDistEpsilon = atoi(argv[6]);
	//~ icp.setEuclideanFitnessEpsilon (difDistEpsilon);
	
	//~ // Perform the alignment
	//~ icp.align (*cloud_source_registered);
	//~ if (!icp.hasConverged ()) {
		//~ PCL_ERROR ("\nICP has not converged.\n");
		//~ return (-1);
	//~ }
	
	//~ std::cout << " \nICP has converged, score is  " << icp.getFitnessScore () << std::endl;
	//~ // Obtain the transformation that aligned cloud_source to cloud_source_registered
	//~ Eigen::Matrix4f transformation = icp.getFinalTransformation ();
	
	//~ std::cout << " Matrix " << std::endl;
	//~ print4x4Matrix (transformation);
	
	
	////////////////////////////////////////////////////////////////////
	// JOINT //
	
	console::print_highlight ("Jointiterative closest point...\n");
	// Set up
	JointIterativeClosestPoint<PointXYZ, PointXYZ> reg;
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
		Eigen::Matrix4f trans_final = reg.getFinalTransformation ();
		//~ for (int y = 0; y < 4; y++)
			//~ for (int x = 0; x < 4; x++)
				//~ EXPECT_NEAR (trans_final (y, x), delta_transform (y, x), 1E-2);
		
		print4x4Matrix(trans_final);
		
		//~ EXPECT_TRUE (cloud_reg.empty ()); // By definition, returns an empty cloud
		
		// Clear
		reg.clearInputSources ();
		reg.clearInputTargets ();
	}
	
	////////////////////////////////////////////////////////////////////
	
	
	
	//~ // Visualization
	//~ pcl::visualization::PCLVisualizer viewer ("ICP");
	
	//~ PointCloud<PointXYZ>* cs = &cloud_source;
	//~ PointCloud<PointXYZ>* ct = &cloud_target;
	//~ PointCloud<PointXYZ>::Ptr cr = &cloud_reg;
	
	//~ // Create two vertically separated viewports
	//~ int v1 (0);
	//~ int v2 (1);
	//~ viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	//~ viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	
	//~ float bckgr_gray_level = 0.0;  // Black
	//~ float txt_gray_lvl = 1.0 - bckgr_gray_level;
	// Original point cloud (source) is white
	//~ pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_source_color_h 
		//~ (cs,
		 //~ (int) 255 * txt_gray_lvl,
		 //~ (int) 255 * txt_gray_lvl, 
		 //~ (int) 255 * txt_gray_lvl);
	//~ viewer.addPointCloud (cs, cloud_source_color_h, "cloud_source v1", v1 );
	//~ viewer.addPointCloud (cs, cloud_source_color_h, "cloud_source v2", v2);
	
	//~ // target cloud is green
	//~ pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_target_color_h (ct, 20, 180, 20);
	//~ viewer.addPointCloud (ct, cloud_target_color_h, "cloud_target_v1", v1);
	
	//~ // ICP aligned point cloud is red
	//~ pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_reg_color_h (cr, 180, 20, 20);
	//~ viewer.addPointCloud (cr, cloud_reg_color_h, "cloud_icp_v2", v2);
	
	//~ // Adding text descriptions in each viewport
	//~ viewer.addText ("White: cloud source\nGreen: cloud target", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	//~ viewer.addText ("White: cloud source\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);	
	//~ std::stringstream ss;
	//~ ss << maxIterations;
	//~ std::string iterations_cnt = "ICP iterations = " + ss.str ();
	//~ viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
	
	
	//~ // Set background color
	//~ viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	//~ viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
	
	//~ // Display the visualiser
	//~ while (!viewer.wasStopped ())
	//~ {
		//~ viewer.spinOnce ();
	//~ }
	
	return 0;
}








