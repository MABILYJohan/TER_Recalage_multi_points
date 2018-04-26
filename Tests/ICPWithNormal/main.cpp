/***********************
 * TEST ICPWithNormals *
 ***********************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/time.h>   // TicToc
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


//~ #include <gtest/gtest.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

#include <pcl/features/normal_3d.h>


using namespace pcl;
using namespace pcl::io;
using namespace std;


template <typename PointSource, typename PointTarget>
class RegistrationWrapper : public Registration<PointSource, PointTarget>
{
	public:
	void computeTransformation (pcl::PointCloud<PointSource> &, const Eigen::Matrix4f&) { }
	
	bool hasValidFeaturesTest ()
	{
		return (this->hasValidFeatures ());
	}
	void findFeatureCorrespondencesTest (int index, std::vector<int> &correspondence_indices)
	{
		this->findFeatureCorrespondences (index, correspondence_indices);
	}
};


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
	
	PointCloud<PointXYZ> cloud_source = downSample_cloud (BIGcloud_source);
	PointCloud<PointXYZ> cloud_target = downSample_cloud (BIGcloud_target);
	PointCloud<PointXYZ> cloud_reg;
	PointCloud<PointXYZRGBA> cloud_with_color;
	
	
	////////////////////////////////////////////////////////////////////
	// ICP W NORMAL //
	
	console::print_highlight ("titerative closest point with normals...\n");
	
	typedef PointNormal PointT;
	PointCloud<PointT>::Ptr src (new PointCloud<PointT>);
	copyPointCloud (cloud_source, *src);
	PointCloud<PointT>::Ptr tgt (new PointCloud<PointT>);
	copyPointCloud (cloud_target, *tgt);
	PointCloud<PointT> output;
	
	std::cout << " settings normalEstimationPoint" << std::endl;
	NormalEstimation<PointNormal, PointNormal> norm_est;
	norm_est.setSearchMethod (search::KdTree<PointNormal>::Ptr (new search::KdTree<PointNormal>));
	int kSearch = 10;
	norm_est.setKSearch (kSearch);
	std::cout << " kSearch (" << kSearch << ")\n" <<  std::endl;
	norm_est.setInputCloud (tgt);
	norm_est.compute (*tgt);
	
	
	std::cout << " settings IterativeClosestPoint" << std::endl;
	IterativeClosestPoint<PointT, PointT> reg;
	typedef registration::TransformationEstimationPointToPlane<PointT, PointT> PointToPlane;
	boost::shared_ptr<PointToPlane> point_to_plane (new PointToPlane);
	reg.setTransformationEstimation (point_to_plane);
	reg.setInputSource (src);
	reg.setInputTarget (tgt);
	int maxIterations = 50;
	reg.setMaximumIterations (maxIterations);
	double epsilon = 1e-8;
	reg.setTransformationEpsilon (epsilon);
	std::cout << " maxIterations(" << maxIterations <<")\n" 
			<< " epsilon(" << epsilon <<")\n" 
			<< std::endl;
	// Use a correspondence estimator which needs normals
	registration::CorrespondenceEstimationNormalShooting<PointT, PointT, PointT>::Ptr ce (new registration::CorrespondenceEstimationNormalShooting<PointT, PointT, PointT>);
	reg.setCorrespondenceEstimation (ce);
	// Add rejector
	registration::CorrespondenceRejectorSurfaceNormal::Ptr rej (new registration::CorrespondenceRejectorSurfaceNormal);
	rej->setThreshold (0); //Could be a lot of rotation -- just make sure they're at least within 0 degrees
	reg.addCorrespondenceRejector (rej);
	
	
	// Register
	std::cout << " Register" << std::endl;
	reg.align (output);
	if (!reg.hasConverged ()) {
		PCL_ERROR ("\nreg has not converged.\n");
		return (-1);
	}
	
	//~ // Check again, for all possible caching schemes
	//~ std::cout << " Check" << std::endl;
	//~ for (int iter = 0; iter < 4; iter++)
	//~ {
		//~ bool force_cache = (bool) iter/2;
		//~ bool force_cache_reciprocal = (bool) iter%2;
		//~ pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		//~ // Ensure that, when force_cache is not set, we are robust to the wrong input
		//~ if (force_cache)
			//~ tree->setInputCloud (tgt);
		//~ reg.setSearchMethodTarget (tree, force_cache);
		
		//~ pcl::search::KdTree<PointT>::Ptr tree_recip (new pcl::search::KdTree<PointT>);
		//~ if (force_cache_reciprocal)
			//~ tree_recip->setInputCloud (src);
		//~ reg.setSearchMethodSource (tree_recip, force_cache_reciprocal);
		
		//~ // Register
		//~ std::cout << " \tRegister" << std::endl;
		//~ reg.align (output);
		//~ if (!reg.hasConverged ()) {
			//~ printf ("\treg has not converged.\n");
			//~ continue;
		//~ }
	//~ }
	
	printf ("reg has converged with score of %f\n", reg.getFitnessScore ());
	
	////////////////////////////////////////////////////////////////////
	
	
	//~ // Visualization
	pcl::visualization::PCLVisualizer viewer ("ICPWNormals");
	
	PointCloud<PointXYZ>::Ptr cloud_source_vizu (new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_target_vizu (new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_output_vizu (new PointCloud<PointXYZ>);
	
	copyPointCloud (cloud_source, *cloud_source_vizu);
	copyPointCloud (cloud_target, *cloud_target_vizu);
	copyPointCloud (output, *cloud_output_vizu);
	
	//~ // Create two vertically separated viewports
	int v1 (0);
	int v2 (1);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	// Original point cloud (source) is white
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_source_color_h 
		(cloud_source_vizu,
		 (int) 255 * txt_gray_lvl,
		 (int) 255 * txt_gray_lvl, 
		 (int) 255 * txt_gray_lvl);
	viewer.addPointCloud (cloud_source_vizu, cloud_source_color_h, "cloud_source v1", v1 );
	viewer.addPointCloud (cloud_source_vizu, cloud_source_color_h, "cloud_source v2", v2);
	
	//~ // target cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_target_color_h (cloud_target_vizu, 20, 180, 20);
	viewer.addPointCloud (cloud_target_vizu, cloud_target_color_h, "cloud_target_v1", v1);
	
	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_reg_color_h (cloud_output_vizu, 180, 20, 20);
	viewer.addPointCloud (cloud_output_vizu, cloud_reg_color_h, "cloud_icp_v2", v2);
	
	// Adding text descriptions in each viewport
	viewer.addText ("White: cloud source\nGreen: cloud target", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText ("White: cloud source\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);	
	std::stringstream ss;
	ss << maxIterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str ();
	viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
	
	
	//~ // Set background color
	viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
	
	//~ // Display the visualiser
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
	
	return 0;
}








