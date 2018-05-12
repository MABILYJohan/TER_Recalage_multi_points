#ifndef UTILS_PCL_H
#define UTILS_PCL_H


#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>


using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

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

void compute_Hausdorff (PointCloud<PointXYZ> &cloud_a, PointCloud<PointXYZ> &cloud_b)
{
	typedef PointXYZ PointType;
	typedef PointCloud<PointXYZ> Cloud;
	// Estimate
	TicToc tt;
	tt.tic ();
	
	print_highlight (stderr, "Computing ");
	
	// compare A to B
	pcl::search::KdTree<PointType> tree_b;
	tree_b.setInputCloud (cloud_b.makeShared ());
	float max_dist_a = -std::numeric_limits<float>::max ();
	for (size_t i = 0; i < cloud_a.points.size (); ++i)
	{
		std::vector<int> indices (1);
		std::vector<float> sqr_distances (1);
		
		tree_b.nearestKSearch (cloud_a.points[i], 1, indices, sqr_distances);
		if (sqr_distances[0] > max_dist_a)
			max_dist_a = sqr_distances[0];
	}
	
	// compare B to A
	pcl::search::KdTree<PointType> tree_a;
	tree_a.setInputCloud (cloud_a.makeShared ());
	float max_dist_b = -std::numeric_limits<float>::max ();
	for (size_t i = 0; i < cloud_b.points.size (); ++i)
	{
	std::vector<int> indices (1);
	std::vector<float> sqr_distances (1);
	
	tree_a.nearestKSearch (cloud_b.points[i], 1, indices, sqr_distances);
	if (sqr_distances[0] > max_dist_b)
		max_dist_b = sqr_distances[0];
	}
	
	max_dist_a = std::sqrt (max_dist_a);
	max_dist_b = std::sqrt (max_dist_b);
	
	float dist = std::max (max_dist_a, max_dist_b);
	
	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
	print_info ("A->B: "); print_value ("%f", max_dist_a);
	print_info (", B->A: "); print_value ("%f", max_dist_b);
	print_info (", Hausdorff Distance: "); print_value ("%f", dist);
	print_info (" ]\n");
}

void vizu (PointCloud<PointXYZ> cloud_source, 
			PointCloud<PointXYZ> cloud_target,
			PointCloud<PointXYZ> cloud_icp,
			int iterations)
{
	// Visualization
	pcl::visualization::PCLVisualizer viewer ("ICP");
	
	PointCloud<PointXYZ>::Ptr cloud_source_vizu (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_source, *cloud_source_vizu);
	PointCloud<PointXYZ>::Ptr cloud_target_vizu (new PointCloud<PointXYZ>);
	copyPointCloud (cloud_target, *cloud_target_vizu);
	PointCloud<PointXYZ>::Ptr cloud_icp_vizu (new PointCloud<PointXYZ>);;
	copyPointCloud (cloud_icp, *cloud_icp_vizu);
	
	// Create two vertically separated viewports
	int v1 (0);
	int v2 (1);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	
	// Original point cloud is black
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h 
		(cloud_target_vizu,
		 bckgr_gray_level,
		 bckgr_gray_level, 
		 bckgr_gray_level);
	viewer.addPointCloud (cloud_target_vizu, cloud_tr_color_h, "cloud_tr_v1", v1);
	viewer.addPointCloud (cloud_target_vizu, cloud_tr_color_h, "cloud_tr_v2", v2);
	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h (cloud_source_vizu, 20, 180, 20);
	viewer.addPointCloud (cloud_source_vizu, cloud_in_color_h, "cloud_in_v1", v1);
	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h (cloud_icp_vizu, 180, 20, 20);
	viewer.addPointCloud (cloud_icp_vizu, cloud_icp_color_h, "cloud_icp_v2", v2);
	
	// Adding text descriptions in each viewport
	viewer.addText ("Black: cloud target\nGreen: cloud source", 10, 15, 16, bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, "icp_info_1", v1);
	viewer.addText ("Black: cloud target\nRed: ICP aligned point cloud", 10, 15, 16, bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, "icp_info_2", v2);	
	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str ();
	viewer.addText (iterations_cnt, 10, 60, 16, bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, "iterations_cnt", v2);
	
	// Set background color
	viewer.setBackgroundColor (255, 255, 255, v1);
	viewer.setBackgroundColor (255, 255, 255, v2);
	
	// Display the visualiser
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}





#endif

