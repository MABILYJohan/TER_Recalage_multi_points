#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
//~ #include <pcl/console/time.h>   // TicToc

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr loadCloud (char *fileName)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud (new pcl::PointCloud<pcl::PointXYZ>);
	
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
		pcl::io::vtkPolyDataToPointCloud(polydata, *myCloud);
	}
	else if(ext.compare(".pcd")==0) // Si c'est un fichier au format PCD
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *myCloud) == -1) //* load the file
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

pcl::PointCloud<pcl::PointXYZ>::Ptr downSample_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr reductCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setLeafSize (0.5, 0.5, 0.5);
	grid.setInputCloud (originalCloud);
	grid.filter (*reductCloud);
	
	return reductCloud;
}


/*
 * Permet de visualiser des fichiers STL et PCD
*/
int main (int argc, char *argv[])
{
	if (argv[1] != NULL && strcmp(argv[1], "-h")==0) {
		std::cout << " \nUsage  " << std::endl;
		printf ("%s file1.stl file2.stl [maxCorDist] [iterations] [epsilon] [difDistEpsilon]\n\n", argv[0]);
		exit (0);
	}
	
	pcl::console::print_highlight ("Loading point clouds...\n");
	// Checking program arguments
	if (argc-1 < 2)
	{
		printf ("Usage :\n");
		printf ("\t\t%s file1.stl file2.stl [maxCorDist] [iterations] [epsilon] [difDistEpsilon]\n", argv[0]);
		PCL_ERROR ("Provide two stl files.\n");
		return (-1);
	}
	std::cout << " loading cloud src " << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr BIGcloud_a = loadCloud(argv[1]);
	std::cout << " loading cloud target " << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr BIGcloud_b = loadCloud(argv[2]);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a_registered (new pcl::PointCloud<pcl::PointXYZ>); 
	
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	pcl::console::print_highlight ("DownSampling...\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a = downSample_cloud (BIGcloud_a);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b = downSample_cloud (BIGcloud_b);
	
	
	// Visualization
	pcl::visualization::PCLVisualizer viewer ("ICP");
	
	// Create two vertically separated viewports
	int v1 (0);
	//int v2 (1);
	viewer.createViewPort (0.0, 0.0, 1.0, 1.0, v1); //viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	//viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	
	// cloud_a is green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_a_color_h (cloud_a, 20, 180, 20);
	viewer.addPointCloud (cloud_a, cloud_a_color_h, "cloud_a", v1);
	
	// cloud_b is red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_b_color_h (cloud_b, 180, 20, 20);
	viewer.addPointCloud (cloud_b, cloud_b_color_h, "cloud_b", v1);
	
	// Adding text descriptions in each viewport
	viewer.addText ("Green: cloud a", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info_1", v1);
	viewer.addText ("Red: cloud b", 15, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info_2", v1);	
	
	
	// Set background color
	viewer.setBackgroundColor ((int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, v1);
	//viewer.setBackgroundColor ((int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, v1);
	
	// Display the visualiser
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
	
	
	return 0;
}
