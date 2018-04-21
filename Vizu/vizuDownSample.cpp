/***************************
 * VIZUALIZE A DOWN SAMPLE *
 ***************************/


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>


int main (int argc, char *argv[])
{
	
	std::cout << " loading cloud " << std::endl;
	
	// Checking program arguments
	if (argc-1 < 1)
	{
		printf ("Usage :\n");
		printf ("\t\t%s file1.stl [dimX] [dimY] [dimZ]\n", argv[0]);
		PCL_ERROR ("Provide one stl files.\n");
		return (-1);
	}
	
	std::string file = argv[1];
	
	std::size_t pos = file.find_last_of(".");
	std::string ext = file.substr(pos);
	
	std::cout << ext << std::endl;
	
	// Nuage de point
	pcl::PointCloud<pcl::PointXYZ>::Ptr BIGcloud_source (new pcl::PointCloud<pcl::PointXYZ>); 
	
	if(ext.compare(".stl") == 0) // Si c'est un fichier au format STL
	{
		// vtk reader
		vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
		vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
		reader->SetFileName(argv[1]);
		reader->Update();

		// convert vtk to pcl object
		pcl::io::vtkPolyDataToPointCloud(polydata, *BIGcloud_source);
	}
	else if(ext.compare(".pcd")==0) // Si c'est un fichier au format PCD
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *BIGcloud_source) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}
	}
	else
	{
		std::cout << "Les format supportés sont STL et PCD" << std::endl;
		exit(1);
	}
	
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	std::cout << "DownSampling" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	
	double a(0.5);
	double b(0.5);
	double c(0.5);
	if (argv[2]!=NULL && atof(argv[2])>0.1)	a = atof(argv[2]);
	if (argv[3]!=NULL && atof(argv[3])>0.1)	b = atof(argv[3]);
	if (argv[4]!=NULL && atof(argv[4])>0.1)	c = atof(argv[4]);
	
	grid.setLeafSize (a, b, c);
	grid.setInputCloud (BIGcloud_source);
	grid.filter (*cloud_source);
	
	
	
	// Visualization
	pcl::visualization::PCLVisualizer viewer ("ICP");
	
	// Create two vertically separated viewports
	int v1 (0);
	int v2 (1);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	
	// Original point cloud (source) is red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> BIGcloud_source_color_h (BIGcloud_source, 180, 20, 20);
	viewer.addPointCloud (BIGcloud_source, BIGcloud_source_color_h, "original cloud", v1 );
	
	// reducted cloudSource is green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_source_color_h (cloud_source, 20, 180, 20);
	viewer.addPointCloud (cloud_source, cloud_source_color_h, "downsampled cloud", v2);


	
	// Adding text descriptions in each viewport
	viewer.addText ("Red: original cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "1", v1);
	viewer.addText ("Green: downsampled cloud\n", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "2", v2);	
	
	// Set background color
	viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
	
	// Display the visualiser
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
	
	
	return 0;
}








