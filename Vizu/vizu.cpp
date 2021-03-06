#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

/*
 * Permet de visualiser les fichiers STL et PCD
*/

int main(int argc, char *argv[])
{
	std::string file = argv[1];
	
	std::size_t pos = file.find_last_of(".");
	std::string ext = file.substr(pos);
	
	std::cout << ext << std::endl;
	
	// Nuage de point
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZRGB>);// = new pcl::PointCloud<pcl::PointXYZRGB>()
	
	if(ext.compare(".stl") == 0) // Si c'est un fichier au format STL
	{
		// vtk reader
		vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
		vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
		reader->SetFileName(argv[1]);
		reader->Update();

		// convert vtk to pcl object
		pcl::io::vtkPolyDataToPointCloud(polydata, *mycloud);
	}
	else if(ext.compare(".pcd")==0) // Si c'est un fichier au format PCD
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file, *mycloud) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}
	}
	else
	{
		std::cout << "Les format supporter sont STL et PCD" << std::endl;
		exit(1);
	}
	
	/*std::cout << mycloud->points.size() << std::endl;
	for (size_t i = 0; i < mycloud->points.size(); ++i)
		std::cout << mycloud->points[i].x << " " 
		<< mycloud->points[i].y << " " 
		<< mycloud->points[i].z << std::endl;*/

	//~ pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//~ viewer.showCloud(mycloud);

	//~ while (!viewer.wasStopped())
	//~ {
	//~ }
	
	pcl::visualization::PCLVisualizer viewer ("viewer");
	// The color we will be using
	float bckgr_gray_level = 1.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> mycloud_color_h 
		(mycloud,
		 (int) 255,
		 (int) 0, 
		 (int) 0);
	viewer.addPointCloud (mycloud, mycloud_color_h, "mycloud");
	viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize (1280, 1024);  // Visualiser window size
	
	// Display the visualiser
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
	
	getchar();
	return 0;
}
