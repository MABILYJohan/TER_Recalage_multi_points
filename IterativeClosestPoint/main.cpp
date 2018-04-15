#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/registration/icp.h>

int main(int argc, char *argv[])
{
	
	// pcl object
	//pcl::PointCloud<pcl::PointXYZRGB> * mycloud = new pcl::PointCloud<pcl::PointXYZRGB>();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);// = new pcl::PointCloud<pcl::PointXYZRGB>()
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	
	// vtk reader
	// CLOUD 1
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	reader->SetFileName("1.1 clean.stl");
	reader->Update();
	// convert vtk to pcl object
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud1);
	
	// CLOUD 2
	reader = vtkSmartPointer<vtkSTLReader>::New();
	polydata = reader->GetOutput();
	reader->SetFileName("1.2.1 clean.stl");
	reader->Update();
	// convert vtk to pcl object
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud2);
	
	
	// Display clouds
	std::cout << cloud1->points.size() << std::endl;
	for (size_t i = 0; i < cloud1->points.size(); ++i)
		std::cout << cloud1->points[i].x << " " 
		<< cloud1->points[i].y << " " 
		<< cloud1->points[i].z << std::endl;
	std::cout << cloud2->points.size() << std::endl;
	for (size_t i = 0; i < cloud2->points.size(); ++i)
		std::cout << cloud2->points[i].x << " " 
		<< cloud2->points[i].y << " " 
		<< cloud2->points[i].z << std::endl;
	
	
	// ITERATIVE CLOSEST POINT
	/**
	 * This creates an instance of an IterativeClosestPoint
	 * and gives it some useful information. 
	 * “icp.setInputCloud(cloud_in);” sets cloud_in as the PointCloud
	 *  to begin from and “icp.setInputTarget(cloud_out);”
	 *  sets cloud_out as the PointCloud which we want cloud_in to look like.
	**/ 
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputCloud(cloud1);
	icp.setInputTarget(cloud2);
	
	/**
	 * Creates a pcl::PointCloud<pcl::PointXYZ> to which
	 * the IterativeClosestPoint can save the resultant cloud after
	 * applying the algorithm. If the two PointClouds align correctly 
	 * (meaning they are both the same cloud merely with some kind of 
	 * rigid transformation applied to one of them) 
	 * then icp.hasConverged() = 1 (true). 
	 * It then outputs the fitness score of the final transformation 
	 * and some information about it.
	**/
	//~ pcl::PointCloud<pcl::PointXYZRGB> Final;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	pcl::visualization::CloudViewer viewer_in("cloud1");
	viewer_in.showCloud(cloud1);
	while (!viewer_in.wasStopped())
	{
	}
	getchar();
	pcl::visualization::CloudViewer viewer_out("cloud2");
	viewer_out.showCloud(cloud2);
	while (!viewer_out.wasStopped())
	{
	}
	getchar();
	
	pcl::visualization::CloudViewer viewer("final");
	viewer.showCloud(Final);
	while (!viewer.wasStopped())
	{
	}
	getchar();
	
	
	//~ // Test PCD
	//~ std::stringstream ss;
    //~ ss << "final" << ".pcd";
    //~ pcl::io::savePCDFile (ss.str (), Final, true);
	
	
	return 0;
}
