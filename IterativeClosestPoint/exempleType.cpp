#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/registration/icp.h>

int main(int argc, char *argv[])
{
	
	/*
	 * Creates two pcl::PointCloud<pcl::PointXYZ> boost shared pointers and initializes them
	 * */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
	
	/************************
	 *	struct PointXYZ		*
	 *	{					*
	 *	  float x;			*
	 *	  float y;			*
	 *	  float z;			*
	 *	};					*
	 ************************/
	
	// Fill in the CloudIn data
	/**
	 * fill in the PointCloud structure with random point values,
	 * and set the appropriate parameters (width, height, is_dense).
	 * Also, they output the number of points saved,
	 * and their actual data values.
	**/
	cloud_in->width    = 5;
	cloud_in->height   = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize (cloud_in->width * cloud_in->height);
	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}
	std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
		<< std::endl;
	for (size_t i = 0; i < cloud_in->points.size (); ++i) 
	{	
		std::cout << "    " <<
			cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
			cloud_in->points[i].z << std::endl;
	}
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;
	
	
	/**
	 * performs a simple rigid transform on the pointcloud and again outputs the data values.
	* */
	for (size_t i = 0; i < cloud_in->points.size (); ++i) {
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	}
	std::cout << "Transformed " << cloud_in->points.size () << " data points:"
		<< std::endl;
	for (size_t i = 0; i < cloud_out->points.size (); ++i) {
		std::cout << "    " << cloud_out->points[i].x << " " <<
			cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	}
	
	
	
	//~ // pcl object
	//~ //pcl::PointCloud<pcl::PointXYZRGB> * mycloud = new pcl::PointCloud<pcl::PointXYZRGB>();
	//~ pcl::PointCloud<pcl::PointXYZRGB>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZRGB>);// = new pcl::PointCloud<pcl::PointXYZRGB>()
	//~ // vtk reader
	//~ vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	//~ vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	//~ reader->SetFileName("ball.stl");
	//~ reader->Update();

	//~ // convert vtk to pcl object
	//~ pcl::io::vtkPolyDataToPointCloud(polydata, *mycloud);

	//~ std::cout << mycloud->points.size() << std::endl;
	//~ for (size_t i = 0; i < mycloud->points.size(); ++i)
		//~ std::cout << mycloud->points[i].x << " " 
		//~ << mycloud->points[i].y << " " 
		//~ << mycloud->points[i].z << std::endl;

	//~ pcl::visualization::CloudViewer viewer_in("cloud_in");
	//~ viewer_in.showCloud(cloud_in);
	//~ while (!viewer_in.wasStopped())
	//~ {
	//~ }
	//~ getchar();
	//~ pcl::visualization::CloudViewer viewer_out("cloud_out");
	//~ viewer_out.showCloud(cloud_out);
	//~ while (!viewer_out.wasStopped())
	//~ {
	//~ }
	//~ getchar();
	
	
	/**
	 * This creates an instance of an IterativeClosestPoint
	 * and gives it some useful information. 
	 * “icp.setInputCloud(cloud_in);” sets cloud_in as the PointCloud
	 *  to begin from and “icp.setInputTarget(cloud_out);”
	 *  sets cloud_out as the PointCloud which we want cloud_in to look like.
	**/ 
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	
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
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	return 0;
}
