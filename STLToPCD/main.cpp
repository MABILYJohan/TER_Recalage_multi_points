#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char *argv[])
{
	
	// pcl object
	//pcl::PointCloud<pcl::PointXYZRGB> * mycloud = new pcl::PointCloud<pcl::PointXYZRGB>();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZRGB>);// = new pcl::PointCloud<pcl::PointXYZRGB>()
	// vtk reader
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	
	int i;
	
	for(i=1; i<argc; i++)
	{
		printf("%s\n",argv[i]);
		reader->SetFileName(argv[i]);
		reader->Update();

		// convert vtk to pcl object
		pcl::io::vtkPolyDataToPointCloud(polydata, *mycloud);
		
		// Sauvegarde du nuage au format PCD
		std::stringstream ss;
		ss << i-1 << ".pcd";
		pcl::io::savePCDFileASCII (ss.str (), *mycloud);
	}
	
	
	return 0;
}
