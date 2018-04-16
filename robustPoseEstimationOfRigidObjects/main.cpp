#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int main (int argc, char **argv)
{
	
	/**
	 * Then we instantiate the necessary data containers,
	 *  check the input arguments and load the object and scene point clouds. 
	 * Although we have defined the basic point type to contain normals, 
	 *  we only have those in advance for the object (which is often the case). 
	 * We will estimate the normal information for the scene below.
	 * */
	// Point clouds
	PointCloudT::Ptr object (new PointCloudT);
	PointCloudT::Ptr object_aligned (new PointCloudT);
	PointCloudT::Ptr scene (new PointCloudT);
	FeatureCloudT::Ptr object_features (new FeatureCloudT);
	FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  
	// Get input object and scene
	if (argc != 3)
	{
		pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
		return (1);
	}
  
	// Load object and scene
	pcl::console::print_highlight ("Loading point clouds...\n");
	if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 
		|| pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
	{
		pcl::console::print_error ("Error loading object/scene file!\n");
		return (1);
	}
	
	
	/**
	 * To speed up processing, we use PCL’s VoxelGrid class to
	 *  downsample both the object and the scene point clouds
	 *  to a resolution of 5 mm.
	 * */
	// Downsample
	pcl::console::print_highlight ("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (*object);
	grid.setInputCloud (scene);
	grid.filter (*scene);
	
	
	/**
	 * The missing surface normals for the scene are now estimated 
	 * using PCL’s NormalEstimationOMP. 
	 * The surface normals are required for computing the features
	 * below used for matching.
	 * */
	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<PointNT,PointNT> nest;
	nest.setRadiusSearch (0.01);
	nest.setInputCloud (scene);
	nest.compute (*scene);
	
	
	/**
	 * For each point in the downsampled point clouds,
	 *  we now use PCL’s FPFHEstimationOMP class to compute 
	 *  Fast Point Feature Histogram (FPFH) descriptors used for
	 *  matching during the alignment process.
	 * */
	// Estimate features
	pcl::console::print_highlight ("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch (0.025);
	fest.setInputCloud (object);
	fest.setInputNormals (object);
	fest.compute (*object_features);
	fest.setInputCloud (scene);
	fest.setInputNormals (scene);
	fest.compute (*scene_features);
	
	
	/**
	 * We are now ready to setup the alignment process. 
	 * We use the class SampleConsensusPrerejective, 
	 *  which implements an efficient RANSAC pose estimation loop.
	 * This is achieved by early elimination of bad pose hypothesis
	 *  using the class CorrespondenceRejectorPoly.
	 * */
	// Perform alignment
	pcl::console::print_highlight ("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
	align.setInputSource (object);
	align.setSourceFeatures (object_features);
	align.setInputTarget (scene);
	align.setTargetFeatures (scene_features);
	align.setMaximumIterations (50000); // Number of RANSAC iterations
	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (5); // Number of nearest features to use
	align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
	align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}
	
	
	/*******************************************************************
	 * A LIRE : POUR AMELIORER L EXECUTION DU PROGRAMME
	 * -----------------------------------------------------------------
	 * Apart from the usual input point clouds and features,
	 * this class takes some additional runtime parameters which have
	 * great influence on the performance of the alignment algorithm. 
	 * The first two have the same meaning as in 
	 * the alignment class SampleConsensusInitialAlignment:
	 * 
	 * 	Number of samples - setNumberOfSamples ():
	 * 	 	The number of point correspondences to sample between 
	 *  	the object and the scene. At minimum,
	 *  	3 points are required to calculate a pose.
	 * 
	 * Correspondence randomness - setCorrespondenceRandomness ():
	 *  	Instead of matching each object FPFH descriptor to its nearest
	 *  	matching feature in the scene, we can choose between the N best
	 *  	matches at random. This increases the iterations necessary, 
	 *  	but also makes the algorithm robust towards outlier matches.
	 * 
	 * Polygonal similarity threshold - setSimilarityThreshold ():
	 *  	The alignment class uses the CorrespondenceRejectorPoly class 
	 *  	for early elimination of bad poses based on pose-invariant 
	 *  	geometric consistencies of the inter-distances between sampled
	 *  	points on the object and the scene. 
	 *  	The closer this value is set to 1, 
	 *  	the more greedy and thereby fast the algorithm becomes.
	 *  	However, this also increases the risk of eliminating
	 *  	good poses when noise is present.
	 * Inlier threshold - setMaxCorrespondenceDistance ():
     *  	This is the Euclidean distance threshold used for determining
     *  	whether a transformed object point is correctly aligned
     *  	to the nearest scene point or not. 
     *  	In this example, we have used a heuristic value of 1.5 times
     *  	the point cloud resolution.
     * Inlier fraction - setInlierFraction ():
     *  	In many practical scenarios, large parts of the observed
     *  	object in the scene are not visible, either due to clutter,
     *  	occlusions or both. In such cases,
     *  	we need to allow for pose hypotheses that do not align
     *  	all object points to the scene.
     *  	The absolute number of correctly aligned points is determined
     *  	using the inlier threshold, 
     *  	and if the ratio of this number to the total number of points
     *  	in the object is higher than the specified inlier fraction,
     *  	we accept a pose hypothesis as valid.
	 ******************************************************************/
	
	
	if (align.hasConverged ())
	{
		/**
		 * The aligned object is stored in the point cloud object_aligned.
		 * If a pose with enough inliers was found 
		 *  (more than 25 % of the total number of object points),
		 *  the algorithm is said to converge,
		 *  and we can print and visualize the results.
		 * */
		// Print results
		printf ("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation ();
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
		
		// Show alignment
		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
		visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
		visu.spin ();
	}
	else
	{
		pcl::console::print_error ("Alignment failed!\n");
		return (1);
	}
	
	return (0);
}
