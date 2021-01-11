#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include<algorithm>
#include<iterator>
#include<boost/shared_ptr.hpp>
#include<pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <math.h>
#include<pcl/io/ply_io.h>
#include <string>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataReader.h>
#include <vtkTriangle.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>





typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void plyToPcd(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string pathToply){
  pcl::PLYReader Reader;
  Reader.read(pathToply, *cloud);
}


void read_PCD1(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::string pathTopcd){

          //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

          if(pcl::io::loadPCDFile<pcl::PointXYZ>(pathTopcd,*cloud)==-1)
          {
             PCL_ERROR("no pudo leer achivo .PCD \n");
          }
          printf("Se pudo leer el archivo .PCD\n");

          std::cout << "Loaded "
           << cloud->width * cloud->height
           << " data points from test_pcd.pcd with the following fields: "
           << std::endl;

           // for (std::size_t i = 0; i < cloud->points.size (); ++i)
           // std::cout << "    " << cloud->points[i].x
           //    << " "    << cloud->points[i].y
           //    << " "    << cloud->points[i].z << std::endl;
}


void save_PCD(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::string pathToOut){

  pcl::io::savePCDFileASCII(pathToOut, *cloud);
  std::cerr << "Saved  data points to test_pcd.pcd." << std::endl;
}


void vizualizeMesh(pcl::PolygonMesh &mesh){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPolygonMesh(mesh,"meshes",0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
//  viewer->updatePolygonMesh (mesh);
  while (!viewer->wasStopped ()){
      viewer->spinOnce (100);
      //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}

void vizualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    // esto es lo mas importante para vizualizar nubes de putos en el origen
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid<pcl::PointXYZ> (*cloud, centroid);
    pcl::demeanPointCloud <pcl::PointXYZ, double> (*cloud, centroid, *cloud);
    //----------------------------------------------------------------------

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));    
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud,"intensity");

    //viewer.addPointCloud<PointXYZI>(cloud,handler,"My cloud");

    viewer->addPointCloud<pcl::PointXYZ> (cloud);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    

    while (!viewer->wasStopped ()){
      viewer->spinOnce (100);
      //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


void plane_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_inliers){
 
	//PointCloudT::Ptr  cloud_inliers (new PointCloudT), cloud_outliers (new PointCloudT);
        PointCloudT::Ptr cloud_outliers (new PointCloudT);

	// Segment the ground
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr 	inliers_plane (new pcl::PointIndices);
	PointCloudT::Ptr cloud_plane (new PointCloudT);

	// Make room for a plane equation (ax+by+cz+d=0)
	plane->values.resize (4);

	pcl::SACSegmentation<PointT> seg;				// Create the segmentation object
	seg.setOptimizeCoefficients (true);				// Optional
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setDistanceThreshold (2.0f); //1.0f genial
	seg.setInputCloud (cloud);
	seg.segment (*inliers_plane, *plane);

	if (inliers_plane->indices.size () == 0) {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
	}

	// Extract inliers
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);			// Extract the inliers
	extract.filter (*cloud_inliers);		// cloud_inliers contains the plane

	// Extract outliers
	//extract.setInputCloud (cloud);		// Already done line 50
	//extract.setIndices (inliers);			// Already done line 51
	extract.setNegative (true);				// Extract the outliers
	extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane


	// Visualization
	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler (cloud, 255, 20, 20); // Plane in RED
	viewer.addPointCloud (cloud_inliers, cloud_inliers_handler, "cloud inliers");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_outliers_handler (cloud, 200, 200, 200); // Everything else in GRAY
	viewer.addPointCloud (cloud_outliers, cloud_outliers_handler, "cloud outliers");	

	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}

} 



static double vtk_area3(pcl::PolygonMesh &mesh){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh,polydata);
  pcl::io::vtkPolyDataToPointCloud(polydata,*cloud);

  double CoM[3];
  vtkIdType npts_com = 0, *ptIds_com = nullptr;
  vtkSmartPointer<vtkCellArray> cells_com = polydata->GetPolys ();

  double center[3], p1_com[3], p2_com[3], p3_com[3], totalArea_com = 0;
  double comx = 0, comy = 0, comz = 0;
  for (cells_com->InitTraversal (); cells_com->GetNextCell (npts_com, ptIds_com);)
  {
    polydata->GetPoint (ptIds_com[0], p1_com);
    polydata->GetPoint (ptIds_com[1], p2_com);
    polydata->GetPoint (ptIds_com[2], p3_com);
    vtkTriangle::TriangleCenter (p1_com, p2_com, p3_com, center);
    double area_com = vtkTriangle::TriangleArea (p1_com, p2_com, p3_com);
    comx += center[0] * area_com;
    comy += center[1] * area_com;
    comz += center[2] * area_com;
    totalArea_com += area_com;
  }
  cout<<"area: "<<totalArea_com<<endl;

  return totalArea_com;

}


void fastTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &triangles){

  // esto es lo mas importante para vizualizar nubes de putos en el origen
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid<pcl::PointXYZ> (*cloud, centroid);
  pcl::demeanPointCloud <pcl::PointXYZ, double> (*cloud, centroid, *cloud);
  //------

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (100);   //100     //It was 20
  n.compute (*normals);                //Normals are estimated using standard method.

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);//x

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  //pcl::PolygonMesh triangles;

  std::cout << "Applying surface meshing...";

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(10);      //10     //It was 0.025

  // Set typical values for the parameters
  gp3.setMu (300);   //5 //It was 2.5
  gp3.setMaximumNearestNeighbors (20);    //It was 100
  gp3.setNormalConsistency(false); //It was false

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
  pcl::io::saveVTKFile ("mesh.vtk", triangles);

  //vizualizeMesh(triangles);

 
}

int main(int argc, char* argv[]){
  if(argc==2){
  cout<< argc <<endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flat (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

  pcl::PolygonMesh mesh;
  pcl::PolygonMesh triangles;
  pcl::PolygonMesh triangles2;
  double area_total=0.0;
  double area_flat=0.0;
  double area_mountain=0.0;

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D<pcl::PointXYZ>(*cloud, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;

  
  std::string pathTopcd=argv[1];
  std::string pathToOut="filter.pcd";
  read_PCD1(cloud,pathTopcd);

  fastTriangulation(cloud,triangles);
  area_total=vtk_area3(triangles);

  plane_segmentation(cloud,cloud_flat);
  vizualizeCloud(cloud_flat);

  fastTriangulation(cloud_flat,triangles2);
  area_flat=vtk_area3(triangles2);

  save_PCD(cloud,pathToOut);

  
  area_mountain=area_total-area_flat;
  std::cout << "Information about the area " << area_total << std::endl;
  std::cout << "Total area: " << area_total << std::endl;
  std::cout << "Flat area: " << area_flat << std::endl;
  std::cout << "Mountain area: " << area_total-area_flat << std::endl;
  std::cout << "porcentage of flat area: " << (area_flat*100.0)/area_total << std::endl;
  std::cout << "porcentage of flat Mountain: " << (area_mountain*100.0)/area_total << std::endl;
  
}else{
  cout<<"no se ingreso el path del archivo .ply"<<endl;
}
}
