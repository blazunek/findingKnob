// Stdlib
#include <cstdlib>
#include <cmath>
#include <climits>
#include <string>
#include <thread>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <numeric>
#include <tuple>
#include <map>

// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/filesystem.hpp>


using namespace std::chrono_literals;
namespace bf = boost::filesystem;


/// Type Definitions
using PointT = pcl::PointXYZRGB;  // The point type used for input


/// Constants
const float EPSILON = 0.25f;
const float EPSILON_INTERACTION = 0.07f;
const float DEFAULT_RADIUS_SEARCH = 0.02f;
const float IGNORE_VALUE = 0.0000000;


/// Global variables
int knobManuPoint = 0;


boost::shared_ptr<pcl::visualization::PCLVisualizer> global_viewer(
new pcl::visualization::PCLVisualizer("Annotation Editor"));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr knobManu (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr knobAuto (new pcl::PointCloud<pcl::PointXYZRGB>);//ustvarimo objekt


std::unordered_set<int> current_point_indices;
int current_point_index = -1;
bool point_new = true;
int current_object_index = 0;
bool update_point = false;
bool area_selection = false;
bool showing_all = true;
enum IntAction { WRITE, NEXT, PREVIOUS, ADD, DELETE, CLEAR, SHOW, NONE };
IntAction current_action = NONE;

void clickCallback(const pcl::visualization::PointPickingEvent& event);
void areaCallback(const pcl::visualization::AreaPickingEvent& event);
void keyboardCallback(const pcl::visualization::KeyboardEvent &event);
void printCommands();
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr knobAuto);

int
main (int argc, char** argv)
{  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("5.pcd", *cloud) == -1) // naložimo file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " //potrditev, da je file naložen
            << cloud->width * cloud->height
            << std::endl;
  //std::cout << "Generating example point clouds.\n\n";

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  double maxX = 0, minX = 0, maxY = 0, minY = 0, Xdif = 0, Ydif = 0, maxZ = 0, minZ = 0, Zdif = 0;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (std::size_t i = 0; i < inliers->indices.size (); ++i)
  {
  	if(maxX > cloud->points[inliers->indices[i]].x)
			maxX = cloud->points[inliers->indices[i]].x;
	if(minX < cloud->points[inliers->indices[i]].x)
			minX = cloud->points[inliers->indices[i]].x;
	if(maxY > cloud->points[inliers->indices[i]].y)
			maxY = cloud->points[inliers->indices[i]].y;
	if(minY < cloud->points[inliers->indices[i]].y)
			minY = cloud->points[inliers->indices[i]].y;
	if(maxZ > cloud->points[inliers->indices[i]].z)
			maxZ = cloud->points[inliers->indices[i]].z;
	if(minZ < cloud->points[inliers->indices[i]].z)
			minZ = cloud->points[inliers->indices[i]].z;
  }

  Xdif = maxX-minX;
  Ydif = maxY-minY;
  Zdif = maxZ-minZ;

  knobAuto->points.resize (inliers->indices.size ());
  knobManu->points.resize (inliers->indices.size ());

  int knobAutoPoint = 0, leftAreaPoint = 0, rightAreaPoint = 0;
  float sumDistLeft = 0, sumDistRight = 0, avgDist = 0;
  int arrAutoAll[cloud->size()] = {};
 
  for(std::size_t i = 0; i < cloud->size(); ++i){
  	if(cloud->points[i].y < 0.10 && cloud->points[i].y > 0.04 && cloud->points[i].x < maxX - (Xdif * 0.9) && cloud->points[i].x > maxX - (Xdif * 0.7)){
  		sumDistLeft += abs(coefficients->values[0]*cloud->points[i].x + coefficients->values[1]*cloud->points[i].y + coefficients->values[2]*cloud->points[i].z + coefficients->values[0])/sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1]*coefficients->values[1] + coefficients->values[2]*coefficients->values[2]) ;
  		leftAreaPoint += 1;
  	}
  }
  float distLeft = sumDistLeft/leftAreaPoint;

  for(std::size_t i = 0; i < cloud->size(); ++i){
  	if(cloud->points[i].y < 0.10 && cloud->points[i].y > 0.04 && cloud->points[i].x < maxX - (Xdif * 0.3) && cloud->points[i].x > maxX - (Xdif * 0.1)){
  		sumDistRight += abs(coefficients->values[0]*cloud->points[i].x + coefficients->values[1]*cloud->points[i].y + coefficients->values[2]*cloud->points[i].z + coefficients->values[0])/sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1]*coefficients->values[1] + coefficients->values[2]*coefficients->values[2]) ;
  		rightAreaPoint += 1;
  	}
  }
  float distRight = sumDistRight/rightAreaPoint;
  std::cout << "R:" << distRight << std::endl;
  std::cout << "L:" <<distLeft << std::endl;
  float borderRight = 0, borderLeft = 0;

  if(distLeft < distRight){
  	borderLeft = 0.9;
  	borderRight = 0.7;
  	avgDist = distLeft;
  }else{
  	borderLeft = 0.3;
  	borderRight = 0.1;
  	avgDist = distRight;
  }

  for (std::size_t i = 0; i < cloud->size(); ++i)
  {
  	if(cloud->points[i].y < 0.10 && cloud->points[i].y > 0.04 && cloud->points[i].x < maxX - (Xdif * borderLeft) && cloud->points[i].x > maxX - (Xdif * borderRight)){
  		if(abs(coefficients->values[0]*cloud->points[i].x + coefficients->values[1]*cloud->points[i].y + coefficients->values[2]*cloud->points[i].z + coefficients->values[0])/sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1]*coefficients->values[1] + coefficients->values[2]*coefficients->values[2]) < (avgDist*0.993)) {
	  		knobAuto->points[knobAutoPoint].x = cloud->points[i].x;
	  		knobAuto->points[knobAutoPoint].y = cloud->points[i].y;
	  		knobAuto->points[knobAutoPoint].z = cloud->points[i].z;
        arrAutoAll[knobAutoPoint] = i;
	  		knobAutoPoint += 1;
  		}
  	}
  } 

  std::cout << knobAutoPoint << std::endl;

  int arrAuto[knobAutoPoint];
  int sizeArrAuto = sizeof(arrAuto)/sizeof(arrAuto[0]);

  std::cout << sizeArrAuto << std::endl;

  for(int i = 0; i < knobAutoPoint; i++)
  {
      arrAuto[i] = arrAutoAll[i];
  }

  for (int i = 0; i < sizeArrAuto; ++i)
  {
    std::cout << i << ": " << arrAuto[i] << " , " << arrAutoAll[i] << std::endl;
  }
  /*

  std::cout << "Zmax: " << maxZ << " Zmin: " << minZ << std::endl;

  std::cout << sizeof(arrAutoAll) << " , " << sizeof(arrAuto) << std::endl;*/

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  global_viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud cloud");
  global_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(knobAuto, 0, 255, 0);
  global_viewer->addPointCloud<pcl::PointXYZRGB> (knobManu, single_color, "sample knobManu");
  global_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample knobManu");
  global_viewer->initCameraParameters ();

  // Register mouse click and keyboard callbacks
  global_viewer->registerPointPickingCallback(clickCallback);
  global_viewer->registerAreaPickingCallback(areaCallback);
  global_viewer->registerKeyboardCallback(keyboardCallback);

  //visualization of computer model
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = rgbVis(cloud,knobAuto);


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    global_viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  
  return (0);
}

void clickCallback(const pcl::visualization::PointPickingEvent& event) {
  int idx = event.getPointIndex();
  // Check the clicked point is valid
  if (idx == -1) return;
  float x, y, z;
  event.getPoint(x, y, z);
  if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
    PCL_WARN("NAN point");
    return;
  }

  knobManu->points[knobManuPoint] = cloud->points[idx];
  knobManuPoint+=1;
  std::cout << knobManu->points[knobManuPoint-1] << std::endl;

  // Update data
  current_point_index = idx;
  if (current_point_indices.find(idx) == current_point_indices.end()) {
    current_point_indices.insert(idx);
    point_new = true;
  } else {
    current_point_indices.erase(idx);
    point_new = false;
  }

  // Set update flag to true
  update_point = true;

  // No area selection
  area_selection = false;
}


///
void areaCallback(const pcl::visualization::AreaPickingEvent& event) {
  std::vector<int> indices;
  if (!event.getPointsIndices(indices)) return;

  // Update the data
  for (auto &i : indices)
    current_point_indices.insert(i);

  // Set update flag to true
  update_point = true;

  // Area was selected
  area_selection = true;
}


///
void keyboardCallback(const pcl::visualization::KeyboardEvent &event) {
  if (event.getKeySym() == "w" && event.keyDown()) {
    PCL_INFO("Writing to file\n");
    current_action = WRITE;
  } else if (event.getKeySym() == "n" && event.keyDown()) {
    PCL_INFO("Next object\n");
    current_action = NEXT;
  } else if (event.getKeySym() == "p" && event.keyDown()) {
    PCL_INFO("Previous object\n");
    current_action = PREVIOUS;
  } else if (event.getKeySym() == "a" && event.keyDown()) {
    PCL_INFO("Add point to object\n");
    current_action = ADD;
  } else if (event.getKeySym() == "d" && event.keyDown()) {
    PCL_INFO("Delete point from object\n");
    current_action = DELETE;
  } else if (event.getKeySym() == "k" && event.keyDown()) {
    PCL_INFO("Clear point selection\n");
    current_action = CLEAR;
  } else if (event.getKeySym() == "s" && event.keyDown()) {
    PCL_INFO("Show all objects\n");
    current_action = SHOW;
  } else if (event.getKeySym() == "h" && event.keyDown()) {
    printCommands();
  }
}


void printCommands() {
  PCL_INFO("\n---- Annotation Tool Commands ---- \n\n"
           "Display:\n"
           "  GREEN - current annotated object points\n"
           "  CYAN  - annotated points of other objects (toggle with 's' key)\n"
           "  RED   - current selected points\n\n"
           "Selecting points:\n"
           "  Hold SHIFT key and click a point with the left mouse button to select it\n"
           "  Clicking a selected point will deselect it\n"
           "  In AREA mode (toggle with 'x' key), drag a 2D box around the points to be selected\n"
           "Keyboard commands\n"
           "  a - to add the selected points to the current object\n"
           "  d - to remove the selected points from the current object\n"
           "  k - to clear the current selection of points\n"
           "  x - toggle AREA mode\n"
           "  n - to select the next object\n"
           "  p - to go back to the previous object\n"
           "  s - to turn on/off the display of all object points\n"
           "  w - to write the annotation to file\n"
           "  h - to display this message\n"
           "  q - to quit\n\n"
           "----------------------------------\n\n");
}

pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr knobAuto)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(knobAuto, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (knobAuto, single_color, "sample knobAuto");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample knobAuto");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}