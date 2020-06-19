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


/// Global variables
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("VisAuto"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> global_viewer(new pcl::visualization::PCLVisualizer("VisManu"));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr knobAuto (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr knobManu (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color(knobAuto, 0, 255, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(knobManu, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

int knobManuPoint = 0;
int arrAutoAll[5000];
int arrManuAll[10000];
int arrCloudAll[30000];


enum IntAction { WRITE, NEXT, PREVIOUS, ADD, DELETE, CLEAR, SHOW, NONE };
IntAction current_action = NONE;

void clickCallback(const pcl::visualization::PointPickingEvent& event);
void areaCallback(const pcl::visualization::AreaPickingEvent& event);
void keyboardCallback(const pcl::visualization::KeyboardEvent &event);
void printCommands();
void validation();
pcl::visualization::PCLVisualizer::Ptr VisAuto (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr knobAuto);
pcl::visualization::PCLVisualizer::Ptr VisManu (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr knobAuto);

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


  //function add
  global_viewer = VisManu(cloud,knobManu);
  viewer = VisAuto(cloud,knobAuto);

  // Register mouse click and keyboard callbacks
  global_viewer->registerPointPickingCallback(clickCallback);
  global_viewer->registerAreaPickingCallback(areaCallback);
  global_viewer->registerKeyboardCallback(keyboardCallback);


  while (!global_viewer->wasStopped ())
  {
    viewer->spinOnce ();
    global_viewer->spinOnce ();
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

  if(current_action == 3){ //Add
    for(unsigned i = 0; i <= knobManuPoint; i++){
      if(idx == arrManuAll[i]){
        std::cout << "Already added:" << std::endl;
        return;
      }else if(idx < 500){
        std::cout << "False click:" << std::endl;
        return;
      }
    }
    std::cout << "Point:" << std::endl;
    knobManu->points[knobManuPoint] = cloud->points[idx];
    arrManuAll[knobManuPoint] = idx;
    knobManuPoint+=1;
    std::cout << idx << " added!" << std::endl;
  }else if(current_action == 4){ //Delete
    for(unsigned i = 0; i <= knobManuPoint; i++){
      if(idx == arrManuAll[i]){
        std::cout << "Deleted:" << idx << std::endl;
        for(unsigned j = i; j <= knobManuPoint; j++){
          knobManu->points[j] = knobManu->points[j+1];
          arrManuAll[j] = arrManuAll[j+1];
        }
        knobManuPoint-=1;
      }
    }
  }

  global_viewer->updatePointCloud(knobManu, red_color, "sample knobManu");
  global_viewer->updatePointCloud(cloud, rgb, "cloud cloud");
}

void areaCallback(const pcl::visualization::AreaPickingEvent& event) {
  std::vector<int> indices;
  bool add = 0;
  int added = 0; int alreadyAdded = 0; int falseAdded = 0; int deletePtn = 0;
  if (!event.getPointsIndices(indices)) return;

  // Update the data
  for (auto &idx : indices){
    add = 1;
    if(current_action == 3){ //Add
      for(unsigned i = 0; i <= knobManuPoint; i++){
        if(idx == arrManuAll[i]){
          alreadyAdded += 1;
          add = 0;
        }else if(idx < 1000){
          falseAdded += 1;
          add = 0;
        }
      }
      if(add == 1){
          knobManu->points[knobManuPoint] = cloud->points[idx];
          arrManuAll[knobManuPoint] = idx;
          knobManuPoint+=1;
          added += 1;
      }
    }else if(current_action == 4){ //Delete
      for(unsigned i = 0; i <= knobManuPoint; i++){
        if(idx == arrManuAll[i]){
          std::cout << idx << std::endl;
          for(unsigned j = i; j <= knobManuPoint; j++){
            knobManu->points[j] = knobManu->points[j+1];
            arrManuAll[j] = arrManuAll[j+1];
          }
          knobManuPoint-=1;
          deletePtn +=1;
        }
      }
    }
  } 
  if(current_action == 3){
    std::cout << "\nNew area added" << std::endl;
    std::cout << "Points added: " << added << std::endl;
    std::cout << "Points added already: " << alreadyAdded << std::endl;
    std::cout << "False selected points: " << falseAdded << std::endl;
  }else if(current_action == 4){
    std::cout << "\nArea deleted" << std::endl;
    std::cout << "Number of deleted points: " << deletePtn << std::endl;
  }

  global_viewer->updatePointCloud(knobManu, red_color, "sample knobManu");
    global_viewer->updatePointCloud(cloud, rgb, "cloud cloud");
}

///
void keyboardCallback(const pcl::visualization::KeyboardEvent &event) {
  if (event.getKeySym() == "w" && event.keyDown()) {
    PCL_INFO("Writing to file\n");
    current_action = WRITE;
  } else if (event.getKeySym() == "v" && event.keyDown()) {
    PCL_INFO("Validation\n");
    validation();
  } else if (event.getKeySym() == "x" && event.keyDown()) {
    PCL_INFO("Select an area\n");
  } else if (event.getKeySym() == "a" && event.keyDown()) {
    PCL_INFO("Add point to object\n");
    current_action = ADD;
  } else if (event.getKeySym() == "d" && event.keyDown()) {
    PCL_INFO("Delete point from object\n");
    current_action = DELETE;
  } else if (event.getKeySym() == "h" && event.keyDown()) {
    printCommands();
  }
}


void printCommands() {
  PCL_INFO("\n---- Annotation Tool Commands ---- \n\n"
           "Display:\n"
           "VisAuto - Computer search for knob\n"
           "  GREEN - Points found as knob\n"
           "VisManu - Your selection for knob\n"
           "  RED   - Points you have selected for knob\n\n"
           "Selecting points:\n"
           "  Click 'a' for adding points and 'd' for unselecting points"
           "  Hold SHIFT key and click a point with the left mouse button to select/unselect it\n"
           "  In AREA mode (toggle with 'x' key), drag a 2D box around the points to be selected\n"
           "Keyboard commands\n"
           "  a - to add the selected points to the current object\n"
           "  d - to remove the selected points from the current object\n"
           "  x - toggle AREA mode\n"
           "  h - to display this message\n"
           "  v - validation of the model"
           "  q - to quit\n\n"
           "----------------------------------\n\n");
}

void validation(){
  int sizeArrAutoAll = sizeof(arrAutoAll)/sizeof(arrAutoAll[0]);
  int sizeArrManuAll = sizeof(arrManuAll)/sizeof(arrManuAll[0]);

  int numPointAuto = 0; int numPointManu = 0;
  int numPointCloud = cloud->size();
  bool FPbool = 0;
  int TP = 0;
  int FP = 0;
  int FN = 0;
  int TN = 0;

  for(unsigned i = 0; i < sizeArrAutoAll; i++){
    if(arrAutoAll[i] != 0){
      numPointAuto+=1;
    }
  }
  for(unsigned i = 0; i < sizeArrManuAll; i++){
    if(arrManuAll[i] != 0){
      numPointManu+=1;
    }
  }


  int arrAutoSel[numPointAuto];
  int arrManuSel[numPointManu];

  for(unsigned i = 0; i <= numPointAuto; i++){
    arrAutoSel[i] = arrAutoAll[i];
  }

  for(unsigned i = 0; i <= numPointManu; i++){
    arrManuSel[i] = arrManuAll[i];
  }

  std::cout << "Number of computer selected points: " << numPointAuto << std::endl;
  std::cout << "Number of manualy selected points: " << numPointManu << std::endl;
  std::cout << "Number of all points: " << numPointCloud << std::endl;

  for(unsigned int i = 0; i < numPointAuto; i++){
    FPbool = 1;
    for(unsigned int j = 0; j < numPointManu; j++){
      if(arrAutoSel[i] == arrManuSel[j]){
        TP += 1;
        FPbool = 0;
      }
    }
    if(FPbool == 1){
      FP += 1;
    }
  }

  FN = (numPointManu) - (TP);
  TN = numPointCloud - (TP+FP+FN);


  std::cout << "TP: " << TP << std::endl;
  std::cout << "FP: " << FP << std::endl;
  std::cout << "FN: " << FN << std::endl;
  std::cout << "TN: " << TN << std::endl;

  double precision = TP / (double)(TP + FP);
  double recall = TP / (double)(TP + FN);
  double F1 = 2*precision*recall / (precision + recall);
  std::cout << "Precision: " << setprecision(4) << precision << " %" << std::endl;
  std::cout << "Recall: " << setprecision(4) << recall << " %" << std::endl;
  std::cout << "F1: " << setprecision(4) << F1 << " %" << std::endl;
}

pcl::visualization::PCLVisualizer::Ptr VisAuto (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr knobAuto)
{
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (knobAuto, green_color, "sample knobAuto");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample knobAuto");
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr VisManu (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr knobAuto)
{
  global_viewer->setBackgroundColor (0, 0, 0);
  global_viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud cloud");
  global_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud cloud");
  global_viewer->addPointCloud<pcl::PointXYZRGB> (knobManu, red_color, "sample knobManu");
  global_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample knobManu");
  global_viewer->initCameraParameters ();
  global_viewer->initCameraParameters ();
  return (global_viewer);
}