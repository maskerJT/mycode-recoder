function: show the plyfile in a dead loop

#include <iostream>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <windows.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。

#include <boost/filesystem/operations.hpp>

using namespace std;
using namespace std::chrono_literals;
using namespace boost::filesystem;


int jtPly2Pcd(const string& filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud)
{
    pcl::PolygonMesh mesh;
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::loadPolygonFilePLY(filepath, mesh);									//PCL利用VTK的IO接口，可以直接读取stl,ply,obj等格式的三维点云数据,传给PolygonMesh对象
    pcl::io::mesh2vtk(mesh, polydata);												//将PolygonMesh对象转化为vtkPolyData对象
    pcl::io::vtkPolyDataToPointCloud(polydata, *ptcloud);								//获取点云
    //pcl::io::savePCDFileASCII("robust.pcd", *ptcloud);									//存储为pcb文件，如果不需要pcd点云甚至可以不保存
    return 0;
}


int main(void)
{
    int filelabel = 1;
    int filecount = 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "Generating example point clouds.\n";
    string text = "This is PCL viewer";
    string cloudname = "cloud1";
    string filename = "file1.ply";
    string prefixpath = "C:\\Users\\jiang\\source\\repos\\Project1\\Project1";
    string fullpath = prefixpath + "\\"+filename;
    cout << "the full path is :" << fullpath<<endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);


    while (exists(fullpath))
    {
        cout << "there exitsts the fullpath:" << fullpath << endl;
        cloud1->clear();
        jtPly2Pcd(filename, cloud1);
        viewer->removeAllPointClouds();
        viewer->addPointCloud<pcl::PointXYZ>(cloud1, cloudname);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudname);
        viewer->initCameraParameters();
        //viewer->addText(text,20,0,"cloud");
        viewer->setWindowName(text);
        for (long i=1; i < 50; i++)
        {
            viewer->spinOnce(100);
        }
        if (filelabel == 5)
        {
            filelabel = 1;
        }
        else
            filelabel++;
        filename = "file" + to_string(filelabel)+".ply";
        fullpath = prefixpath + "\\" + filename;
        cloudname = "cloud" + to_string(filecount);
        //std::this_thread::sleep_for(3000ms);
        cout << cloud1->points.size() << endl;
    }
    //输出点云文件中点的个数
    cloud1->height = 1;
    viewer->addPointCloud<pcl::PointXYZ>(cloud1, "could1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "could1");
    viewer->initCameraParameters();
    viewer->spinOnce();


    while (!viewer->wasStopped())
    {
        cout << "dead!" << endl;
        std::this_thread::sleep_for(100ms);
        viewer->spinOnce();
    }
}
