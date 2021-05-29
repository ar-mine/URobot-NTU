#include <iostream>
#include <algorithm> 
#include <string>
#include <thread>
// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
// Realsense
#include <librealsense2/rs.hpp> 

using namespace std;

class PointsHandle
{
public:
    PointsHandle();
    PointsHandle(string name)
    {
        model_name = name;
        if(readfile() < 0)
            exit(-1);

        viewer_init();
        thread th_input(&PointsHandle::input_thread, this);
        cout<<cloud->points.size()<<endl;
        while(!viewer->wasStopped())
        {   
            viewer->spinOnce (100);
            loop();
            //std::this_thread::sleep_for(100ms);
        }
        th_input.join();
    }
    
    ~PointsHandle()
    {
        cout<<"Quit"<<endl;
    }

    int readfile()
    {
        // Load model from *.pcd file
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile(model_name, *cloud_in) < 0)
        {
            std::cout << "Error loading model cloud." << std::endl;
            return -1;
        }
        cout<<"Load file"<<model_name<<"successful!"<<endl;
        cloud = cloud_in;
        return 0;
    }

    int savefile(string save_name)
    {
        // Load model from *.pcd file
        if (pcl::io::savePCDFile(save_name, *cloud) < 0)
        {
            std::cout << "Error loading model cloud." << std::endl;
            return -1;
        }
        cout<<"Save file"<<model_name<<"successful!"<<endl;
        return 0;
    }

    void passFiter(string filedName, float lb, float ub, bool reverse=false)
    {
        // Create the filtering object
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName(filedName);
        pass.setFilterLimits(lb, ub);
        pass.setFilterLimitsNegative(reverse);
        pass.filter(*cloud_filtered);
        cloud = cloud_filtered;
    }

    void outlierFilter(int meanK, float thresh)
    {
        // Create the filtering object
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(thresh);
        sor.filter(*cloud_filtered);
        cloud = cloud_filtered;
    }
    
    void downsampleFilter(float res)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (res, res, res);
        sor.filter(*cloud_filtered);
        cloud = cloud_filtered;
    }

    void viewerFunction(pcl::visualization::PCLVisualizer& viewer)
    {
        ;
    }

    void viewer_init()
    {
        pcl::visualization::PCLVisualizer::Ptr view (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        view->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        view->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
        view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
        view->addCoordinateSystem (1.0);
        view->initCameraParameters ();
        viewer = view;
    }

    void loop()
    {   
        if(!input_vstr.empty())
        {
            cout<<"ok"<<endl;
            string input_command = input_vstr[0];
            if(input_command == "pf")
            {
                cloud_backup = cloud;
                passFiter(input_vstr[1], atof(input_vstr[2].c_str()), atof(input_vstr[3].c_str()));
                viewer->updatePointCloud(cloud);
            }
            else if(input_command == "of")
            {
                cloud_backup = cloud;
                outlierFilter(atoi(input_vstr[1].c_str()), atof(input_vstr[2].c_str()));
                viewer->updatePointCloud(cloud);
            }
            else if(input_command == "df")
            {
                cloud_backup = cloud;
                downsampleFilter(atof(input_vstr[1].c_str()));
                viewer->updatePointCloud(cloud);
            }
            else if(input_command == "s")
            {
                savefile(input_vstr[1]);
            }
            else if(input_command == "b")
            {
                cloud = cloud_backup;
                viewer->updatePointCloud(cloud);
            }
            cout<<cloud->points.size()<<endl;
            input_vstr.clear();
        }
    }

    void input_thread()
    {
        while(!viewer->wasStopped())
        {
            vector<string> vstr;
            string input_param;
            while(!getline(cin, input_param));
            boost::split(vstr, input_param, boost::is_any_of(" "), boost::token_compress_on);
            input_vstr = vstr;
        }
    }

private:
    vector<string> input_vstr;
    string model_name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_backup;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

string modelNameParse(int argc, char *argv[])
{
    //Model filename
    vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (filenames.size () > 1)
    {
        std::cout << "Filenames error.\n";
        exit (-1);
    }
    return argv[filenames[0]];
}

int main(int argc, char *argv[])
{
    PointsHandle p = PointsHandle(modelNameParse(argc, argv));
    return 0;
}