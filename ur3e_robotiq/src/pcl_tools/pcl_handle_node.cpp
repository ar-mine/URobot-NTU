#include <iostream>
#include <algorithm> 
#include <string>
// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
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

        pcl::visualization::CloudViewer viewer("Cloud Viewer");
        viewer.showCloud(cloud);
        viewer.runOnVisualizationThread(boost::bind(&PointsHandle::viewerFunction, this, _1));

        while(!viewer.wasStopped())
        {   
            string input_command;
            cin>>input_command;
            getchar();
            if(input_command == "p")
            {
                string input_param;
                while(!getline(cin, input_param));
                vector<string> vstr;
                cout<<"ok"<<endl;
                boost::split(vstr, input_param, boost::is_any_of(" "), boost::token_compress_on);
                cloud_backup = cloud;
                passFiter(vstr[0], atof(vstr[1].c_str()), atof(vstr[2].c_str()));
                viewer.showCloud(cloud);
            }
            else if(input_command == "o")
            {
                string input_param;
                while(!getline(cin, input_param));
                vector<string> vstr;
                cout<<"ok"<<endl;
                boost::split(vstr, input_param, boost::is_any_of(" "), boost::token_compress_on);
                cloud_backup = cloud;
                outlierFilter(atoi(vstr[0].c_str()), atof(vstr[1].c_str()));
                viewer.showCloud(cloud);
            }
            else if(input_command == "s")
            {
                string input_param;
                while(!getline(cin, input_param));
                cout<<"ok"<<endl;
                savefile(input_param);
            }
            else if(input_command == "b")
            {
                cloud = cloud_backup;
                viewer.showCloud(cloud);
            }
        }
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
 
    void viewerFunction(pcl::visualization::PCLVisualizer& viewer)
    {
        ;
    }

private:
    string model_name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_backup;
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