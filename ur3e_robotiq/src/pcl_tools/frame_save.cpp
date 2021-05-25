#include <iostream>
#include <algorithm> 
#include <string>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

using namespace std;

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer; // pcl::PointCloud<pcl::PointXYZRGB>::Ptr

std::string model_filename;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

void parseCommandLine(int argc, char *argv[])
{
  //Model filename
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () > 1)
  {
    std::cout << "Filenames error.\n";
    exit (-1);
  }
  model_filename = argv[filenames[0]];
}

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}

int main(int argc, char *argv[])
{
    parseCommandLine(argc, argv);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //======================
    // Stream configuration
    //======================
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
    
    rs2::pipeline_profile selection = pipe.start(cfg); 

    cout<<"Pipe start"<<endl;

    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }

    cout<<"Depth sensor set"<<endl;

    // Capture a single frame and obtain depth + RGB values from it    
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    auto RGB = frames.get_color_frame();

    // Map Color texture to each point
    pc.map_to(RGB);

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

     // Convert generated Point Cloud to PCL Formatting
    auto cloud = PCL_Conversion(points, RGB);

    if (pcl::io::savePCDFile(model_filename, *cloud) < 0)
    {
        std::cout << "Error saving model cloud." << std::endl;
        return (-1);
    }
}