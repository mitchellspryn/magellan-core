#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <random>

#include <Eigen/Dense>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/features/don.h>
#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/octree/octree_search.h>

// TODO: can we get this to work
//#include <pcl/cuda/filters/voxel_grid.h>

void readFile(const std::string& fileName, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    pcl::PCLPointCloud2 tempCloud;
    pcl::PCDReader reader;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int pcdVersion;
    int dataType;
    int offset = 0;
    unsigned int dataIndex;

    int status = reader.read(
        fileName,
        tempCloud,
        origin,
        orientation,
        pcdVersion,
        offset);

    if (status != 0) 
    {   
        throw std::runtime_error("Could not read pcl file. Read returns " 
                + std::to_string(status)
                + ".");
    }

    pcl::fromPCLPointCloud2(tempCloud, *cloud);
}

void downsampleCloud(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud)
{
    //std::vector<int> dummy;
    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    //pcl::removeNaNFromPointCloud(*inputCloud, *tmp1, dummy);
    //pcl::removeNaNNormalsFromPointCloud(*tmp1, *tmp2, dummy);

    pcl::VoxelGrid<pcl::PointXYZRGBNormal> filter;
    filter.setInputCloud(inputCloud);
    filter.setLeafSize(0.0381f, 0.0381f, 0.0381f);
    filter.filter(*outputCloud);
}

void downsampleCloudOrdered(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud)
{
    int downsampleFactor = 2;

    int outputWidth = static_cast<int>(std::ceil(inputCloud->width / downsampleFactor));
    int outputHeight = static_cast<int>(std::ceil(inputCloud->height / downsampleFactor));
    outputCloud->resize(outputWidth*outputHeight);
    outputCloud->width = outputWidth;
    outputCloud->height = outputHeight;

    for (int y = 0; y < outputCloud->height; y++)
    {
        for (int x = 0; x < outputCloud->width; x++)
        {
            int inputIdx = (y*downsampleFactor*inputCloud->width) + (x*downsampleFactor);
            int outputIdx = (y*outputCloud->width) + x;

            //std::cout << inputIdx << "=>" << outputIdx << "\n";

            (*outputCloud)[outputIdx].x = (*inputCloud)[inputIdx].x;
            (*outputCloud)[outputIdx].y = (*inputCloud)[inputIdx].y;
            (*outputCloud)[outputIdx].z = (*inputCloud)[inputIdx].z;
            (*outputCloud)[outputIdx].r = (*inputCloud)[inputIdx].r;
            (*outputCloud)[outputIdx].g = (*inputCloud)[inputIdx].g;
            (*outputCloud)[outputIdx].b = (*inputCloud)[inputIdx].b;
        }
    }
}

// TODO: not currently working
//void downsampleCloudCuda(
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud)
//{
//    pcl_cuda::VoxelGrid<pcl::PointXYZRGBNormal> filter;
//    filter.setInputCloud(inputCloud);
//    filter.setLeafSize(0.0381f, 0.0381f, 0.0381f);
//    filter.filter(*outputCloud);
//}

void estimateNormals(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr initialCloud,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampledCloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    //pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> normalEstimator;
    pcl::NormalEstimationOMP<pcl::PointXYZRGBNormal, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(downsampledCloud);

    //normalEstimator.setSearchSurface(initialCloud);

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    normalEstimator.setSearchMethod(tree);

    normalEstimator.setRadiusSearch(0.05);

    normalEstimator.setViewPoint(
        0,
        0,
        0.5334); // height of the bot

    std::chrono::high_resolution_clock clk;
    std::chrono::high_resolution_clock::time_point start = clk.now();
    normalEstimator.compute(*normals);
    std::chrono::high_resolution_clock::time_point end = clk.now();
    double usElapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();

    std::cout << "Compute time: " << usElapsed << std::endl;

    for (size_t i = 0; i < normals->size(); i++)
    {
        (*downsampledCloud)[i].normal_x = (*normals)[i].normal_x;
        (*downsampledCloud)[i].normal_y = (*normals)[i].normal_y;
        (*downsampledCloud)[i].normal_z = (*normals)[i].normal_z;
    }
}

void estimateNormalsIntegral(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr initialCloud,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampledCloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> normalEstimator;

    normalEstimator.setNormalEstimationMethod(normalEstimator.AVERAGE_3D_GRADIENT);
    normalEstimator.setMaxDepthChangeFactor(0.02f);
    normalEstimator.setNormalSmoothingSize(10.0f);
    normalEstimator.setInputCloud(downsampledCloud);

    normalEstimator.setViewPoint(
        0,
        0,
        0.5334); // height of the bot

    normalEstimator.compute(*normals);

    for (size_t i = 0; i < normals->size(); i++)
    {
        (*downsampledCloud)[i].normal_x = (*normals)[i].normal_x;
        (*downsampledCloud)[i].normal_y = (*normals)[i].normal_y;
        (*downsampledCloud)[i].normal_z = (*normals)[i].normal_z;
    }
}

void estimateNormalsSample(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr initialCloud,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampledCloud)
{
    pcl::SamplingSurfaceNormal<pcl::PointXYZRGBNormal> normalEstimator;
    normalEstimator.setInputCloud(initialCloud);

    normalEstimator.setRatio(0.1);
    normalEstimator.setSample(100);
    normalEstimator.filter(*downsampledCloud);
}

void estimateNormalsGpu(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampledCloud)
{
    std::chrono::high_resolution_clock clk;
    pcl::PointXYZ hostBuf[downsampledCloud->points.size()];
    for (size_t i = 0; i < downsampledCloud->points.size(); i++)
    {
        pcl::PointXYZRGBNormal p = (*downsampledCloud)[i];
        hostBuf[i].x = p.x;
        hostBuf[i].y = p.y;
        hostBuf[i].z = p.z;
    }

    pcl::gpu::DeviceArray<pcl::PointXYZ> downsampledCloudGpu;
    std::chrono::high_resolution_clock::time_point start = clk.now();
    downsampledCloudGpu.upload(&hostBuf[0], downsampledCloud->points.size());
    std::chrono::high_resolution_clock::time_point end = clk.now();
    double usElapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "Upload time: " << usElapsed << std::endl;

    pcl::gpu::Feature::Normals normals;
    pcl::gpu::NormalEstimation normalEstimator;

    normalEstimator.setInputCloud(downsampledCloudGpu);
    normalEstimator.setRadiusSearch(0.05, 100);

    normalEstimator.setViewPoint(
        0,
        0,
        0.5334); // height of the bot

    start = clk.now();
    normalEstimator.compute(normals);
    end = clk.now();
    usElapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "Compute time: " << usElapsed << std::endl;

    start = clk.now();
    normals.download(&hostBuf[0]);
    end = clk.now();
    usElapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "Download time: " << usElapsed << std::endl;

    for (size_t i = 0; i < downsampledCloud->points.size(); i++)
    {
        (*downsampledCloud)[i].normal_x = hostBuf[i].data[0];
        (*downsampledCloud)[i].normal_y = hostBuf[i].data[1];
        (*downsampledCloud)[i].normal_z = hostBuf[i].data[2];
    }
}

void colorByNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normalsCloud)
{
    for (size_t i = 0; i < normalsCloud->size(); i++)
    {
        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;

        if (std::isfinite((*normalsCloud)[i].normal_x))
        {
            float nx = (*normalsCloud)[i].normal_x;
            float ny = (*normalsCloud)[i].normal_y;
            float nz = (*normalsCloud)[i].normal_z;

            float len = sqrt((nx*nx) + (ny*ny) + (nz*nz));

            r = static_cast<uint8_t>((127.0f * nx / len) + 127.0f);
            g = static_cast<uint8_t>((127.0f * ny / len) + 127.0f);
            b = static_cast<uint8_t>((127.0f * nz / len) + 127.0f);
        }

        uint32_t rgb = (
            (static_cast<uint32_t>(r) << 16)
            |
            (static_cast<uint32_t>(g) << 8)
            |
            (static_cast<uint32_t>(b))
        );

        (*normalsCloud)[i].rgb = *reinterpret_cast<float*>(&rgb);
    }
}

void testOctreeSearch(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampledCloud)
{
    float resolution = 1;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> tree(resolution);

    tree.setInputCloud(downsampledCloud);
    std::chrono::high_resolution_clock clk;
    std::chrono::high_resolution_clock::time_point start = clk.now();
    tree.addPointsFromInputCloud();
    std::chrono::high_resolution_clock::time_point end = clk.now();
    double usElapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();

    std::cout << "Creating tree took " << usElapsed << " us." << std::endl;

	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(1.0,5.0);

    constexpr int numTrials = 20;
    std::vector<double> times;
    times.reserve(numTrials);

    float radius = 3.5 * 1.5 * 0.0254f;
    std::vector<int> pointRadiusSearch;
    std::vector<float> radiusSquaredDistance;

    for (int i = 0; i < numTrials; i++)
    {
		pcl::PointXYZRGBNormal p;
        p.x = distribution(generator);
        p.y = distribution(generator);
        p.z = distribution(generator);

        start = clk.now();
        bool found = tree.radiusSearch(p, radius, pointRadiusSearch, radiusSquaredDistance);
        end = clk.now();
        times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end-start).count());
    }

    double minTime = std::numeric_limits<double>::max();
    double maxTime = std::numeric_limits<double>::min();
    double avgTime = 0;

    for (size_t i = 0; i < times.size(); i++)
    {
        double t = times[i];

        std::cout << t << "\n";

        avgTime += t;
        minTime = std::min(minTime, t);
        maxTime = std::max(maxTime, t);
    }

    std::cout << "Min search time: " << minTime << " us." << "\n";
    std::cout << "Max search time: " << maxTime << " us." << "\n";
    std::cout << "Avg search time: " << avgTime / static_cast<double>(times.size()) << " us." << "\n";
}

void writeFile(const std::string filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    pcl::PCDWriter writer;
    writer.writeASCII(filePath, *cloud);
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./preprocessor <file>" << std::endl;
        return 0;
    }

    int device = 0;
    pcl::gpu::setDevice(device);
    pcl::gpu::printShortCudaDeviceInfo(device);

    std::string inputFilePath(argv[1]);
    std::chrono::high_resolution_clock clk;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dsColoredNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    std::cout << "Reading cloud..." << std::endl;
    readFile(inputFilePath, cloud);

    std::cout << "Echoing to file..." << std::endl;
    writeFile("echo.pcd", cloud);

    std::cout << "Performing initial normals coloring..." << std::endl;
    colorByNormals(cloud);

    std::cout << "Writing to file..." << std::endl;
    writeFile("initialNormals.pcd", cloud);

    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    double usElapsed;

    std::cout << "Downsampling cloud..." << std::endl;
    start = clk.now();
    downsampleCloud(cloud, downsampledCloud);
    //downsampleCloudOrdered(cloud, downsampledCloud);
    end = clk.now();
    usElapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "Downsampled in " << usElapsed << " microseconds." << std::endl;

    std::cout << "Writing downsampled cloud to file..." << std::endl;
    writeFile("downsampled.pcd", downsampledCloud);

    //for (int i = 0; i < 20; i++)
    //{
        std::cout << "Estimating normals..." << std::endl;
        start = clk.now();
        estimateNormalsGpu(downsampledCloud);
        //estimateNormals(cloud, downsampledCloud);
        //estimateNormalsSample(cloud, downsampledCloud);
        //estimateNormalsIntegral(cloud, downsampledCloud);
        //estimateNormalsIntegral(cloud, cloud);
        end = clk.now();
        usElapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        std::cout << "Normals estimated in " << usElapsed << " microseconds." << std::endl;
    //}
    
    std::cout << "Writing normal estimated to file..." << std::endl;
    writeFile("normals.pcd", downsampledCloud);
    //writeFile("normals.pcd", cloud);

    std::cout << "Coloring new normals..." << std::endl;
    colorByNormals(downsampledCloud);
    //colorByNormals(cloud);
    
    std::cout << "Writing to file..." << std::endl;
    writeFile("coloredFinalNormals.pcd", downsampledCloud);
    //writeFile("coloredFinalNormals.pcd", cloud);
    
    std::cout << "Testing out octree..." << std::endl;
    testOctreeSearch(downsampledCloud);
    
    std::cout << "Graceful termination." << std::endl;
}
