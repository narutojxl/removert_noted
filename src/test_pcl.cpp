

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 


#include <iostream>


int main(int argc, char* argv[]){

    using PointType = pcl::PointXYZI;
    pcl::PointCloud<PointType>::Ptr cloud_in;
    pcl::PointCloud<PointType>::Ptr cloud_out;

    cloud_in.reset(new pcl::PointCloud<PointType>());
    cloud_out.reset(new pcl::PointCloud<PointType>());

    cloud_in->clear();
    cloud_out->clear();

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 3) = 1;
    transform(1, 3) = 1;
    transform(2, 3) = 1;

    const std::size_t num = 100;
    PointType  tmp, out ;
    for (std::size_t i = 0; i < num; ++i){
        tmp.x = i*1.0;
        tmp.y = i*1.0;
        tmp.z = i*1.0;
        tmp.intensity = 0;
        cloud_in->points.emplace_back(tmp);
    }
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform);
    //test whether the output and input point cloud index not change
    //input cloud and output cloud's index do not change

     for(std::size_t i = 0; i < cloud_out->points.size(); i++){
         std::printf("i = %d, point_in = (%f, %f, %f), point_out = (%f, %f, %f)\n", i, 
                     cloud_in->points[i].x,  cloud_in->points[i].y,  cloud_in->points[i].z,
                     cloud_out->points[i].x,  cloud_out->points[i].y,  cloud_out->points[i].z);
     }

    std::cout<<std::endl;
    
    Eigen::Affine3f T;
    T.matrix() = transform.inverse().cast<float>(); 
    for(std::size_t i = 0; i < cloud_out->points.size(); i++){
        tmp.x = cloud_out->points[i].x;
        tmp.y = cloud_out->points[i].y;
        tmp.z = cloud_out->points[i].z;
        tmp.intensity = 0;
        out = pcl::transformPoint(tmp, T);
        std::printf("i = %d, point_in = (%f, %f, %f), point_out = (%f, %f, %f)\n", i, tmp.x, tmp.y, tmp.z, out.x, out.y, out.z);
    }

    return 0;
}