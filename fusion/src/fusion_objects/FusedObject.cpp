//
// Created by dieter on 25.04.19.
//

#include <fusion/fusion_objects/FusedObject.h>

#include "fusion/fusion_objects/FusedObject.h"

FusedObject::FusedObject() {
    this->cameraData = nullptr;
    this->lidarPoints = new std::vector<MappedPoint>;
}

FusedObject::FusedObject(ObjectBoundingBox *cameraData) {
    this->cameraData = cameraData;
    this->lidarPoints = new std::vector<MappedPoint>;
    this->setRandomColor();
}

FusedObject::FusedObject(ObjectBoundingBox *cameraData, std::vector<MappedPoint> *points) {
    this->cameraData = cameraData;
    this->lidarPoints = points;
}

void FusedObject::setColor(int r, int g, int b) {
    this->r = r;
    this->g = g;
    this->b = b;
}

void FusedObject::setRandomColor() {
    this->r = std::rand() % 255;;
    this->g = std::rand() % 255;;
    this->b = std::rand() % 255;;
}

void FusedObject::addPoint(const MappedPoint &point) {
    this->lidarPoints->emplace_back(point);
}

void FusedObject::drawObject(const cv_bridge::CvImagePtr &imagePtr) {
    if (this->cameraData != nullptr) {
        //Draw the bounding box first
        cv::Point pt1((int) (cameraData->x - cameraData->w / 2), (int) (cameraData->y - cameraData->h / 2));
        cv::Point pt2((int) (cameraData->x + cameraData->w / 2), (int) (cameraData->y + cameraData->h / 2));
        cv::rectangle(imagePtr->image, pt1, pt2, cv::Scalar(b, g, r));
    }

    //Draw the lidar points
    for (auto it = lidarPoints->begin(); it != lidarPoints->end(); it++) {
        int thickness = 3 - (int) it->map(it->getDistance(), 0, 100, 1, 2);
        cv::circle(imagePtr->image, cv::Point((int) it->getPictureX(), (int) it->getPictureY()),
                   thickness,
                   cv::Scalar(b, g, r), cv::FILLED, cv::LINE_8);
    }
}

void FusedObject::filterBiggestCluster(float tolerance) {

    if (this->lidarPoints->empty()) return;

    int start = this->lidarPoints->size();

    // Construct pointcloud from fused object points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &point : *this->lidarPoints) {
        pcl::PointXYZ p = point.getPCLPoint();
        cloud->push_back(p);
    }

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // create the extraction object for the clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // specify euclidean cluster parameters
    ec.setClusterTolerance(tolerance); //
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(40);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    // extract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) return;

    /*int j = 0;
    int biggest_clustersize = 0;
    int biggestClusterIndex = 0;
    for(pcl::PointIndices &indices : cluster_indices) {
        std::cout << "Cluster size: " << indices.indices.size() << std::endl;
        if (indices.indices.size() > biggest_clustersize) {
            biggest_clustersize = (int) indices.indices.size();
            biggestClusterIndex = j;
        }
        j++;
        std::cout << "Biggest cluster found, cluster index: " << biggestClusterIndex << std::endl;
    }*/

    for (int i = 0; i < this->lidarPoints->size(); i++) {
        if (!(std::find(cluster_indices[0].indices.begin(), cluster_indices[0].indices.end(), i) !=
              cluster_indices[0].indices.end())) {
            this->lidarPoints->erase(this->lidarPoints->begin() + i);
        }
    }

    int end = this->lidarPoints->empty();

    std::cout << "Removed " << start - end << " points" << std::endl;
}

/*
visualization_msgs::Marker FusedObject::calculateBoundingBox() {
    // Construct pointcloud from fused object points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &point : *this->lidarPoints) {
        pcl::PointXYZ p = point.getPCLPoint();
        cloud->push_back(p);
    }

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = bboxTransform[0];
    marker.pose.position.y = bboxTransform[1];
    marker.pose.position.z = bboxTransform[2];
    marker.pose.orientation.x = bboxQuaternion.x();
    marker.pose.orientation.y = bboxQuaternion.y();
    marker.pose.orientation.z = bboxQuaternion.z();
    marker.pose.orientation.w = bboxQuaternion.w();
    marker.scale.x = maxPoint.x - minPoint.x;
    marker.scale.y = maxPoint.y - minPoint.y;
    marker.scale.z = maxPoint.z - minPoint.z;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    return marker;
}
*/
visualization_msgs::Marker FusedObject::calculateBoundingBox() {
    // Construct pointcloud from fused object points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &point : *this->lidarPoints) {
        pcl::PointXYZ p = point.getPCLPoint();
        cloud->push_back(p);
    }

    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;

    pcl::compute3DCentroid(*cloud, centroid);
    pcl::getMinMax3D(*cloud, min, max);

    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    //marker.header.frame_id = cloud->header.frame_id;
    marker.header.frame_id = "velo_link";
    marker.header.stamp = ros::Time::now();

    //marker.ns = ns;
    //marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = (max[0] - min[0]);
    marker.scale.y = (max[1] - min[1]);
    marker.scale.z = (max[2] - min[2]);

    if (marker.scale.x == 0)
        marker.scale.x = 0.1;

    if (marker.scale.y == 0)
        marker.scale.y = 0.1;

    if (marker.scale.z == 0)
        marker.scale.z = 0.1;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();
//   marker.lifetime = ros::Duration(0.5);
    return marker;
}

void FusedObject::outputToLabelFile(char *fileLocation) {
    std::ofstream outfile;

    if (this->cameraData->Class != "car") {
        return;
    }

    outfile.open(fileLocation, std::ios_base::app);
    if (outfile.fail())
        throw std::ios_base::failure(std::strerror(errno));

    // First the label
    outfile << "Car ";

    //Truncation
    outfile << "-1 ";

    //Occlusion
    outfile << "-1 ";

    //Alpha
    outfile << "-10 ";

    //2D boundingbox
    outfile << std::defaultfloat << (cameraData->x - cameraData->w / 2) << " "; //X1
    outfile << std::defaultfloat << (cameraData->y - cameraData->h / 2) << " "; //Y1
    outfile << std::defaultfloat << (cameraData->x + cameraData->w / 2) << " "; //X2
    outfile << std::defaultfloat << (cameraData->y + cameraData->h / 2) << " "; //Y2

    //3D boundingbox
    outfile << "-1 "; //H
    outfile << "-1 "; //W
    outfile << "-1 "; //L
    outfile << "-1000 -1000 -1000 "; //t
    outfile << "-10 "; //ry

    //Score
    outfile << std::defaultfloat << this->cameraData->probability << " ";

    //End line
    outfile << std::endl;

    outfile.close();
}
