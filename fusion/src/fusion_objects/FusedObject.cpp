//
// Created by dieter on 25.04.19.
//

#include <fusion/fusion_objects/FusedObject.h>

#include "fusion/fusion_objects/FusedObject.h"

FusedObject::FusedObject() {
    this->cameraData = nullptr;
    this->bbox = nullptr;
    this->lidarPoints = new std::vector<MappedPoint>;
}

FusedObject::FusedObject(ObjectBoundingBox *cameraData) {
    this->cameraData = cameraData;
    this->bbox = nullptr;
    this->lidarPoints = new std::vector<MappedPoint>;
    this->setRandomColor();
}

FusedObject::FusedObject(ObjectBoundingBox *cameraData, std::vector<MappedPoint> *points) {
    this->cameraData = cameraData;
    this->bbox = nullptr;
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

    int end = this->lidarPoints->size();

    std::cout << "Removed " << start - end << " points" << std::endl;
}

void FusedObject::filterPointCloudOutsideBB() {
    int start = this->lidarPoints->size();
    for (int i = 0; i < this->lidarPoints->size(); i++) {
        if (!(this->cameraData->x - this->cameraData->w / 2 < this->lidarPoints->at(i).getPictureX() && this->lidarPoints->at(i).getPictureX() < this->cameraData->x + this->cameraData->w / 2)) {
            if (!(this->cameraData->y - this->cameraData->h / 2 < this->lidarPoints->at(i).getPictureY() && this->lidarPoints->at(i).getPictureY() < this->cameraData->y + this->cameraData->h / 2)) {
                this->lidarPoints->erase(this->lidarPoints->begin() + i);
            }
        }

    }
    int end = this->lidarPoints->size();
    std::cout << "Removed " << start - end << " points" << std::endl;
}

visualization_msgs::MarkerPtr FusedObject::calculateBoundingBox() {
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
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    visualization_msgs::MarkerPtr newBox (new visualization_msgs::Marker());
    newBox->header.frame_id = "velo_link";
    newBox->ns = "objects";
    newBox->id = rand();
    newBox->header.stamp = ros::Time::now();
    newBox->type = visualization_msgs::Marker::CUBE;
    newBox->action = visualization_msgs::Marker::ADD;
    newBox->pose.position.x = pcaCentroid[0];
    newBox->pose.position.y = pcaCentroid[1];
    newBox->pose.position.z = pcaCentroid[2];
    newBox->pose.orientation.x = bboxQuaternion.x();
    newBox->pose.orientation.y = bboxQuaternion.y();
    newBox->pose.orientation.z = bboxQuaternion.z();
    newBox->pose.orientation.w = bboxQuaternion.w();
    newBox->scale.x = maxPoint.x - minPoint.x;
    newBox->scale.y = maxPoint.y - minPoint.y;
    newBox->scale.z = maxPoint.z - minPoint.z;
    newBox->color.a = 1.0; // Don't forget to set the alpha!
    newBox->color.r = 0.0;
    newBox->color.g = 1.0;
    newBox->color.b = 0.0;
    newBox->lifetime = ros::Duration(0.1);

    this->bbox = newBox;

    return this->bbox;
}
/*
visualization_msgs::MarkerPtr FusedObject::calculateBoundingBox() {
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

    uint32_t shape = visualization_msgs::Marker::POINTS;

    visualization_msgs::MarkerPtr newBox (new visualization_msgs::Marker());
    //marker.header.frame_id = cloud->header.frame_id;
    newBox->header.frame_id = "velo_link";
    newBox->header.stamp = ros::Time::now();

    newBox->ns = "objects";
    newBox->id = rand();
    newBox->type = shape;
    newBox->action = visualization_msgs::Marker::ADD;

    newBox->pose.position.x = centroid[0];
    newBox->pose.position.y = centroid[1];
    newBox->pose.position.z = centroid[2];
    newBox->pose.orientation.x = 0.0;
    newBox->pose.orientation.y = 0.0;
    newBox->pose.orientation.z = 0.0;
    newBox->pose.orientation.w = 1.0;

    newBox->scale.x = (max[0] - min[0]);
    newBox->scale.y = (max[1] - min[1]);
    newBox->scale.z = (max[2] - min[2]);

    if (newBox->scale.x == 0)
        newBox->scale.x = 0.1;

    if (newBox->scale.y == 0)
        newBox->scale.y = 0.1;

    if (newBox->scale.z == 0)
        newBox->scale.z = 0.1;

    newBox->color.r = 1;
    newBox->color.g = 1;
    newBox->color.b = 1;
    newBox->color.a = 0.5;

    newBox->lifetime = ros::Duration(0.5);
    this->bbox = newBox;
//   marker.lifetime = ros::Duration(0.5);
    return this->bbox;
}
*/
void FusedObject::outputToLabelFile(char *fileLocation) {
    std::ofstream outfile;

    if (this->cameraData->Class != "car") {
        return;
    }

    outfile.open(fileLocation, std::ios_base::app);
    if (outfile.fail()) {
        //throw std::ios_base::failure(std::strerror(errno));
        return;
    }

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
    if(!this->bbox) {
        outfile << "-1 "; //H
        outfile << "-1 "; //W
        outfile << "-1 "; //L
        outfile << "-1000 -1000 -1000 "; //t
        outfile << "-10 "; //ry
    } else {
        outfile << std::defaultfloat << this->bbox->scale.x << " "; //H
        outfile << std::defaultfloat << this->bbox->scale.y << " "; //W
        outfile << std::defaultfloat << this->bbox->scale.z << " "; //L
        outfile << std::defaultfloat << this->bbox->pose.position.x << " "; //tx
        outfile << std::defaultfloat << this->bbox->pose.position.y << " "; //ty
        outfile << std::defaultfloat << this->bbox->pose.position.z << " "; //tz
        outfile << "-10 "; //ry
    }


    //Score
    outfile << std::defaultfloat << this->cameraData->probability << " ";

    //End line
    outfile << std::endl;

    outfile.close();
}

void FusedObject::toMsg(FusedObjectsMsgPtr msg) {
    // Camera data
    msg->cameraData.x = this->cameraData->x;
    msg->cameraData.y = this->cameraData->y;
    msg->cameraData.w = this->cameraData->w;
    msg->cameraData.h = this->cameraData->h;
    msg->cameraData.Class = this->cameraData->Class;
    msg->cameraData.probability = this->cameraData->probability;

    // Bounding box data
    msg->bbox.type = this->bbox->type;
    msg->bbox.action = this->bbox->action;
    msg->bbox.pose.position.x = this->bbox->pose.position.x;
    msg->bbox.pose.position.y = this->bbox->pose.position.y;
    msg->bbox.pose.position.z = this->bbox->pose.position.z;
    msg->bbox.pose.orientation.x = this->bbox->pose.orientation.x;
    msg->bbox.pose.orientation.y = this->bbox->pose.orientation.y;
    msg->bbox.pose.orientation.z = this->bbox->pose.orientation.z;
    msg->bbox.pose.orientation.w = this->bbox->pose.orientation.w;
    msg->bbox.scale.x = this->bbox->scale.x;
    msg->bbox.scale.y = this->bbox->scale.y;
    msg->bbox.scale.z = this->bbox->scale.z;
    msg->bbox.color.a = this->bbox->color.a;
    msg->bbox.color.r = this->bbox->color.r;
    msg->bbox.color.g = this->bbox->color.g;
    msg->bbox.color.b = this->bbox->color.b;

    // Construct vector of MappedPointMsgs
    std::vector<sensor_fusion_msg::MappedPointMsg> points;
    for (auto &point : *this->lidarPoints) {
        sensor_fusion_msg::MappedPointMsg p;
        p.cameraPlane = point.getCameraPlane();
        p.scale = point.getScale();
        p.pictureY = point.getPictureY();
        p.pictureX = point.getPictureX();
        p.screenWidth = point.getScreenWidth();
        p.screenHeight = point.getScreenHeight();
        p.copX = point.getCopX();
        p.copY = point.getCopY();
        p.copZ = point.getCopZ();
        p.distance = point.getDistance();
        p.point.x = point.getPCLPoint().x;
        p.point.y = point.getPCLPoint().y;
        p.point.z = point.getPCLPoint().z;
        points.emplace_back(p);
    }
    msg->lidarPoints = points;

    // Color
    msg->r = this->r;
    msg->g = this->g;
    msg->b = this->b;
}
