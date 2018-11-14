#include "lidar.h"

#define IM_WIDTH 600
#define IM_HEIGHT 600

static boost::mutex mutex;

lidar::lidar()
{

}

void lidar::lidarCallback(ConstLaserScanStampedPtr &msg)
{
    // Get timing
    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    // Create empty image
    cv::Mat im(IM_HEIGHT, IM_WIDTH, CV_8UC3);
    im.setTo(0);

    // Get points from lidar sweep
    std::vector<LidarRay> detectedPoints = GetLidarPoints(msg);

    // Display lines on image
    im = DisplayLidarPoints(im, detectedPoints, false);

    // Create new empty image and copy a grayscale version of original image into it
    cv::Mat im_gray;
    cv::cvtColor(im, im_gray, CV_RGB2GRAY);

    // Use Hough transform to find lines in image
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(im_gray, lines, 1, CV_PI/180, 60, 5, 10);

    // Create new image for cv data
    cv::Mat cvMat(IM_HEIGHT, IM_WIDTH, CV_8UC3);
    cvMat.setTo(0);

    // Display the found lines on the image
    cvMat = DisplayLines(cvMat, lines);

    // Add a little blur
    GaussianBlur( im_gray, im_gray, cv::Size(9, 9), 2, 2 );

    // Use Hough transform to find circles in the image
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(im_gray, circles, cv::HOUGH_GRADIENT, 1, im_gray.rows/8, 85, 11, 10.5, 10.6);
    //std::cout << "There are: " << circles.size() << " circles." << std::endl;

    ConvertCirclesToLidarMarbles(circles);
    cvMat = DisplayCircles(cvMat, circles);

    // Add robot location and time overlay
    cv::circle(im, cv::Point(300, 300), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));

    // Display images
    mutex.lock();
    //cv::imshow("Lidar", im);
    //cv::imshow("Gray", im_gray);
    cv::imshow("CV", cvMat);
    mutex.unlock();
}

std::vector<LidarRay> lidar::GetLidarPoints(ConstLaserScanStampedPtr &msg)
{
    // Get some basic information
    float angleMin = float(msg->scan().angle_min());
    float angleIncrement = float(msg->scan().angle_step());

    float rangeMin = float(msg->scan().range_min());
    float rangeMax = float(msg->scan().range_max());

    float pxPerM = 200 / rangeMax;

    // Declare empty return vector
    std::vector<LidarRay> detectedPoints;

    // Reset the nearest point to a high value
    nearestPoint.range = 100.0f;

    // Clear last sweep
    lidarRays.clear();

    // Loop through each scan ray
    for (size_t i = 0; i < msg->scan().ranges_size(); i++)
    {
        // Get angle and distance of the ray
        float angle = angleMin + i * angleIncrement;
        float range = std::min(float(msg->scan().ranges(i)), rangeMax); // Gets the lower of the scan ray and max range
        // Get start point
        cv::Point2f startpt(300.5f + rangeMin * pxPerM * std::cos(angle), 300.5f - rangeMin * pxPerM * std::sin(angle));

        // Get end point
        cv::Point2f endpt(300.5f + range * pxPerM * std::cos(angle), 300.5f - range * pxPerM * std::sin(angle));

        // Creates new instance of ray struct
        LidarRay ray =
        {
            startpt,
            endpt,
            range,
            range == rangeMax,
            angle
        };

        // Adds that ray to return vector
        detectedPoints.push_back(ray);
        lidarRays.push_back(ray);

        // If this ray is shorter than the previous nearest point, replace it
        if(ray.range < nearestPoint.range)
        {
            nearestPoint = ray;
        }
    }

    return detectedPoints;
}

cv::Mat lidar::DisplayLidarPoints(cv::Mat im, std::vector<LidarRay> points, bool displayMaxRange)
{
    for(size_t i = 0; i < points.size() - 1; i++)
    {
        // If this point is max range and the next isn't, OR if this point is not max range and the next is
        if(((!points[i].isMaxRange && points[i + 1].isMaxRange) || (points[i].isMaxRange && !points[i + 1].isMaxRange)) && displayMaxRange)
        {
            cv::line(im, points[i].endPoint * 16, points[i + 1].endPoint * 16, cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA, 4);
        }

        // If BOTH this point AND the next are max range
        else if(points[i].isMaxRange && points[i + 1].isMaxRange && displayMaxRange)
        {
            cv::line(im, points[i].endPoint * 16, points[i + 1].endPoint * 16, cv::Scalar(0, 255, 0, 255), 1, cv::LINE_AA, 4);
        }

        // If neither this point or the next are at max range
        else if(!points[i].isMaxRange && !points[i + 1].isMaxRange && abs(points[i].range - points[i + 1].range) < 1.0)
        {
            cv::line(im, points[i].endPoint * 16, points[i + 1].endPoint * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);
        }

        // Display entire ray
        //cv::line(im, points[i].startPoint * 16, points[i].endPoint * 16, cv::Scalar(255, 0, 0, 255), 1, cv::LINE_AA, 4);
    }

    return im;
}

/// Iffy method that I might remove at some point
cv::Mat lidar::DisplayScanSegments(cv::Mat im, std::vector<lidar::ScanSegment> segments)
{
    for( size_t i = 0; i < segments.size(); i++)
    {
        ScanSegment segment = segments[i];
        for(uint j = 0; j < segment.points.size() - 2; j++)
        {
            if(segment.type == STRAIGHT)
            {
                cv::line(im, segment.points[j].endPoint * 16, segment.points[j + 1].endPoint * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);
            }
            else if(segment.type == CURVED)
            {
                cv::line(im, segment.points[j].endPoint * 16, segment.points[j + 1].endPoint * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);
            }
        }
    }

    return im;
}

std::vector<LidarMarble> lidar::ConvertCirclesToLidarMarbles(std::vector<cv::Vec3f> circles)
{
    // Declare return vector
    std::vector<LidarMarble> marbles;

    // Determine whether marbles have been detected
    lidar::marblesPresent = circles.size() != 0;
    if(circles.size() == 0)
    {
        return marbles;
    }

    // Clear static marble vector - might be a good idea to put a mutex around this. TODO
    detectedMarbles.clear();

    // Sets nearestMarble to high value
    nearestMarble.distance = 1000.0f;

    // Loop through detected circles
    for(size_t i = 0; i < circles.size(); i++)
    {
        // Get relevant points
        cv::Point center(circles[i][0], circles[i][1]);
        cv::Point robPos(300, 300); // Manually set based on image size
        cv::Point diffPoint = center - robPos;

        // Get euclidean distance from center to the center of the circle
        float distance = sqrt(pow(diffPoint.x, 2.0) + pow(diffPoint.y, 2.0));

        // Get the angle between the robot's forward axis and the center of the circle,
        float angle = atan2(diffPoint.x, diffPoint.y) - 90.0f * M_PI/180.0f;

        // Print distance and angle
        //std::cout << "Distance to Marble: " << i << " - " << distance << std::endl;
        //std::cout << "Angle to Marble: " << i << " - " << angle << std::endl;

        // Create marble object from data
        LidarMarble marble = {distance, angle, circles[i][2]};

        // Replaces nearestMarble if this marble is closer
        if(marble.distance < nearestMarble.distance)
        {
            nearestMarble = marble;
        }

        // Add that object to return vector and static vector
        marbles.push_back(marble);
        detectedMarbles.push_back(marble);
    }

    return marbles;
}

cv::Mat lidar::DisplayCircles(cv::Mat im, std::vector<cv::Vec3f> circles)
{
    for( size_t i = 0; i < circles.size(); i++ )
    {
       cv::Point2f center(circles[i][0], circles[i][1]);
       float radius = circles[i][2];
       //std::cout << "radius" << radius << std::endl;

       // Circle outline
       cv::circle( im, center, radius, cv::Scalar(255, 0, 0), 2, 8, 0 );
    }

    return im;
}

cv::Mat lidar::DisplayLines(cv::Mat im, std::vector<cv::Vec4i> lines)
{
    for( size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::line( im, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 1, CV_AA);
    }

    return im;
}

std::vector<lidar::ScanSegment> lidar::GetSegmentsOfScan(std::vector<LidarRay> points)
{
    std::vector<ScanSegment> segments;
    ScanSegment currentSegment;
    int segmentType = 0;

    float threshold = 0.001f;

    // Loop through points
    for(uint i = 0; i < points.size(); i++)
    {
        // Break out before out of bounds exception
        if(i == points.size() - 1)
        {
            break;
        }

        // Get three points needed for collinearity calculation
        cv::Point2f currentPoints [3] = {points[i].endPoint, points[i+1].endPoint, points[i+2].endPoint};

        // Determine collinearity of points
        float collinearity = lidar::GetCollinearity(currentPoints);

        // Add current point to vector
        currentSegment.points.push_back(points[i]);

        // If these three points are collinear we're no longer in a curve, and should finish the segment
        if(collinearity < threshold && segmentType == CURVED)
        {
            // Add the second point - the last non collinear point
            currentSegment.points.push_back(points[i + 1]);
            // Set the segment type
            currentSegment.type = CURVED;
            // Add the segment to the output
            segments.push_back(currentSegment);

            // Set the next segment type
            segmentType = STRAIGHT;

            // Clear the current segment
            currentSegment.points.clear();
        }

        // If the points are not collinear, we're no longer on a straight section, and should finish the segment
        else if(collinearity > threshold && segmentType == STRAIGHT)
        {
            currentSegment.points.push_back(points[i + 1]);
            currentSegment.type = STRAIGHT;
            segments.push_back(currentSegment);

            segmentType = CURVED;

            currentSegment.points.clear();
        }
    }

    return segments;
}

float lidar::GetCollinearity(cv::Point2f points[3])
{
    cv::Point2f A = points[0];
    cv::Point2f B = points[1];
    cv::Point2f C = points[2];

    float area = A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y);

    return area;
}
