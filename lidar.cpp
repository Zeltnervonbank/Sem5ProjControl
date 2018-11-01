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
    std::vector<cv::Point2f> detectedPoints = GetLidarPoints(msg);

    // Display lines on image
    im = DisplayLidarPoints(im, detectedPoints);



    std::vector<ScanSegment> segments = GetSegmentsOfScan(detectedPoints);


    //im = DisplayScanSegments(im, segments);
    // Create new empty image and copy a grayscale version of original image into it
    cv::Mat im_gray;
    cv::cvtColor(im, im_gray, CV_RGB2GRAY);

    // Use Hough transform to find lines in image
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(im_gray, lines, 1, CV_PI/180, 80, 10, 10);

    // Display the found lines on the image
    im = DisplayLines(im, lines);

    // Add a little blur
    GaussianBlur( im_gray, im_gray, cv::Size(9, 9), 2, 2 );

    // Use Hough transform to find circles in the image
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(im_gray, circles, cv::HOUGH_GRADIENT, 1, im_gray.rows/8, 30, 8, 0, 8);
    std::cout << "There are: " << circles.size() << " circles." << std::endl;

    im = DisplayCircles(im, circles);

    // Add robot location and time overlay
    cv::circle(im, cv::Point(300, 300), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));

    // Display images
    mutex.lock();
    cv::imshow("lidar", im);
    //cv::imshow("circles", circleMat);
    cv::imshow("Gray", im_gray);
    mutex.unlock();
}

std::vector<cv::Point2f> lidar::GetLidarPoints(ConstLaserScanStampedPtr &msg)
{
    float angle_min = float(msg->scan().angle_min());
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    float px_per_m = 200 / range_max;

    std::vector<cv::Point2f> detectedPoints;

    for (size_t i = 0; i < msg->scan().ranges_size(); i++)
    {
      float angle = angle_min + i * angle_increment;
      float range = std::min(float(msg->scan().ranges(i)), range_max);
      if(range == range_max)
      {
          //continue;
      }
      // Get start point
      cv::Point2f startpt(300.5f + range_min * px_per_m * std::cos(angle),
                          300.5f - range_min * px_per_m * std::sin(angle));

      // Get end point
      cv::Point2f endpt(300.5f + range * px_per_m * std::cos(angle),
                        300.5f - range * px_per_m * std::sin(angle));

      detectedPoints.push_back(endpt);
    }

    return detectedPoints;
}

cv::Mat lidar::DisplayLidarPoints(cv::Mat im, std::vector<cv::Point2f> points)
{
    for(size_t i = 0; i < points.size() - 1; i++)
    {
        cv::line(im, points[i] * 16, points[i + 1] * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);
    }

    return im;
}

cv::Mat lidar::DisplayScanSegments(cv::Mat im, std::vector<lidar::ScanSegment> segments)
{
    for( size_t i = 0; i < segments.size(); i++)
    {
        ScanSegment segment = segments[i];
        for(uint j = 0; j < segment.points.size() - 2; j++)
        {
            if(segment.type == STRAIGHT)
            {
                cv::line(im, segment.points[j] * 16, segment.points[j + 1] * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);
            }
            else if(segment.type == CURVED)
            {
                cv::line(im, segment.points[j] * 16, segment.points[j + 1] * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);
            }
        }
    }

    return im;
}

cv::Mat lidar::DisplayCircles(cv::Mat im, std::vector<cv::Vec3f> circles)
{
    for( size_t i = 0; i < circles.size(); i++ )
    {
       cv::Point2f center(circles[i][0], circles[i][1]);
       float radius = circles[i][2];
       // circle outline
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

std::vector<lidar::ScanSegment> lidar::GetSegmentsOfScan(std::vector<cv::Point2f> points)
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
        cv::Point2f currentPoints [3] = {points[i], points[i+1], points[i+2]};

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
