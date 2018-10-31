#include "lidar.h"

static boost::mutex mutex;

lidar::lidar()
{

}

void lidar::lidarCallback(ConstLaserScanStampedPtr &msg)
{

  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 600;
  int height = 600;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);

  std::vector<cv::Point2f> detectedPoints;



  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);

    // Get start point
    cv::Point2f startpt(300.5f + range_min * px_per_m * std::cos(angle),
                        300.5f - range_min * px_per_m * std::sin(angle));

    // Get end point
    cv::Point2f endpt(300.5f + range * px_per_m * std::cos(angle),
                      300.5f - range * px_per_m * std::sin(angle));

    detectedPoints.push_back(endpt);

    // Create line from start to end
    //cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);

    //    std::cout << angle << " " << range << " " << intensity << std::endl;
  }

  std::vector<ScanSegment> segments = GetSegmentsOfScan(detectedPoints);

  for(uint i = 0; i < segments.size(); i++)
  {
      ScanSegment segment = segments[i];
      for(uint j = 0; j < segment.points.size() - 2; j++)
      {
          if(segment.type == STRAIGHT)
          {
              //cv::circle(im, segment.points[j], 1, cv::Scalar(0, 0, 255));
              cv::line(im, segment.points[j] * 16, segment.points[j + 1] * 16, cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA, 4);
          }
          else if(segment.type == CURVED)
          {
              //cv::circle(im, segment.points[j], 1, cv::Scalar(0, 255, 0));
              cv::line(im, segment.points[j] * 16, segment.points[j + 1] * 16, cv::Scalar(0, 255, 255, 255), 1, cv::LINE_AA, 4);
          }
      }
  }

  cv::Mat im_gray;
  cv::cvtColor(im, im_gray, CV_RGB2GRAY);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(im_gray, lines, 1, CV_PI/180, 80, 10, 10);

  GaussianBlur( im_gray, im_gray, cv::Size(9, 9), 2, 2 );

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(im_gray, circles, cv::HOUGH_GRADIENT, 1, im_gray.rows/8, 30, 8, 0, 0);



  cv::Mat circleMat(height, width, CV_8UC3);
  circleMat.setTo(0);
  std::cout << "There are: " << circles.size() << " circles." << std::endl;

  for( size_t i = 0; i < circles.size(); i++ )
  {
     cv::Vec3i c = circles[i];
     cv::Point2f center(circles[i][0], circles[i][1]);
     float radius = circles[i][2];
     // circle outline
     cv::circle( im, center, radius, cv::Scalar(255, 0, 0), 2, 8, 0 );
  }

  for( size_t i = 0; i < lines.size(); i++)
  {
      cv::Vec4i l = lines[i];
      cv::line( im, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 1, CV_AA);
  }



  cv::circle(im, cv::Point(300, 300), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  //cv::imshow("circles", circleMat);
  mutex.unlock();
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
