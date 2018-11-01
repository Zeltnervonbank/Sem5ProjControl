#include "camera.h"

Camera::Camera()
{

}

MarbleLocation Camera::getMarbelCenter(cv::Mat im)
{
    cv::Mat kombi;
    cv::Mat mask;
    cv::Mat cirkler;
    cv::Mat cirkler_gray;


    kombi = im.clone();
    cv::cvtColor(im,im,CV_BGR2RGB);


    cv::cvtColor(kombi, kombi, CV_BGR2HSV);
    cv::inRange(kombi,cv::Scalar(0,0,50),cv::Scalar(80,220,200),mask);

    kombi.setTo(cv::Scalar(255), mask); // Set area of mask to value 255

    for(int i=0; i<kombi.rows;i++){
         for(int j=0; j<kombi.cols;j++){
             if(kombi.at<cv::Vec3b>(i,j)[0]==0){
                 kombi.at<cv::Vec3b>(i,j)[0]=255;
                 kombi.at<cv::Vec3b>(i,j)[1]=0;
                 kombi.at<cv::Vec3b>(i,j)[2]=255;
             }
         }
     }

    cv::cvtColor(kombi, kombi, CV_HSV2RGB);
    cirkler = kombi.clone();
    std::vector<cv::Vec3f> circles;
    cv::cvtColor(cirkler, cirkler_gray, CV_RGB2GRAY);

    cv::HoughCircles(cirkler_gray,circles,CV_HOUGH_GRADIENT,1,cirkler_gray.rows/8,110,13,0,0);

    MarbleLocation mLoc;
    mLoc.radius=0;

    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        if(c[2]>=mLoc.radius){
        mLoc.center = c[0];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        cv::circle( cirkler, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline

        mLoc.radius = c[2];
        cv::circle( cirkler, center, mLoc.radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }
    }

    //std::cout << "center punkt:" << mLoc.center << std::endl;


    //std::cout << mLoc.radius << std::endl;
    cv::imshow("komb", kombi);
    cv::imshow( "Hough Circle Transform Demo", cirkler );

    return mLoc;
}
