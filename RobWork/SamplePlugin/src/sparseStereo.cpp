#include "sparseStereo.hpp"

sparseStereo::sparseStereo( int hough_dp, int hough_upperThres,
                int hough_centerThres, int hough_minRadius,
                int hough_maxRadius)
    :   mHough_dp{hough_dp},
        mHoughUpperThres{hough_upperThres},
        mHoughCenterThres{hough_centerThres},
        mHoughMinRadius{hough_minRadius},
        mHoughMaxRadius{hough_maxRadius}
{}

void sparseStereo::printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc, cv::Mat &proj_mat, cv::Mat &cam_mat) {
    rw::kinematics::State state = wc->getDefaultState();
    rw::kinematics::Frame* cameraFrame = wc->findFrame(frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );

            Eigen::Matrix<double, 3, 4> KA;
            KA << fovy_pixel, 0, width / 2.0, 0,
                  0, fovy_pixel, height / 2.0, 0,
                  0, 0, 1, 0;

            // OPENCV //
            cv::Mat KA_opencv = (cv::Mat_<double>(3, 4) << fovy_pixel, 0, width/2.0, 0,
                                                           0, fovy_pixel, height / 2.0, 0,
                                                           0, 0, 1, 0);

            cam_mat = KA_opencv.colRange(0, 3);

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            rw::math::Transform3D<> camPosOGL = cameraFrame->wTf(state); // Transform world to camera
            rw::math::Transform3D<> openGLToVis = rw::math::Transform3D<>(rw::math::RPY<>(-rw::math::Pi, 0, rw::math::Pi).toRotation3D()); // Rotate camera to point towards the table
            rw::math::Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e() << std::endl;

            cv::Mat H_opencv = (cv::Mat_<double>(4, 4) << H.R().getRow(0)[0], H.R().getRow(0)[1], H.R().getRow(0)[2], H.P()[0],
                                                          H.R().getRow(1)[0], H.R().getRow(1)[1], H.R().getRow(1)[2], H.P()[1],
                                                          H.R().getRow(2)[0], H.R().getRow(2)[1], H.R().getRow(2)[2], H.P()[2],
                                                                   0        ,           0       ,          0        ,     1   );
            // Calculates projection matrix opencv
            proj_mat = KA_opencv * H_opencv;
        }
    }
}


cv::Mat sparseStereo::addGaussianNoise(double mean, double stdDev, cv::Mat img){
    cv::Mat result = img.clone();
    cv::Mat GausNoise(img.size(), CV_32SC3);
    cv::randn(GausNoise, mean, stdDev);
    result += GausNoise;
    normalize(result, result, 0 ,255, cv::NORM_MINMAX, CV_8UC3);

    for(unsigned i = 0; i < img.rows; i++){
        for(unsigned j = 0; j < img.cols; j++){
            if ( result.at<cv::Vec3b>(i,j)[0] + GausNoise.at<cv::Vec3i>(i, j)[0] > 255 )
                result.at<cv::Vec3b>(i,j)[0] = 255;
            else if ( result.at<cv::Vec3b>(i,j)[0] + GausNoise.at<cv::Vec3i>(i, j)[0] < 0 )
                result.at<cv::Vec3b>(i,j)[0] = 0;
            else
                result.at<cv::Vec3b>(i,j)[0] += GausNoise.at<cv::Vec3i>(i, j)[0];

            if ( result.at<cv::Vec3b>(i,j)[1] + GausNoise.at<cv::Vec3i>(i, j)[1] > 255 )
                result.at<cv::Vec3b>(i,j)[1] = 255;
            else if ( result.at<cv::Vec3b>(i,j)[1] + GausNoise.at<cv::Vec3i>(i, j)[1] < 0 )
                result.at<cv::Vec3b>(i,j)[1] = 0;
            else
               result.at<cv::Vec3b>(i,j)[1] + GausNoise.at<cv::Vec3i>(i, j)[1];

            if ( result.at<cv::Vec3b>(i,j)[2] + GausNoise.at<cv::Vec3i>(i, j)[2] > 255 )
                result.at<cv::Vec3b>(i,j)[2] = 255;
            else if ( result.at<cv::Vec3b>(i,j)[2] + GausNoise.at<cv::Vec3i>(i, j)[2] < 0 )
                result.at<cv::Vec3b>(i,j)[2] = 0;
            else
                result.at<cv::Vec3b>(i,j)[2] + GausNoise.at<cv::Vec3i>(i, j)[2];
        }
    }

    return result;
}

void sparseStereo::evaluatePerformance(float mean, std::vector<float> stdDev, int numberOfImages, cv::Mat projLeftMat, cv::Mat projRightMat){
    const std::string imagePath =  "../../data/VisionPerformanceimages/";
    
    // Loading images
    std::vector<cv::Mat> testLeftImg;
    std::vector<cv::Mat> testRightImg;

    for(unsigned int i = 1; i < numberOfImages+1; i++){
        testLeftImg.push_back(cv::imread(imagePath + "Camera_Left" + std::to_string(i) + ".png"));
        testRightImg.push_back(cv::imread(imagePath + "Camera_Right" + std::to_string(i) + ".png"));
    }

    // Evaluating performance
    cv::Mat left_eval_pic;
    cv::Mat right_eval_pic;
    cv::Mat ball_pose;
    float cur_std_dev;

    std::ofstream myFile;
    std::ofstream myFileTime;
    myFileTime.open("../performaceTime");
    myFile.open("../performanceEstimation");

    cv::Mat TF_TABLE = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 0.1,
                                                  0, 0, 0, 1);

    for(unsigned int i = 0; i < stdDev.size(); i++){
        cur_std_dev = stdDev[i];
        for(unsigned int j = 0; j < numberOfImages; j++){
            if(j % 5 == 0){
                std::cout << "Testing standard deviation " << cur_std_dev << " Picture " << j << " out of " << numberOfImages << std::endl;
            }
            left_eval_pic = addGaussianNoise(0, cur_std_dev, testLeftImg[j]);
            right_eval_pic = addGaussianNoise(0, cur_std_dev, testRightImg[j]);

            //Logging time

            auto start = std::chrono::high_resolution_clock::now();
            std::pair<cv::Mat, cv::Mat> ball_pose = findPose(left_eval_pic, right_eval_pic, projLeftMat, projRightMat);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);

            ball_pose.first = TF_TABLE * ball_pose.first;
            myFile << cur_std_dev << " " << ball_pose.first.at<double>(0, 0) << " " << ball_pose.first.at<double>(0, 1) << " " << ball_pose.first.at<double>(0, 2) << std::endl;
            myFileTime << duration.count() << std::endl;
        }
    }
    myFileTime.close();
    myFile.close();
}

std::pair<cv::Point2d, cv::Mat> sparseStereo::locateBall(cv::Mat undistortedImage){
    std::vector<cv::Vec3f> circles;
    //cv::Scalar lowerHSVBoundary = {45, 0, 0}, upperHSVBoundary = {65, 255, 255};
    cv::Scalar lowerHSVBoundary = {30, 30, 10}, upperHSVBoundary = {90, 255, 255};

	// Blur the image
	cv::Mat image_kernel;
	//cv::blur(undistortedImage, image_kernel, cv::Size(3, 3));

	// Convert to HSV colorspace
	cv::Mat image_hsv;
	cv::cvtColor(undistortedImage, image_hsv, cv::COLOR_BGR2HSV);

	// Threshold the HSV image, keep only the green pixels
	//cv::Mat lower_green_hue_range, green_hue_image;
	//cv::Mat upper_green_hue_range;
	//cv::inRange(image_hsv, lowerHSVBoundary, upperHSVBoundary, image_hsv);

    // cv::erode(image_hsv, image_hsv, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);
    // cv::dilate(image_hsv, image_hsv, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);

    cv::erode(image_hsv, image_hsv, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(image_hsv, image_hsv, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    //morphological closing (fill small holes in the foreground)
    cv::dilate(image_hsv, image_hsv, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(image_hsv, image_hsv, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	// // Threshold the HSV image, keep only the green pixels
	 cv::Mat lower_green_hue_range, green_hue_image;
	//cv::Mat upper_green_hue_range;
	cv::inRange(image_hsv, lowerHSVBoundary, upperHSVBoundary, green_hue_image);
	//cv::inRange(image_hsv, cv::Scalar(30, 86, 6), cv::Scalar(65, 255, 255), upper_green_hue_range);

	// Combine the above two images
	//cv::Mat green_hue_image;
	//cv::addWeighted(lower_green_hue_range, 1.0, upper_green_hue_range, 1.0, 0.0, green_hue_image);


    // cv::imshow("green_hue_image", green_hue_image);
    // cv::waitKey(0);

    cv::HoughCircles(green_hue_image, circles, cv::HOUGH_GRADIENT, mHough_dp, green_hue_image.rows, mHoughUpperThres, mHoughCenterThres, mHoughMinRadius, mHoughMaxRadius);

    cv::Point2d objectCenter;

	if (circles.size() == 0) std::cout << "No circles were detected." << std::endl;
	else{
		for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle){
			objectCenter.x = round(circles[current_circle][0]);
            objectCenter.y = round(circles[current_circle][1]);
			int radius = round(circles[current_circle][2]);

			cv::circle(undistortedImage, objectCenter, radius, cv::Scalar(0, 0, 255), 2);
		}
	}

    // cv::imshow("undistortedImage", undistortedImage);
    // cv::waitKey(0);

    return std::make_pair(objectCenter, undistortedImage);
}

std::pair<cv::Mat, cv::Mat> sparseStereo::findPose(cv::Mat left_img, cv::Mat right_img, cv::Mat projLeftMat, cv::Mat projRightMat){
    cv::Mat triangulatePoint(1, 1, CV_64FC4);
    cv::Mat leftPoint(1, 1, CV_64FC2);
    cv::Mat rightPoint(1, 1, CV_64FC2);
    cv::Mat result;

    std::pair<cv::Point2d,cv::Mat> locateCircleLeft = locateBall(left_img);
    std::pair<cv::Point2d,cv::Mat> locateCircleRight = locateBall(right_img);
    

    leftPoint.at<cv::Vec2d>(0) = locateCircleLeft.first;
    rightPoint.at<cv::Vec2d>(0) = locateCircleRight.first;

    cv::triangulatePoints(projLeftMat, projRightMat, leftPoint, rightPoint, triangulatePoint);
    triangulatePoint =  triangulatePoint/triangulatePoint.at<double>(0, 3);

    cv::Mat transformationTable = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 0.1,
                                                  0, 0, 0, 1);

    return std::make_pair((transformationTable*triangulatePoint), locateCircleLeft.second);
}


int sparseStereo::getHough_dp() const{
    return mHough_dp;
}

int sparseStereo::getHoughUpperThres() const{
    return mHoughUpperThres;
}

int sparseStereo::getHoughCenterThres() const{
    return mHoughCenterThres;
}

int sparseStereo::getHoughMinRadius() const{
    return mHoughMinRadius;
}

int sparseStereo::getHoughMaxRadius() const{
    return mHoughMaxRadius;
}


cv::Mat sparseStereo::colorFiltering(const cv::Mat &input) {
    cv::Mat result, img = input.clone(), hsv, mask;
    //Create trackbars in "Control" window
    cv::namedWindow("Control", cv::WINDOW_AUTOSIZE); //create a window called "Control"
    int lowH = 0, highH = 179, lowS = 0, highS = 255, lowV = 0, highV = 255;
    cv::createTrackbar("LowH", "Control", &lowH, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", "Control", &highH, 179);
    cv::createTrackbar("LowS", "Control", &lowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &highS, 255);
    cv::createTrackbar("LowV", "Control", &lowV, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &highV, 255);
    while (true) {
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        cv::inRange(hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask); //Threshold the image
        //morphological opening (remove small objects from the foreground)
        cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        //morphological closing (fill small holes in the foreground)
        cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::bitwise_and(img, img, result, mask);
        cv::imshow("Thresholded Image", mask); //show the thresholded image
       // cv::imshow("Original", img); //show the original image
        //cv::imshow("Output", result);
    //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    if (cv::waitKey(0) == 27) { break; }
    }
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask);

    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 30, 0), cv::Scalar(20, 255, 255), mask);
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::bitwise_and(img, img, result, mask);

    return result;
}