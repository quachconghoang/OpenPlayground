#include <cstdio>
#include <cstring>
#include <vector>
#include <opencv2/opencv.hpp>

cv::Point templateMatching(cv::Mat &imgBin, const cv::Mat &tmp) {
    int match_method = CV_TM_CCOEFF_NORMED;
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::Mat result;
    int result_cols =  imgBin.cols - tmp.cols + 1;
    int result_rows = imgBin.rows - tmp.rows + 1;
    result.create( imgBin.rows, imgBin.cols, CV_32FC1 );
    cv::matchTemplate( imgBin, tmp, result, match_method );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

	
	cv::imshow("rs", result);
	//cv::threshold(result, result, 0.8, 1, CV_THRESH_BINARY);
	//cv::imshow("rs-bin", result);
	cv::waitKey();

    static int cnt = 0;
    char fname[10];
    sprintf(fname, "result%d.txt", cnt);
    FILE *out = fopen(fname, "w");
    for(int y = 0; y < result.rows; ++y)
        for(int x = 0; x < result.cols; ++x)
            fprintf(out, "%6.2f \n", result.at<float>(y, x), (x == imgBin.cols - 1 ? '\n' : ' '));
    fclose(out);
    ++cnt;
    maxLoc.x += tmp.cols/2;
    maxLoc.y += tmp.rows/2;
    return maxLoc;
}

int main() {
    cv::Mat imgRGB = cv::imread("D:/LaneData/Sample_30-04/rgb/2000.png");
    cv::Mat imgGray, imgBin;
    cv::cvtColor(imgRGB, imgGray, cv::COLOR_RGB2GRAY);
    cv::imshow("imgGray", imgGray);
	cv::threshold(imgGray, imgBin, 128, 255, CV_THRESH_BINARY | CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
    cv::imshow("imgBin", imgBin);

    cv::Size s(42, 21);
    cv::Mat tmp_left  = cv::Mat::zeros(s, CV_8UC1);
    cv::Mat tmp_right = cv::Mat::zeros(s, CV_8UC1);

    std::vector<cv::Point> vp_left;

    vp_left.push_back(cv::Point(34,0));
    vp_left.push_back(cv::Point(42,0));
    vp_left.push_back(cv::Point(8,21));
    vp_left.push_back(cv::Point(0,21));

    std::vector<cv::Point> vp_right;

    vp_right.push_back(cv::Point(0,0));
    vp_right.push_back(cv::Point(8,0));
    vp_right.push_back(cv::Point(42,21));
    vp_right.push_back(cv::Point(34,21));

    cv::fillConvexPoly( tmp_left , cv::Mat(vp_left ), cv::Scalar( 255, 255, 255 ));
    cv::fillConvexPoly( tmp_right, cv::Mat(vp_right), cv::Scalar( 255, 255, 255 ));

	cv::imshow("tmpLeft", tmp_left);
	cv::imshow("tmpRight", tmp_right);

	cv::Mat imgBin_small;
	cv::resize(imgBin, imgBin_small, cv::Size(320, 240));

    cv::Point matchLocLeft = templateMatching(imgBin, tmp_left);
    cv::Point matchLocRight = templateMatching(imgBin, tmp_right);
	cv::Point matchLocMagic_Left = templateMatching(imgBin_small, tmp_left);
	cv::Point matchLocMagic_Right = templateMatching(imgBin_small, tmp_right);

	cv::circle(imgRGB, matchLocMagic_Left * 2, 4, cv::Scalar(0, 0, 255));
	cv::circle(imgRGB, matchLocMagic_Right * 2, 4, cv::Scalar(0, 0, 255));
    cv::circle(imgRGB, matchLocLeft, 4, cv::Scalar(0, 255, 0));
    cv::circle(imgRGB, matchLocRight, 4, cv::Scalar(255, 0, 0));
    cv::imshow("img", imgRGB);
    cv::imwrite("imgTmp.png", imgRGB);
    cv::waitKey();
    return 0;
}
