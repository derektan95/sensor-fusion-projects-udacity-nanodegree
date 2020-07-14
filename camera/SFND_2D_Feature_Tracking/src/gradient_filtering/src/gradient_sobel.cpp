#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gradientSobel()
{
    // TODO: Based on the image gradients in both x and y, compute an image
    // which contains the gradient magnitude according to the equation at the
    // beginning of this section for every pixel position. Also, apply different
    // levels of Gaussian blurring before applying the Sobel operator and compare the results.

    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // show result of Gaussian Filter
    cv::namedWindow("Original Gray Image", 1); // create window
    cv::imshow("Original Image", imgGray);
    cv::waitKey(0); // wait for keyboard input before continuing

    // apply Gaussian smoothing
    cv::Mat img_blurred;
    int filterSize = 5;
    int stdDev = 2.0;
    cv::GaussianBlur(imgGray, img_blurred, cv::Size(filterSize, filterSize), stdDev);

    // show result of Gaussian Filter
    cv::namedWindow("Gaussian Filtering", 1); // create window
    cv::imshow("Gaussian Filtering", img_blurred);
    cv::waitKey(0); // wait for keyboard input before continuing

    // create Sobel filter kernel
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2,
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);

    // create Sobel filter kernel
    float sobel_y[9] = {-1, -2, -1,
                        0, 0, 0,
                        +1, +2, +1};
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

    // apply Sobel filter
    cv::Mat result_sobel_x;
    cv::filter2D(img_blurred, result_sobel_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::Mat result_sobel_y;
    cv::filter2D(img_blurred, result_sobel_y, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // Take root sum squared of magnitude of filter at different directions
    cv::Mat magnitudeImg = imgGray.clone();
    for (size_t i = 0; i < magnitudeImg.rows; i++)
    {
        for (size_t j = 0; j < magnitudeImg.cols; j++)
        {
            magnitudeImg.at<unsigned char>(i, j) = sqrt(pow(result_sobel_x.at<unsigned char>(i, j), 2) + pow(result_sobel_y.at<unsigned char>(i, j), 2));
        }
    }

    // show result of Gaussian Filter
    cv::namedWindow("Sobel Filtering", 1); // create window
    cv::imshow("Sobel Filtering", magnitudeImg);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    gradientSobel();
}