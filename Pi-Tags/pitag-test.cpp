/*
 * Copyright (c) 2017 Matthew Petroff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "FiducialDefines.h"
#include "FiducialModelPi.h"

using namespace ipa_Fiducials;

void draw_tags(std::vector<t_points> vec_points, cv::Mat &image) {
	for (unsigned int i = 0; i < vec_points.size(); i++)
	{
		bool connect_points = false;
		cv::Vec3b rgbValVec[] = { cv::Vec3b(0,0,0), cv::Vec3b(255,255,255),
					cv::Vec3b(255,0,0), cv::Vec3b(0,255,255), cv::Vec3b(0,255,0) };

		for (unsigned int j = 0; j < vec_points[i].image_points.size(); j++)
		{
			cv::Vec3b rgbVal = rgbValVec[4];
			if (vec_points[i].image_points[j].x != 0)
			{
				cv::circle(image, vec_points[i].image_points[j], 3, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
				if (connect_points)
				{
					cv::line(image, vec_points[i].image_points[j - 1], vec_points[i].image_points[j], cv::Scalar(rgbVal[0], rgbVal[1], rgbVal[2]), 1, cv::LINE_AA);
				}
				connect_points = true;
			}
			else
				connect_points = false;
		}
	}
	return;
}


int main(int argc, char** argv)
{
	if (argc != 3)
	{
		puts("Usage: pitag-test modelfile imagefile");
		return 1;
	}

	std::shared_ptr<AbstractFiducialModel> tag_detector;
	cv::Mat camera_matrix;
	std::string model_filename = argv[1];
	std::string image_filename = argv[2];
	std::vector<t_pose> tags_vec;

	tag_detector = std::shared_ptr<FiducialModelPi>(new FiducialModelPi());

	camera_matrix = cv::Mat::zeros(3, 3, CV_64FC1);

	if (tag_detector->Init(camera_matrix, model_filename, false) & RET_FAILED)
	{
		puts("Initializing fiducial detector with camera matrix [FAILED]");
		return -1;
	}

	cv::VideoCapture cap(0);

	// Check if camera opened successfully
	if (!cap.isOpened()) {
		std::cout << "Error opening video stream or file" << std::endl;
		return -1;
	}

	while (1) {
		cv::Mat frame;
		//image = cv::imread(image_filename);
		// Capture frame-by-frame
		//cap >> frame;
		cap.read(frame);

		// If the frame is empty, break immediately
		if (frame.empty())
			break;

		unsigned long ret_val = RET_OK;

		std::vector<t_points> vec_points;
		tag_detector->GetPoints(frame, vec_points);

		bool draw = true;
		if (draw) {
			draw_tags(vec_points, frame);
		}


	//for (unsigned int i = 0; i < vec_points.size(); i++) {
	//	std::cout << "Detected Tag " << vec_points[i].id << ": " << vec_points[i].image_points << std::endl;
	//}

	// Press  ESC on keyboard to exit
	char c = (char)cv::waitKey(25);
	if (c == 27)
		break;

	// Display the resulting frame
	cv::imshow("Frame", frame);
	}

	// When everything done, release the video capture object
	cap.release();

	// Closes all the frames
	cv::destroyAllWindows();

	return 0;
}
