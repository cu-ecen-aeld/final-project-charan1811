/*
g++ client.c -o client `pkg-config opencv4 --libs --cflags`
*/
/************************************include files***************************/
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/select.h>
#include <stdlib.h>
#include <stdbool.h>
#include <syslog.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <errno.h>
#include <opencv2/opencv.hpp>

#define IMAGE_CAP "test_latest1.jpg"

bool st_kill_process = false;
int client_fd = -1;
FILE *image;
bool ldw_detected = true;
cv::Mat binaryImage;
std::vector<cv::Vec4i> lines;

int receive_image(int comp)
{
	int recv_size = 0, size = 0, read_size, write_size, packet_index = 1, retval;
	std::vector<cv::Point> nonZeroPoints;
    	
	cv::Mat inputImage = cv::imread(IMAGE_CAP, cv::IMREAD_GRAYSCALE);
	cv::threshold(inputImage, binaryImage, 100, 255, cv::THRESH_BINARY_INV);
	cv::imshow("BW_IMAGE", binaryImage);
	cv::waitKey(10);
	cv::findNonZero(binaryImage, nonZeroPoints);
	for (int i = 0; i < nonZeroPoints.size(); i++)
	{
		if (abs((binaryImage.cols / 2) - (nonZeroPoints[i].x)) < comp)
		{
			printf("BUZZER ON %d\n\r", abs((binaryImage.cols / 2) - (nonZeroPoints[i].x)));
		}
		else
		{
			printf("BUZZER OFF %d \n\r", abs((binaryImage.cols / 2) - (nonZeroPoints[i].x)));
		}
	}
	
	return 1;
}

int main(int argc, char* argv[])
{
	openlog(NULL, LOG_CONS | LOG_PID | LOG_PERROR, LOG_USER);

	cv::namedWindow("BW_IMAGE", cv::WINDOW_NORMAL);
	cv::resizeWindow("BW_IMAGE", 500,500);

	while (!st_kill_process)
	{
		// recieve image from the server
		if (receive_image(50) == -1)
		{
			syslog(LOG_ERR, "Unable to receive image \n");
			return -1;
		}
	}

	close(client_fd);
	return 0;
}
