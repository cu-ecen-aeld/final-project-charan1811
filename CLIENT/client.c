#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <syslog.h>
#include <opencv2/opencv.hpp>

/***********************************************************************
g++ client.c -o client `pkg-config opencv4 --libs --cflags`
***********************************************************************/

#define IMAGE_CAP "capture.ppm"
#define SIZE 10241

char marker[]="ANEESHGURRAM";

FILE* image;
int sock_fd= -1;
int st_kill_process=0;
bool ldw_detected = true;
cv::Mat binaryImage;
std::vector<cv::Vec4i> lines;

void cleanup_on_exit()
{
    close(sock_fd);
    fclose(image);
    remove(IMAGE_CAP);
    closelog();
}

// signal handler for closing the connection
void sig_handler(int signal_number)
{
    st_kill_process = 1;
    syslog(LOG_INFO, "SIGINT/ SIGTERM encountered: exiting the process...");
    cleanup_on_exit();
    exit(EXIT_SUCCESS);
}


int write_file(int sockfd)
{
	    int recv_size = 0, size = 0, read_size, write_size, packet_index = 1, retval;
    char imagearray[10241];
    char ack = '#';
    
    do
    {
        retval = read(sockfd, &size, sizeof(int));
    } while (retval < 0);

    printf("size of the image identified as %d\n", size);
   // syslog(LOG_INFO, "size of the image identified as %d\n", size);
    
    
    image = fopen(IMAGE_CAP, "wb+");

    if (image == NULL)
    {
        printf("Error has occurred. Image file could not be opened\n");
        return -1;
    }
    
    
    struct timeval timeout = {10, 0};

    fd_set fds;
    int buffer_fd;

    while (recv_size < size)
    {
        FD_ZERO(&fds);
        FD_SET(sockfd, &fds);

        buffer_fd = select(FD_SETSIZE, &fds, NULL, NULL, &timeout);
        // syslog(LOG_INFO,"Select %d",errno);

        if (buffer_fd < 0)
        {
            syslog(LOG_ERR, "error: bad file descriptor set.\n");
             perror("[-]error: bad file descriptor set.");
            cleanup_on_exit();
        }

        if (buffer_fd == 0)
        {
            syslog(LOG_ERR, "error: buffer read timeout expired.\n");
             perror("[-]error: buffer read timeout expired.");
            cleanup_on_exit();
        }

        if (buffer_fd > 0)
        {
            do
            {
                read_size = read(sockfd, imagearray, 10241);
            } while (read_size < 0);
            printf("\n\rread_size%d", read_size);
            syslog(LOG_INFO, "read_size%d", read_size);

            // Write the currently read data into our image file
            write_size = fwrite(imagearray, 1, read_size, image);
            syslog(LOG_INFO, "write_size%d", write_size);
            printf("\n\rwrite_size%d", write_size);
            if (read_size != write_size)
            {
                printf("error in read write\n");
            }

            // Increment the total number of bytes read
            recv_size += read_size;
            printf("recv_size%d", recv_size);
            printf("packet_index%d", packet_index);
            syslog(LOG_INFO, "recv_size%d", recv_size);
            syslog(LOG_INFO, "packet_index%d", packet_index);

            packet_index++;
        }
        
    }
    
    send(sockfd, &ack, 1, 0);
    printf("sent ack\n");
    syslog(LOG_INFO, "Total received image size: %i\n", recv_size);
    printf("Total received image size: %i\n", recv_size);
    fclose(image);
    
	std::vector<cv::Point> nonZeroPoints;

	cv::Mat inputImage = cv::imread(IMAGE_CAP, cv::IMREAD_GRAYSCALE);
	cv::threshold(inputImage, binaryImage, 100, 255, cv::THRESH_BINARY_INV);
	cv::imshow("DISPLAY", binaryImage);
	cv::waitKey(10);
	cv::findNonZero(binaryImage, nonZeroPoints);

	for (int i = 0; i < nonZeroPoints.size(); i++)
	{
	if (abs((binaryImage.cols / 2) - (nonZeroPoints[i].x)) < 50)
	{
		printf("BUZZER_ON %d\n\r", abs((binaryImage.cols / 2) - (nonZeroPoints[i].x)));
	}
	else
	{
		printf("BUZZER_OFF %d \n\r", abs((binaryImage.cols / 2) - (nonZeroPoints[i].x)));
	}
	}
    return 0; 
}

int main()
{
   char *ip = "172.20.10.4";
    int port = 8080;
    int e;

    int  new_sock;
    struct sockaddr_in server_addr, new_addr;
    socklen_t addr_size;
    int retval=0;

    // Set up signal handler for SIGINT (Ctrl+C)
   // signal(SIGINT, handle_interrupt);
        if ((signal(SIGINT, sig_handler) == SIG_ERR) || (signal(SIGTERM, sig_handler) == SIG_ERR))
    {
        syslog(LOG_ERR, " Signal handler error");
        cleanup_on_exit();
        exit(EXIT_FAILURE);
    }

    int client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd < 0) {
        perror("[-]Error in socket");
        exit(1);
    }
    sock_fd=client_fd;
    printf("[+]Client socket created.\n");

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);  // Use htons to convert port to network byte order
   server_addr.sin_addr.s_addr = inet_addr(ip);

    if ((retval = connect(client_fd, (struct sockaddr *)&server_addr, sizeof(server_addr))) < 0)
    {
        syslog(LOG_ERR, "Connection Failed\n");
         perror("[-]Connection Failed\n");
        exit(EXIT_FAILURE);
    }
    printf("[+]connection created.\n");
	while (!st_kill_process)
    {
        // recieve image from the server
        if (write_file(client_fd) == -1)
        {
            perror("[-]image receive failure\n");
            exit(EXIT_FAILURE);
        }
    }


/*
    while (1) {
        addr_size = sizeof(new_addr);
        new_sock = accept(sockfd, (struct sockaddr*)&new_addr, &addr_size);
        if (new_sock < 0) {
            perror("[-]Error in Accepting");
            exit(1);
        }
        printf("[+]Connection accepted from %s:%d\n", inet_ntoa(new_addr.sin_addr), ntohs(new_addr.sin_port));

        int file_count = 1; // Counter for the received files

        // Write received data to files with incremental names until connection closes
        while (1) {
            write_file(new_sock, file_count);
            printf("[+]Data written to the file: capture.ppm\n");

            // Check if there's more data to receive
		if(file_count==1000) break;

            file_count++; // Increment file count for the next file
        }

        close(new_sock);
    }*/

    close(sock_fd);
    return 0;
}
