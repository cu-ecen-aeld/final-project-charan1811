#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <syslog.h>
#include <opencv2/opencv.hpp>
#include <wiringPi.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <linux/i2c-dev.h>
#include <semaphore.h>

#define SIZE 10241

char marker[]="ANEESHGURRAM";
#define IMAGE_CAP "capture.ppm"
FILE* image;
int sock_fd= -1;
int st_kill_process=0;
bool ldw_detected = true;
cv::Mat binaryImage;
std::vector<cv::Vec4i> lines;
int res =0;
pthread_mutex_t mutexLocks;
sem_t sem1, sem2;

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
    int count = 0;
    
    do
    {
        retval = read(sockfd, &size, sizeof(int));
    } while (retval < 0);

    printf("size of the image identified as %d\n", size);
    
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
    syslog(LOG_INFO, "Total received image size: %i\n", recv_size);
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
		//printf("OUT_OF_LANE %d\n\r", abs((binaryImage.cols / 2) - (nonZeroPoints[i].x)));
		digitalWrite (0, 0) ;
	    }
	    else
	    {
		//printf("INSIDE_OF_LANE %d \n\r", abs((binaryImage.cols / 2) - (nonZeroPoints[i].x)));
		digitalWrite (0, 1) ;
	    }
	}
        
    syslog(LOG_INFO, "returning 0\n", recv_size);
    sem_post(&sem1);
    return 0;
    
}

void readI2C(unsigned char *data, int bytes) {
	int i2cAddress = 0x57;
	int i2cHandle;
	char *deviceName = "/dev/i2c-1";
    
	// Open the I2C bus
	if ((i2cHandle = open(deviceName, O_RDWR)) < 0) {
    	perror("Error opening I2C");
    	exit(1);
	}
    
	// Set the I2C slave address
	if (ioctl(i2cHandle, I2C_SLAVE, i2cAddress) < 0) {
    	perror("Error setting I2C slave address");
    	exit(1);
	}

	// Write a '1' to start ranging session
	unsigned char startSession = 0x01;
	if (write(i2cHandle, &startSession, 1) != 1) {
    	perror("Error writing to I2C");
    	exit(1);
	}
    
	// Wait a few milliseconds (you may need to adjust this)
	usleep(15000); // Wait for 10 milliseconds
    
	// Read the 3-byte response
	if (read(i2cHandle, data, bytes) != bytes) {
    	perror("Error reading from I2C");
    	exit(1);
	}
    
	// Close the I2C bus
	close(i2cHandle);
}

/*
void handle_interrupt(int signum) {
    if (signum == SIGINT) {
        printf("\nReceived SIGINT. Exiting.\n");
        exit(0);
    }
}*/

void* sensor_thread(void*)
{
    unsigned char data[3];
    
    while(1)
    {
        sem_wait(&sem1);
        readI2C(data, 3);
          // Combine the bytes to get the distance in um
        unsigned int distance = (data[0] << 16) | (data[1] << 8) | data[2];
        // Convert um to mm
        float distance_mm = distance / 1000.0;
        //printf("Distance: %.2f mm\n", distance_mm);
        if(distance_mm == 0)
        {
          digitalWrite (7, 1) ;
        }
        else if (distance_mm < 50)
        {
          digitalWrite (7, 0) ;
        }
        
        delay(200); // Delay for 1 second
        digitalWrite (7, 1) ;
    }
}

void* wfile_thread(void* clientfd)
{
    int * client_fd = (int*) clientfd;
    cv::namedWindow("DISPLAY", cv::WINDOW_NORMAL);
    cv::resizeWindow("DISPLAY", 500,500);
	while (!st_kill_process)
    {
        // recieve image from the server
        if (write_file(*client_fd) == -1)
        {
            perror("[-]image receive failure\n");
            exit(EXIT_FAILURE);
        }
        printf("D\n");
    }
}

int main()
{
   //char *ip = "172.20.10.4";
   //char *ip = "10.0.0.121";
   char *ip = "192.168.18.151";
    int port = 8080;
    int e;
    pthread_t wfilethread, sensorthread;
    size_t stack_size = 8026;
    pthread_attr_t attr, attr1;
    pthread_attr_init(&attr);
    pthread_attr_init(&attr1);
    pthread_attr_setstacksize(&attr, stack_size);
    cpu_set_t thread_cpu, thread_cpu_1;
    CPU_ZERO(&thread_cpu);
    CPU_SET(3, &thread_cpu);
    //CPU_ZERO(&thread_cpu_1);
    //CPU_SET(2, &thread_cpu_1);
    
    if (wiringPiSetup () == -1)
        return 1 ;
        
    pinMode (0, OUTPUT) ;         // aka BCM_GPIO pin 17
    pinMode (14, INPUT) ;         // aka BCM_GPIO pin 17
    pinMode(2, INPUT); // Set BCM pin 11 as input
    pinMode (7, OUTPUT) ;
    digitalWrite (0, 0) ;

    int  new_sock;
    struct sockaddr_in server_addr, new_addr;
    socklen_t addr_size;
    int retval=0;
    
    sem_init(&sem1,0,0);
    sem_init(&sem2,0,0);

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
    pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &thread_cpu);
    //pthread_attr_setaffinity_np(&attr1, sizeof(cpu_set_t), &thread_cpu_1);
    pthread_create(&wfilethread, NULL, wfile_thread, &client_fd);
    pthread_create(&sensorthread, &attr, sensor_thread, NULL);
    pthread_join(wfilethread, NULL);
    pthread_join(sensorthread, NULL);
    /*while (!st_kill_process)
    {
        // recieve image from the server
        if (write_file(client_fd) == -1)
        {
            perror("[-]image receive failure\n");
            exit(EXIT_FAILURE);
        }
    }*/



    close(sock_fd);
    return 0;
}
