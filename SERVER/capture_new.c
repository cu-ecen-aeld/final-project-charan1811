

/************************************include files***************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <pthread.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <syslog.h>
#include <linux/videodev2.h>
#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>

#define SIZE 10240

/************************************Macros***************************/
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define HRES 320
#define VRES 240
#define HRES_STR "320"
#define VRES_STR "240"
#define PORT 8080



#define MARKER_SIZE 100 // Define the size of the marker





// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

/************************************Enumeration***************************/
enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

/************************************Struct***************************/
struct buffer 
{
        void   *start;
        size_t  length;
};

/*******************************Global Variables**********************/
static char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = 1;
pthread_mutex_t mutexLocks[1];
bool imageFlags[1] = {false};

/*******************************Function declarations**********************/
static void errno_exit(const char *s)
{
        syslog(LOG_ERR, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}

// File names to store the images //
char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[][10]={"test1.ppm","test2.ppm","test3.ppm","test4.ppm","test5.ppm"};

char marker[]="ANEESHGURRAM";  // Define a buffer for the marker

//char *ip = "172.20.10.4";
//char *ip = "10.0.0.121";
char *ip = "192.168.18.151";
int port = 8080;
//char *filename = "/home/aneesh/courses/common_final/demo/capture.ppm";
char *filename = "/root/server_app/test1.ppm";
    

int sockfd; // Declare sockfd globally for reuse in transfer function

int init(const char *ip, int port) {

	int sockfd, new_socket = 0x00;
    struct sockaddr_in server_addr, new_addr;
    int e;
    int opt = 1;
    socklen_t addr_size;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("[-]Error in socket");
        exit(1);
    }
    printf("[+]Server socket created.\n");
    
    if (setsockopt(sockfd, SOL_SOCKET,
			   SO_REUSEADDR | SO_REUSEPORT, &opt,
			   sizeof(opt)))
    {
        syslog(LOG_ERR, "setsockopt failed \n\r");
        perror("[-]setsockopt failed");
        exit(EXIT_FAILURE);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);  // Use htons to convert port to network byte order
    server_addr.sin_addr.s_addr = inet_addr(ip);
    //server_addr.sin_addr.s_addr = INADDR_ANY;

    // Forcefully attaching socket to the port 8080
    if (bind(sockfd, (struct sockaddr *)&server_addr,sizeof(server_addr)) < 0)
    {
    	syslog(LOG_ERR,"bind failed \n\r");
    	perror("[-]bind failed");
	    exit(EXIT_FAILURE);
    }
    printf("[+]binding created.\n");
    if (listen(sockfd, 3) < 0)
    {
        syslog(LOG_ERR,"listen failed \n\r");
        perror("[-]listen failed");
        exit(EXIT_FAILURE);
    }
    printf("[+]Listening..\n");
    addr_size=sizeof(new_addr);
    new_socket = accept(sockfd, (struct sockaddr *)&new_addr,(socklen_t *)&addr_size); 
    if (new_socket < 0)
    {
        syslog(LOG_ERR,"accept failed \n\r");
        perror("[-]accept failed");
	    exit(EXIT_FAILURE);
    }
    printf("[+]Connection accepted from %s:%d\n", inet_ntoa(new_addr.sin_addr), ntohs(new_addr.sin_port));
    printf("[+]connection created.\n");

	return new_socket;
}

int send_file(void* socket,FILE *fp, int packet_index)
{
    char data[SIZE];
    size_t n;
    int state=0, read_size=0;
    
    
    	while (!feof(fp))
	{
		// while(packet_index = 1){
		// Read from the file into our send buffer
		read_size = fread(data, 1, sizeof(data) - 1, fp);
		if (read_size < 0)
			{
				//printf("fread failed");
				exit(EXIT_FAILURE);
			}
		// Send data through our socket
		do
		{
			state = write(*((int*)socket), data, read_size);
			if (state == -1)
			{
				//printf("Write failed");
				exit(EXIT_FAILURE);
			}
		} while (state < 0);

		packet_index++;

		bzero(data, sizeof(data));
	}
    
    
    return packet_index;
    
}

void transfer(void* socket, const char *filename) {

	int i=0,packet_index=0;
	while(true)
	{
		    pthread_mutex_lock(&mutexLocks[i]);
	if (!imageFlags[i]){
		pthread_mutex_unlock(&mutexLocks[i]);
		++i;
		if (i>=frame_count){
			i=0;
		}
        
		continue;
	}

    FILE *fp = fopen(filename, "r");
    printf("\n\rImage opened");
    if (fp == NULL) {
        perror("[-]Error in reading file.");
        exit(1);
    }
    
    	int size=0;
    	int retval = fseek(fp, 0, SEEK_END);
	if (retval == -1)
	{
		syslog(LOG_ERR, "fseek end failed \n\r");
		perror("[-]fseek end failed");
		exit(EXIT_FAILURE);
	}
	
	size = ftell(fp);
	if (size == -1)
	{
		syslog(LOG_ERR, "ftell failed \n\r");
		perror("[-]ftell failed");
		exit(EXIT_FAILURE);
	}
	
	retval = fseek(fp, 0, SEEK_SET);
	if (retval == -1)
	{
		syslog(LOG_ERR, "fseek set failed \n\r");
		perror("[-]fseek set failed.");
		exit(EXIT_FAILURE);
	}
	printf("\n\rsize sent");
	retval = write(*((int*)socket), (void *)&size, sizeof(int));
	if (retval == -1)
	{
		syslog(LOG_ERR, "Write failed \n\r");
		perror("[-]Write failed .");
		exit(EXIT_FAILURE);
	}
	
	// can implement reading something from client.

    packet_index= send_file(socket,fp, packet_index);
    
    	char ack;
	int ackRead = read(*(int*)socket, &ack, 1);
	printf("\n\rack received");
	if (ackRead == -1){
		perror("[-]Ack Read failed  .");
		syslog(LOG_ERR, "Ack Read failed \n\r");
	}
    
    
    printf("[+]File data sent successfully.\n");

    fclose(fp);
    
    	imageFlags[i] = false;
	pthread_mutex_unlock(&mutexLocks[i]);
	++i;
	if (i>=frame_count){
		i=0;
	}
	}


    
}

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{

    // Lock before writing //
    pthread_mutex_lock(&mutexLocks[tag-1]);
    if (imageFlags[tag-1]){
    	pthread_mutex_unlock(&mutexLocks[tag-1]);
    	return;
    }
    int written, total, dumpfd;
    
    // Open file descriptor //
    dumpfd = open(ppm_dumpname[tag-1], O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);
    written=write(dumpfd, ppm_header, sizeof(ppm_header));

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    close(dumpfd);
    imageFlags[tag-1] = true;
    // Unlock after writing //
    pthread_mutex_unlock(&mutexLocks[tag-1]);
    
}

// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}

unsigned int framecnt=0;
unsigned char bigbuffer[(1280*960)];

static void process_image(const void *p, int size)
{
    int i, newi;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    if (framecnt >= (frame_count+1)){
    	framecnt = frame_count;
    }
    ////printf("Dump YUYV converted to RGB size %d\n", size);
       
    // Pixels are YU and YV alternating, so YUYV which is 4 bytes
    // We want RGB, so RGBRGB which is 6 bytes
    //
    for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
    {
        y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
        yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi+1], &bigbuffer[newi+2]);
        yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi+3], &bigbuffer[newi+4], &bigbuffer[newi+5]);
    }
    dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time);

    fflush(stderr);
    fflush(stdout);
}


static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io)
    {

        case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length))
            {
                switch (errno)
                {

                    case EAGAIN:
                        return 0;

                    case EIO:

                    default:
                        errno_exit("read");
                }
            }

            process_image(buffers[0].start, buffers[0].length);
            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        return 0;


                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            assert(buf.index < n_buffers);

            process_image(buffers[buf.index].start, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

            assert(i < n_buffers);

            process_image((void *)buf.m.userptr, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;
    }

    return 1;
}


static void mainloop(void)
{
    unsigned int count;
    struct timespec read_delay;
    struct timespec time_error;

    read_delay.tv_sec=0;
    read_delay.tv_nsec=30000;

    count = frame_count;

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                exit(EXIT_FAILURE);
            }
            if (read_frame())
            {
                if(nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                else
                    syslog(LOG_INFO,"time_error.tv_sec=%ld, time_error.tv_nsec=%ld\n", time_error.tv_sec, time_error.tv_nsec);

                count--;
                if (count <=0){
                	count = frame_count;
                }
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if(count <= 0){
            	count = frame_count;
            	break;
            }
        }

        if(count <= 0){
           count = frame_count;
     	   break;
        }
    }
}

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) 
        {

        case IO_METHOD_READ:
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) 
                {
                        syslog(LOG_INFO,"allocated buffer %d\n\r", n_buffers);
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}

static void uninit_device(void)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free(buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free(buffers[i].start);
                break;
        }

        free(buffers);
}

static void init_read(unsigned int buffer_size)
{
        buffers = (buffer*)calloc(1, sizeof(*buffers));

        if (!buffers) 
        {
                syslog(LOG_ERR, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) 
        {
                syslog(LOG_ERR, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 3;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        syslog(LOG_ERR, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) 
        {
                syslog(LOG_ERR, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = (buffer*)calloc(req.count, sizeof(*buffers));

        if (!buffers) 
        {
                syslog(LOG_ERR, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL,
                              buf.length,
                              PROT_READ | PROT_WRITE,
                              MAP_SHARED,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        syslog(LOG_ERR, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        buffers = (buffer*)calloc(4, sizeof(*buffers));

        if (!buffers) {
                syslog(LOG_ERR, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                        syslog(LOG_ERR, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            syslog(LOG_ERR, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        syslog(LOG_ERR, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    switch (io)
    {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE))
            {
                syslog(LOG_ERR, "%s does not support read i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING))
            {
                syslog(LOG_ERR, "%s does not support streaming i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    break;
                default:
                        break;
            }
        }

    }
    else
    {
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");
    }
    else
    {
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
        {
        	errno_exit("VIDIOC_G_FMT");	
        }
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    switch (io)
    {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}


static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

static void open_device(void)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                syslog(LOG_ERR, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                syslog(LOG_ERR, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);

        if (-1 == fd) {
                syslog(LOG_ERR, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

void *socketThread(void* arg){
    //send_image(arg);
    transfer((arg),filename);
    printf("Image sending Completed\n");
    return NULL;
}

void *captureThread(void* arg){
    mainloop();
    return NULL;
}

int main(int argc, char **argv)
{
    pthread_t sockThread, capThread;
    openlog(NULL, LOG_CONS | LOG_PID | LOG_PERROR, LOG_USER);
    if(argc > 1)
        dev_name = argv[1];
    else
        dev_name = "/dev/video1";
    
    open_device();
    init_device();
    start_capturing();
    
    sockfd= init(ip, port);
    pthread_create(&capThread, NULL, captureThread, NULL);
    
    pthread_create(&sockThread, NULL, socketThread, &sockfd);

    pthread_join(capThread, NULL);
    pthread_join(sockThread, NULL);

    stop_capturing();
    uninit_device();
    close_device();

    return 0;
}
