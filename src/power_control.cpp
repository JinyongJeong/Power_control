#include <stdio.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#ifdef __linux__
#include <linux/serial.h>
#define SUPPORT_HISPEED 1
#endif

using namespace std;

int fd;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port " << msg->data);

    write(fd,msg->data.c_str(),2);
}


int 
_serial_set_canonical (int fd)
{
    struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
    //    PERROR ("tcgetattr()");
        return -1;
    }

    tio.c_lflag |= ICANON;

    if ( tcsetattr (fd, TCSANOW, &tio) ) {
    //    PERROR ("tcsetattr()");
        return -1;
    }

    return 0;
}
int serial_translate_baud(int inrate)
{
	switch(inrate)
	{
	case 0:
		return B0;
	case 300:
		return B300;
	case 1200:
		return B1200;
	case 2400:
		return B2400;
	case 4800:
		return B4800;
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
#ifdef SUPPORT_HISPEED
	case 460800:
		return B460800;
#endif
	default:
		return -1; // do custom divisor
	}
}

int serial_open(const char *port, int baud, int blocking)
{
	struct termios opts;
	int flags = O_RDWR | O_NOCTTY;
	if (!blocking)
		flags |= O_NONBLOCK;

	fd=open(port, flags, 0);

	if (fd==-1)
		return -1;

	if (tcgetattr(fd, &opts))
	{
		printf("*** %i\n",fd);
		perror("tcgetattr");
		return -1;
	}
	cfsetispeed(&opts, serial_translate_baud(baud));
	cfsetospeed(&opts, serial_translate_baud(baud));
	cfmakeraw(&opts);
    opts.c_cflag &= ~CSTOPB;
	if (tcsetattr(fd,TCSANOW,&opts))
	{
		perror("tcsetattr");
		return -1;
	}

	tcflush(fd, TCIOFLUSH);
	return fd;
}



int 
main (int argc, char *argv[])
{

    ros::init(argc, argv, "power_control_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("/power_control_write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/power_control_read", 1000);


    std::string ttydev;
    int brate = 115200;
    ros::param::param<std::string>("~device", ttydev, "/dev/ttyUSB-power");
    ros::param::param<int>("~baudrate", brate, 115200);
    int fd = serial_open (ttydev.c_str(), brate, 0);

    char buf [50];
    int len;
    memset (&buf, '\0', 50);

    _serial_set_canonical (fd);

    struct pollfd poll_events;

    int poll_state;

    poll_events.fd = fd;
    poll_events.events = POLLIN|POLLERR;
    poll_events.revents = 0;

    while(ros::ok()) {
        
        poll_state = poll((struct pollfd*)&poll_events, 1, 1);

        if(0 < poll_state) {
            if (poll_events.revents & POLLIN) {
                ROS_INFO_STREAM("Reading from serial port");
                std_msgs::String result;
                
                len = read (fd, buf, 50);
                result.data.assign(buf, 50);
                read_pub.publish(result);

            }

            if (poll_events.revents & POLLERR) {
                printf( "receiver error!" );
                break;
            }
        }
        ros::spinOnce();
    }

    close(fd);

    return 0;
}


