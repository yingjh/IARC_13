#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "height_laser/height.h"

//Yu Yun library
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"

int fd;

char next_byte[2];
int out;

ros::Subscriber sub;
ros::Publisher pub;

enum {
    IDLE,
    LOADBUF,
    UNPACK,
};

void serial_interrupt(int sig){ // can be called asynchronously
    close(fd);
	ros::shutdown();
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

}


void get_pose()
{
	//height_laser::height msg;
/*
	if (serial_port.rdbuf()->in_avail() > 0  ) 
    {
	    serial_port.get(next_byte[1]);
	    serial_port.get(next_byte[0]);
        memcpy(&out,next_byte,2);
        printf("d\n",out);
	    serial_port.get(next_byte[0]);
    }
*/
}

int main (int argc, char** argv)
{
  	signal(SIGINT, serial_interrupt); 
	ros::init(argc, argv, "height_laser_node");
	ros::NodeHandle n;
	//pub = n.advertise<height_laser::height>("height_laser",100);
	
	ros::Rate loop_rate(1000);

    char *portname = "/dev/ttyUSB0";
    fd = open (portname, O_RDONLY | O_NOCTTY | O_NDELAY);

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

                                         // receive 25:  approx 100 uS per char transmit
    char buf[4];
    char out[4];
    out[0] = 0;
    int height;
    int out_counter = 0;
    int state = IDLE;

	while (ros::ok())
	{
		//get_pose();
        int n = read (fd, &buf, 1);  // read up to 100 characters if ready to read
        //printf("%d %d %d %d\n", buf[0], buf[1], buf[2], buf[3]);

        if (buf[0] == 10)
            printf("\n");
        else
            printf("%d", buf[0]);
        switch (state)
        {
            case IDLE:
                    if (buf[0] == 10)
                    {
                        state = LOADBUF;
                        out_counter = 1;
                    }
                break;

            case LOADBUF:
                    out[out_counter] = buf[0];
                    out_counter++;
                    if (out_counter == 4)
                    {
                        state = UNPACK;
                    }
                break;

            case UNPACK:
                memcpy(&height,out,4);
                //printf("%d\n", height);
                state = IDLE;
                break;
        }
		ros::spinOnce();
		loop_rate.sleep();
	}
    close(fd);
	return 0;
}
