#include <ros/ros.h>
#include <magellan_messages/MsgSerialPortLine.h>
#include <unistd.h>
#include <stdexcept>
#include <ctype.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <limits>

#define USB_PORT_MAX_LEN 128
#define CLOSING_CMD_MAX_LEN 1024
#define RECEIVE_BUFFER_SIZE 4096

static int serial_port = -1;

bool init_serial_port(long baud, const char* serial_port_name)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty))
    {
        ROS_FATAL(
                "Could not read attributes from serial port %s. Received error %d: %s",
                serial_port_name,
                errno,
                strerror(errno)
        );
        return false;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    if (tcsetattr(serial_port, TCSANOW, &tty)) 
    {
        ROS_FATAL(
                "Could not write attributes to serial port %s. Received error %d: %s",
                serial_port_name,
                errno, 
                strerror(errno)
        );
        return false;
    }

    return true;
}

void write_data_to_serial_port(const unsigned char* data, size_t len)
{
    if (serial_port >= 0 && len > 0)
    {
        ssize_t num_bytes_written = 0;
        ssize_t current_num_written = 0;
        
        while(num_bytes_written < len)
        {
            current_num_written = write(serial_port, data, len - num_bytes_written);
            if (current_num_written < 0)
            {
                if (errno == EINTR || errno == EAGAIN)
                {
                    continue;
                }

                ROS_WARN(
                        "Could not write data to serial port. Errno = %d. %s.",
                        errno,
                        strerror(errno)
                );
            }

            num_bytes_written += current_num_written;
        }
    }
}

void message_received_callback(const magellan_messages::MsgSerialPortLine::ConstPtr &input_data)
{
    write_data_to_serial_port(&input_data->data[0], input_data->data.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magellan_serial_port");

    char serial_port_name[USB_PORT_MAX_LEN];
    size_t baud;
    uint8_t separator;
    bool separator_valid = false;
    size_t min_message_length = 0;
    size_t max_message_length = std::numeric_limits<size_t>::max();
    uint8_t message_header;
    bool message_header_valid = false;
    unsigned char closing_command[CLOSING_CMD_MAX_LEN];
    size_t closing_command_length = 0;

    closing_command[0] = 0;

    int c;
    while((c = getopt(argc, argv, "n:b:s:m:a:h:c:")) != -1)
    {
        size_t len = 0;
        switch (c)
        {
            case 'n':
                len = strnlen(optarg, USB_PORT_MAX_LEN + 1);
                if (len > USB_PORT_MAX_LEN)
                {
                    ROS_FATAL("The USB port name is too long.");
                    return 1;
                }

                strncpy(serial_port_name, optarg, USB_PORT_MAX_LEN);
                break;
            case 'b':
                try
                {
                    baud = std::stol(optarg, NULL);
                    break;
                }
                catch (std::exception ex)
                {
                    ROS_FATAL(
                            "Could not parse baud rate argument from string %s. The following error was encountered: %d: %s",
                            optarg,
                            errno, 
                            ex.what()
                    );
                    return 1;
                }
            case 's':
                if (strlen(optarg) != 1)
                {
                    ROS_FATAL(
                            "Could not parse separator from string %s. The string was not a single character.",
                            optarg
                    );
                    return 1;
                }

                separator = optarg[0];
                separator_valid = true;
                break;
            case 'm':
                try
                {
                    min_message_length = std::stol(optarg, NULL);
                    break;
                }
                catch (std::exception ex)
                {
                    ROS_FATAL(
                            "Could not parse min message length from string %s. The following error was encountered: %d: %s",
                            optarg,
                            errno,
                            ex.what()
                    );
                    return 1;
                }
            case 'a':
                try
                {
                    max_message_length = std::stol(optarg, NULL);
                    break;
                }
                catch (std::exception ex)
                {
                    ROS_FATAL(
                            "Could not parse max message length from string %s. The following error was encountered: %d: %s",
                            optarg,
                            errno,
                            ex.what()
                    );
                    return 1;
                }
            case 'h':
                try
                {
                    message_header = std::stol(optarg, NULL, 16);
                    message_header_valid = true;
                }
                catch (std::exception ex)
                {
                    ROS_FATAL(
                            "Could not parse header from string %s. The following error was encountered: %d: %s",
                            optarg,
                            errno,
                            ex.what()
                    );
                    return 1;
                }
            case 'c':
                len = strnlen(optarg, CLOSING_CMD_MAX_LEN + 1);
                if (len > CLOSING_CMD_MAX_LEN)
                {
                    ROS_FATAL(
                            "The closing command is too long."
                    );
                    return 1;
                }

                //TODO: this is a bit ugly
                strncpy((char*)closing_command, optarg, CLOSING_CMD_MAX_LEN);
                closing_command_length = len;
                break;
        }
    }

    serial_port = open(serial_port_name, O_RDWR | O_NONBLOCK);

    if (serial_port < 0)
    {
        ROS_FATAL(
                "Could not open serial port %s. Open returns %d",
                serial_port_name,
                serial_port
        );

        return 1;
    }

    if (!init_serial_port(baud, serial_port_name))
    {
        // error already logged inside init_serial_port.
        return 1;
    }

    uint8_t receive_array[RECEIVE_BUFFER_SIZE];

    // for now
    for (unsigned int i = 0; i < RECEIVE_BUFFER_SIZE; i++)
    {
        receive_array[i] = 0;
    }

    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<magellan_messages::MsgSerialPortLine>("output_topic", 1000);
    ros::Subscriber subscriber = nh.subscribe<magellan_messages::MsgSerialPortLine>("input_topic", 1000, message_received_callback);

    // TODO: proper rate?
    ros::Rate loop_rate(1000);
    bool message_started = false;
    magellan_messages::MsgSerialPortLine serial_port_output_data;
    serial_port_output_data.data.reserve(1000); //TODO: Is there a better way of determining this buffer size?

    while(ros::ok())
    {
        ssize_t num_bytes_read = read(serial_port, receive_array, RECEIVE_BUFFER_SIZE);

        if (num_bytes_read < 0)
        {
            if (errno != EAGAIN || errno != EWOULDBLOCK || errno != EINTR)
            {
                ROS_WARN(
                        "Could not read from serial port %s. Error: %d: %s",
                        serial_port_name,
                        errno,
                        strerror(errno)
                );
            }
        }
        else
        {
            for (int i = 0; i < num_bytes_read; i++)
            {
                uint8_t current_byte = receive_array[i];

                // Check if byte is valid
                if (!message_header_valid || current_byte == message_header)
                {
                    message_started = true;
                }

                if (!message_started)
                {
                    continue;
                }
                
                serial_port_output_data.data.push_back(current_byte);

                // Check for send conditions
                size_t num_bytes = serial_port_output_data.data.size();
                if (num_bytes < min_message_length)
                {
                    continue;
                }

                if (num_bytes >= max_message_length
                        ||
                    ((separator_valid) && (current_byte == separator)))
                {
                    serial_port_output_data.header.stamp = ros::Time::now(); //TODO: is this needed?
                     publisher.publish(serial_port_output_data);
                     serial_port_output_data.data.clear();
                     message_started = false;
                }
            }
        }
        
        ros::spinOnce();

        loop_rate.sleep();
    }

    if (closing_command_length > 0)
    {
        write_data_to_serial_port(closing_command, closing_command_length);
    }

    close(serial_port);
    serial_port = -1;
}
