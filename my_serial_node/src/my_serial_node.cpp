#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

serial::Serial ros_ser;
#define	sBUFFERSIZE	10//send buffer size 串口发送缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
int l_v=0;
int r_v=0;
int l_d=1;
int r_d=1;
const float d_between=0.165;//半轮间距
const float d=0.125;//轮子半径

typedef union{
	unsigned char cvalue[4];
	int ivalue;
}int_union;

void cmd_to_serial(const geometry_msgs::Twist msg)
{
    r_v=27*(msg.linear.x + msg.angular.z * d_between)/(3.1416*d);
    l_v=27*(msg.linear.x - msg.angular.z * d_between)/(3.1416*d);
    if(r_v>=0)
    {
        r_d=0;
        r_v=r_v;
    }             //电机正转
    else
    {
        r_d=1;
        r_v=-r_v;
    }           //电机反转

    if(l_v>=0)
    {
        l_d=0;
        l_v=l_v;
    }
    else
    {
        l_d=1;
        l_v=-l_v;
    }

}

void data_to_serial(const int left, const int right, const int left_d, const int right_d)
{
     int_union left_v,right_v,left_v_d,right_v_d;
     left_v.ivalue=left;
     right_v.ivalue=right;
     memset(s_buffer,0,sizeof(s_buffer));
     s_buffer[0]=0xff;
     s_buffer[1]=0xfe;

     s_buffer[2]=left_v.cvalue[0];
     s_buffer[3]=right_v.cvalue[0];

     left_v_d.ivalue=left_d;
     right_v_d.ivalue=right_d;
     s_buffer[4]=left_v_d.cvalue[0];
     s_buffer[5]=right_v_d.cvalue[0];

     s_buffer[6]=0x00;
     s_buffer[7]=0x00;
     s_buffer[8]=0x00;
     s_buffer[9]=0x00;
     ros_ser.write(s_buffer,sBUFFERSIZE);
}
//回调函数
void callback(const geometry_msgs::Twist& msg){
     cmd_to_serial(msg);
     data_to_serial(l_v,r_v,l_d,r_d);
 }
int main (int argc, char** argv){
     ros::init(argc, argv, "my_serial_node");
     ros::NodeHandle n;
     //订阅主题command
     ros::Subscriber command_sub = n.subscribe("/cmd_vel", 1000, callback);
     //发布主题sensor
     ros::Publisher sensor_pub = n.advertise<std_msgs::String>("sensor", 1000);
     try
     {
         ros_ser.setPort("/dev/ttyUSB0");
         ros_ser.setBaudrate(115200);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser.setTimeout(to);
         ros_ser.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }
     if(ros_ser.isOpen()){
         ROS_INFO_STREAM("Serial Port opened");
     }else{
         return -1;
     }
     ros::Rate loop_rate(10);
     while(ros::ok()){
         ros::spinOnce();
         if(ros_ser.available()){
             ROS_INFO_STREAM("Reading from serial port");
             std_msgs::String serial_data;
             //获取串口数据
             serial_data.data = ros_ser.read(ros_ser.available());
             ROS_INFO_STREAM("Read: " << serial_data.data);
             //将串口数据发布到主题sensor
             sensor_pub.publish(serial_data);
         }
         loop_rate.sleep();
     }
 }
