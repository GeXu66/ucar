#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
  
#include <boost/thread/thread.hpp>  
#include <ros/ros.h>  
#include <std_msgs/Char.h>
  
#define KEYCODE_W 0x77  
#define KEYCODE_A 0x61  
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71  
  
#define KEYCODE_A_CAP 0x41  
#define KEYCODE_D_CAP 0x44  
#define KEYCODE_S_CAP 0x53  
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
  
class SmartCarKeyboardTeleopNode  
{  
private:
    ros::NodeHandle n_;  
    ros::Publisher pub_;  

public:  
    SmartCarKeyboardTeleopNode()  
    {  
        pub_ = n_.advertise<std_msgs::Char>("keyboard_pub", 1);  
        ros::NodeHandle n_private("~");  
    }  
    void keyboardLoop();
    std_msgs::Char cmdvel_;  
};  
  
SmartCarKeyboardTeleopNode* tbk;  
int kfd = 0;  
struct termios cooked, raw;  
bool done;  
  
int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"keyboard_node");  
    SmartCarKeyboardTeleopNode tbk;  
      
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));  
      
    ros::spin();  
      
    t.interrupt();  
    t.join();
    tcsetattr(kfd, TCSANOW, &cooked);  
      
    return(0);  
}  
  
void SmartCarKeyboardTeleopNode::keyboardLoop()  
{  
    char c;  
    bool dirty = false;  
    int speed = 0;  
    int turn = 0;  
      
    // get the console in raw mode  
    tcgetattr(kfd, &cooked);  
    memcpy(&raw, &cooked, sizeof(struct termios));  
    raw.c_lflag &=~ (ICANON | ECHO);  
    raw.c_cc[VEOL] = 1;  
    raw.c_cc[VEOF] = 2;  
    tcsetattr(kfd, TCSANOW, &raw);  
      
    puts("Reading from keyboard");  
    puts("Use \"S\" to Start and \"Q\" to Stop");
      
    struct pollfd ufd;  
    ufd.fd = kfd;  
    ufd.events = POLLIN;  
      
    for(;;)  
    {  
        boost::this_thread::interruption_point();  
          
        // get the next event from the keyboard  
        int num;  
          
        if ((num = poll(&ufd, 1, 250)) < 0)  
        {  
            perror("poll():");  
            return;  
        }  
        else if(num > 0)  
        {  
            if(read(kfd, &c, 1) < 0)  
            {  
                perror("read():");  
                return;  
            }  
        }  
        else  
        {  
            if (dirty == true)  
            {  
                dirty = false;  
            }  
              
            continue;  
        }  
          
        switch(c)  
        {  
            case KEYCODE_W:
            case KEYCODE_W_CAP:
                dirty = true;
                cmdvel_.data = 'w';
                break;  
            case KEYCODE_S:
            case KEYCODE_S_CAP:
                cmdvel_.data = 's'; 
                dirty = true;  
                break;  
            case KEYCODE_A:
            case KEYCODE_A_CAP:
                cmdvel_.data = 'a';
                dirty = true;  
                break;  
            case KEYCODE_D: 
            case KEYCODE_D_CAP: 
                dirty = true;  
                cmdvel_.data = 'd';
                break; 
            case KEYCODE_Q:
            case KEYCODE_Q_CAP:
                cmdvel_.data = 'q';
                break;
            default:
                cmdvel_.data = ' ';
                dirty = false;  
        }  
        pub_.publish(cmdvel_);  
    }  
}
