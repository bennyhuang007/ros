#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <unistd.h>  
#include <sys/types.h>  
#include <linux/limits.h>  
#include <pty.h>
#include <utmp.h>
#include <fcntl.h>

#include <sstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

int pty=0,slave=0;

#define HEX2ASCII(a)    ((a)<=9?(a)+'0':(a)-10+'A')

static int ASCII2HEX(char c)
{
  if((c>='0')&&(c<='9'))
    return c-'0';
  else if((c>='A')&&(c<='F'))
    return c-'A' + 10;
  else if((c>='a')&&(c<='f'))
    return c-'a' + 10;
  else
    return -1;
}

static void main_loop(ros::Publisher *pub);
static void terminal_in_callback(const std_msgs::String::ConstPtr& msg);
static void setWindowSize(int width, int height);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terminal");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("crazy_term_out", 1000);
  ros::Subscriber sub = n.subscribe("crazy_term_in", 1000, terminal_in_callback);

  char pty_name[PATH_MAX];
  int ret;
  pid_t child;

  ret = openpty(&pty, &slave, pty_name, NULL, NULL);
  if (ret == -1)
  {
    ROS_ERROR("openpty fail");
    exit(EXIT_FAILURE);
  }else
    ROS_INFO("current pty:%s",pty_name);

  child = fork();
  if (child < 0)
  {
    ROS_ERROR("fork fail");
    exit(EXIT_FAILURE);
  }else if(child == 0)
  {
    close(pty);
    login_tty(slave);
    execl("/bin/login", "/bin/login", NULL);
    //execl("/bin/bash", "/bin/bash", NULL);
    ROS_INFO("child bash over !");
  }else
  {
    close(slave);
    ret = fcntl(pty, F_GETFL, 0); 
    fcntl(pty,F_SETFL, ret | O_NONBLOCK);
    main_loop(&pub);
  }
  return 0;
}

static void main_loop(ros::Publisher *pub)
{
  boost::property_tree::ptree root,child;
  char buffer[512];
  int ret;
  while (ros::ok())
  {
    ret = read(pty,buffer,sizeof(buffer));
    if(ret > 0)
    {
      std_msgs::String msg;
      std::string output(2*ret,'\0');
      std::stringstream ss_data;
      for(int cnt=0;cnt<ret;cnt++)
      {
        output.at(2*cnt)=HEX2ASCII(buffer[cnt]>>4);
        output.at(2*cnt+1)=HEX2ASCII(buffer[cnt]&0xf);
      }
      child.put("output",output);
      boost::property_tree::write_json(ss_data, child);

      msg.data=ss_data.str();
      pub->publish(msg);
    }
    ros::spinOnce();
  }
}

static void terminal_in_callback(const std_msgs::String::ConstPtr& msg)
{
  std::stringstream ss(msg->data);
  boost::property_tree::ptree root;
  int c_col,c_row;
  std::string input;
  // Read Data
  try
  {
    boost::property_tree::read_json(ss,root);
  }
  catch(boost::property_tree::ptree_error & e){}
  try
  {
    c_col=root.get<int>("c_col");
    c_row=root.get<int>("c_row");
    setWindowSize(c_col,c_row);
  }
  catch(boost::property_tree::ptree_error & e){}
  try
  {
    input=root.get<std::string>("input");
  }
  catch(boost::property_tree::ptree_error & e){}
  //ROS_INFO("crazy_term_in c_col:%d,c_row:%d,input:\"%s\"",c_col,c_row,input.c_str());
  // transfer
  if(input.length()%2!=0)
  {
    ROS_ERROR("Wrong Length in crazy_term_in");
    return;
  }

  char ctemp = 0;
  int cnt = 0;
  for(std::string::iterator p=input.begin();p!=input.end();p++,cnt++)
  {
    if(ASCII2HEX(*p)>=0)
    {
      if(cnt%2)
      {
        ctemp<<=4;
        ctemp+=ASCII2HEX(*p);
        input.at(cnt/2)=ctemp;
      }else
        ctemp=ASCII2HEX(*p);
    }else
    {
      ROS_ERROR("Wrong Char in crazy_term_in");
      return;
    }
  }
  input.resize(input.length()/2);
  // write to terminal
  if(pty)
    write(pty,input.c_str(),input.length());
}

static void setWindowSize(int width, int height)
{
  if(pty==0)
    return;
  if (width > 0 && height > 0)
  {
    #ifdef TIOCSSIZE
    struct ttysize win;
    ioctl(pty, TIOCGSIZE, &win);
    win.ts_lines = height;
    win.ts_cols  = width;
    ioctl(pty, TIOCSSIZE, &win);
    #endif
    #ifdef TIOCGWINSZ
    struct winsize win;
    ioctl(pty, TIOCGWINSZ, &win);
    win.ws_row   = height;
    win.ws_col   = width;
    ioctl(pty, TIOCSWINSZ, &win);
    #endif
  }
}