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

static void terminal_out_callback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terminal_testing");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("crazy_term_out", 1000, terminal_out_callback);
  ros::spin();
}

static void terminal_out_callback(const std_msgs::String::ConstPtr& msg)
{
  std::stringstream ss(msg->data);
  boost::property_tree::ptree root;
  std::string input;
  // Read Data
  try
  {
    boost::property_tree::read_json(ss,root);
  }
  catch(boost::property_tree::ptree_error & e){}
  try
  {
    input=root.get<std::string>("output");
  }
  catch(boost::property_tree::ptree_error & e){}
  // transfer
  if(input.length()%2!=0)
  {
    ROS_ERROR("Wrong Length in crazy_term_out");
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
      ROS_ERROR("Wrong Char in crazy_term_out");
      return;
    }
  }
  input.resize(input.length()/2);
  printf("%s",input.c_str());
  fflush(stdout);
}
