#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "ARDrone.h"
#include <termios.h>

int getch() {
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering      
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_cmd");

    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    ARDrone *drone = new ARDrone();

    while (ros::ok())
    {
      char c = getch();

      switch(c) {
        case 't' : {
         drone->takeOff();
         break;
       }
       case 'l' : {
         drone->land();
         break;
       }
       case 'r' : {
        drone->reset();
        break;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete drone;

  return 0;
}
