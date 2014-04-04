#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "ARDrone.h"
#include <termios.h>

static struct termios oldt, newt;

void setNonBlockingGetChar() {
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~ICANON;
    newt.c_lflag &= ~ECHO;
    newt.c_lflag &= ~ISIG;
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_cmd");
    setNonBlockingGetChar();

    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    ARDrone *drone = new ARDrone();

    while (ros::ok())
    {
      char c = getchar();

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
      case 'w' : {
        drone->moveForward();
        break;
      }
      case 's' : {
        drone->moveBackward();
        break;
      }
      case 'a' : {
        drone->moveLeft();
        break;
      }
      case 'd' : {
        drone->moveRight();
        break;
      }
      case 'u' : {
        drone->moveUp();
        break;
      }
      case 'j' : {
        drone->moveDown();
        break;
      }
      case 'e' : {
        drone->rotateRight();
        break;
      }
      case 'q' : {
        drone->rotateLeft();
        break;
      }
      case -1: {
        drone->hover();
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  delete drone;

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings

  return 0;
}
