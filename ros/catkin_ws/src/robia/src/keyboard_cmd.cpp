#include <termios.h>

#include "ros/ros.h"
#include "ros/time.h"

#include "std_msgs/Empty.h"

#include "ARDrone.h"


char nonBlockingGetCh() {
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~ICANON;
    newt.c_lflag &= ~ECHO;
    newt.c_lflag &= ~ISIG;
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;

    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
    char c = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // apply new settings

    return c;
  }

  int main(int argc, char **argv)
  {
    const int FREQ = 100;
    ros::init(argc, argv, "keyboard_cmd");

    ros::NodeHandle n;

    ros::Rate loop_rate(FREQ);

    ARDrone *drone = new ARDrone();
    int idleCount = -1;

    while (ros::ok())
    {
      char c = nonBlockingGetCh();

      switch(c) {
        case 't' : {
         drone->takeOff();
         idleCount = 0;
         break;
       }
       case 'l' : {
         drone->land();
         idleCount = -1; 
         break;
       }
       case 'r' : {
        drone->reset();
        idleCount = -1; 
        break;
      }
      case 'w' : {
        drone->moveForward();
        idleCount = 0; 
        break;
      }
      case 's' : {
        drone->moveBackward();
        idleCount = 0; 
        break;
      }
      case 'a' : {
        drone->moveLeft();
        idleCount = 0; 
        break;
      }
      case 'd' : {
        drone->moveRight();
        idleCount = 0; 
        break;
      }
      case 'u' : {
        drone->moveUp();
        idleCount = 0; 
        break;
      }
      case 'j' : {
        drone->moveDown();
        idleCount = 0; 
        break;
      }
      case 'e' : {
        drone->rotateRight();
        idleCount = 0; 
        break;
      }
      case 'q' : {
        drone->rotateLeft();
        idleCount = 0; 
        break;
      }
      case -1: {
        if (idleCount >= 0) {
          ++idleCount;
        }
        // we count 1sec and we do one hover
        // we wait for next action and count for next hover
        if (idleCount >= FREQ/2){
          drone->hover();
          idleCount = -1; 
        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  delete drone;

  return 0;
}
