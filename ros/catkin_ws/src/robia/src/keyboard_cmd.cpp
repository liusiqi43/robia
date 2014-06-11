#include <termios.h>

#include "ros/ros.h"
#include "ros/time.h"

#include "std_msgs/Empty.h"

#include "ARDrone.h"

#include <image_transport/image_transport.h>

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


    //define different keyboard_commandes
    while (ros::ok())
    {
      char c = nonBlockingGetCh();

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
      case 'z' : {
        drone->setMoveForward();
        break;
      }
      case 's' : {
        drone->setMoveBackward();
        break;
      }
      case 'q' : {
        drone->setMoveLeft();
        break;
      }
      case 'd' : {
        drone->setMoveRight();
        break;
      }
      case 'u' : {
        drone->setMoveUp();
        break;
      }
      case 'h' : {
        drone->hover();
        break;
      }
      case 'j' : {
        drone->setMoveDown();
        break;
      }
      case 'e' : {
        drone->setRotateRight();
        break;
      }
      case 'a' :
        drone->setRotateLeft();
        break;
    }

    drone->commit();

    ros::spinOnce();
    //fait attendre pour voir une frequence de 100
    loop_rate.sleep();
  }

  delete drone;

  return 0;
}
