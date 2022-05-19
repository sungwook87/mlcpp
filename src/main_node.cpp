#include "mlcpp_main.h"
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}




int main(int argc, char **argv){

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::init(argc, argv, "mlcpp_class_node");
    ros::NodeHandle n("~");
    mlcpp_class mlcpp_(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}