#include <iostream>
#include <sony_eye_node/nodeCameras.h>


using namespace std;

int main(int argc, char **argv)
{
   //char fileName[20] = "nodeCameras";
 	//char *pArgumentList[4];
	//pArgumentList[0] = NULL; //fileName;	//argv[0] is the filename.
	//pArgumentList[1] = NULL;	// argv[argc] is a null pointer.
	//int i = 0;
	//ros::init(i,pArgumentList, "nodeCameras");	// this is typcially called from within a main(int argc, char **argv) routine, so we have to send the correct arguments
                                 // to do: pick up the remappings from the command line and use them here
   ros::init(argc, argv, "nodeCameras");
   ros::NodeHandle NodeROS;
   ros::Rate loop_rate(5);

    nodeCameras MyCameras(&NodeROS);

    while ((!MyCameras.GetAllDone()) && NodeROS.ok())
    {
       ros::spinOnce();
       loop_rate.sleep();
       wxYield();
    }
    return 0;
}
