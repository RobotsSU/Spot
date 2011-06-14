#ifndef ENUMS_H
#define ENUMS_H

// this enums.h is for cool hand luke.  there are changes to the camera parameters compared to neel.

// this section is for ROS nodes, if you change them here, change them there too!
#define SHUT_DOWN_NODE _T("Quit")
#define DEFAULT_PERCENT_SPEED 80
#define DEFAULT_BALANCE_SPEED 30
#define DIRECTION_STOP 0
#define DIRECTION_RIGHT 1
#define DIRECTION_LEFT 2
#define DIRECTION_FORWARD 3
#define DIRECTION_REVERSE 4

#define SET_REFERENCE       0x01
#define REQUEST_DATA        0x02
#define TOOK_DATA           0x03
#define SET_DUTY_CYCLE      0x04

#define ZERO_SPEED_DUTY_CYCLE 48    // with this duty cycle, the motors do not move
#define MAX_DUTY_CYCLE 60          // produces max speed for right and forward directions
#define MIN_DUTY_CYCLE 35           // produces max speed for left and reverse directions

// this section is for the robot arm
#define ROBOT_ARM_COMMAND _T("Change RobotArm")
#define SHOULDER 0            // direction 0 is pitch up
#define ELBOW_PITCH 1         // direction 0 is pitch up
#define ELBOW_ROLL 2          // direction 0 is clockwise
#define WRIST 3               // direction 0 is clockwise
#define GRIP 4                // direction 0 is close
//#define GRIP_OPEN 5         // always open, regardless of direction, redundant command
#define PITCH_UP  0
#define PITCH_DOWN 1
#define ROLL_CLOCKWISE 0
#define ROLL_COUNTERCLOCKWISE 1
#define CLOSE_GRIP 0
#define OPEN_GRIP 1
#define STOP_ROBOT_ARM_MOTOR 2
#define STOP_ROBOT_ARM_ALL_MOTORS 3

// vocies
#define DEFAULT_SPEAKER_NAME _T("David")
#define DEFAULT_SPEAKER_NUMBER 0
#define DAVID 0
#define DIANE 1
#define ALLISON 2

// general
#define DEFAULT_SERIAL_PORT		"COM6"	// COM6 for fujitsu  COM9 for thinkpad
#define MISSLE_LAUNCH_SOCKET_PORT_NUMBER 12887   // post number to use for sockets
#define SPEECH_SOCKET_PORT_NUMBER 12888
#define X10_SOCKET_PORT_NUMBER 12886
#define X10_DEVICE 2
#define MISSLE_LAUNCHER_DEVICE 1
#define SPEECH_DEVICE 0   // tells routines what device is connected over the socket
#define COMMAND_LENGTH	40	// number of characters in the command strings
#define COMMAND_PREFIX_LENGTH   	1	// number of characters in the command prefixes
#define ATTENTION_CHAR	'A'
#define EVENT_CHARACTER 'X'  //used to trigger WaitCommEvent so we know when the end of an input string
    		// on the serial port has been reached and can go ahead and read the string
    		// without worrying that there is more input to follow
#define MAX_SV_UPDATE_TIME	10 // time (seconds) until we must ask for state vector update
#define SV_UPDATE_INTERVAL	2  // minimum time (seconds) between state vector updates (so they don't bog down the system)
#define GENERAL_TIMEOUT	20	// timeout for waiting for stuff like state parameters to return
#define WXTHREAD_PRIORITY	100
#define READY_TO_RECIEVE_STRING	  "SendCmd"
#define MAP_ROWS	150	// for evidence grid map, created in Memory.h
#define MAP_COLS	150
#define GRID_BOX_LENGTH	10 // in cm per pixel defines the lenthg scale in the evidence grid image
                        // should divide evenly into 10 so that automatic room size adjustments occur correctly
                        // see  Memory::LoadHouseMap() for details

#define CLOSE_ENOUGH_TO_GOAL 30 // used in MoveToCoordinates to decide when to stop

#define ASTAR_GOAL_TURN_PENALTY  10.
#define ASTAR_TURN_FACTOR  5.
#define ASTAR_ALLOW_DIAGONALS  1 // AStarSearch has a deep bug that fails to find a path and jumps all round
            // for some coordinates when diagonals not allowed.  In some cases it crashes the program
            // this will cause a crash:  Row, Col, Angle, GoalRow, GoalCol, GoalAngle = 79,20,47,82,28,0
            // this will cause the last two coordinate sets to be disconnected from the rest of the path:
            // Row, Col, Angle, GoalRow, GoalCol, GoalAngle = 75, 20, 47, 82, 28, 0

#define ASTAR_MAX_NUM_SEARCH_NODES 1000

#define VERTICAL_DOORWAY 0
#define HORIZONTAL_DOORWAY 1
#define NO_ANGLE_SPECIFIED -1001    // used for MoveWithinRoom if we don't need to make a specific final turn
#define UPSTAIRS_HALL_TARGET_X_COORD	49
#define UPSTAIRS_HALL_TARGET_Y_COORD 11
#define MAX_NUM_DOORWAYS   20
#define MAX_NUM_DOORWAY_CONNECTIONS    20  // maximum number of connections that a navigation DOORWAY can have
#define MAX_NUM_DOORWAYS_PER_TARGET 20 // maximum number of doorways that a single target can have
#define CLEARANCE	10	// closest distance (cm) to deliberatly approach obstacle
#define NUMBER_OF_VIEWS_FOR_OPEN_AREA 12 // number of measurements taken around 360 degrees
	// to determine most open direction.  Note that 360/NUMBER_OF_VIEWS_FOR_OPEN_AREA should
	// be an integer and also a multiple of DEGREES_PER_TRANSITION
	// to get an integer number of transitions per turn
	// which will provide an accurate number of degrees per turn, yielding a
	// 360 degree view and an accurate turn to the most open area
#define MOD_CHANGE_IMAGE_FILENAME 60  // we will save only about one image per 60 taken (so, about 1 per minute)
#define TARGET_HEIGHT 17.3  // TARGET_HEIGHT typically 18 cm  (7 in.)
#define TARGET_WIDTH 10.5		// TARGET_WIDTH typically 10 cm (4 in.)
#define TARGET_AREA 146	// TARGET_AREA typcially 130 sq cm  (20 sq in.)
#define CIRCLE_TARGET_RADIUS 2 // circle target for battery docking

#define NUMBER_OF_CAMERAS 2
#define FIRST_CAMERA_DEVICE_NUMBER 0	// when there are multiple video devices attached, the cameras are usually not the first devices
#define FORWARD_CAMERA 0
#define UPWARD_CAMERA 1
#define FORWARD_CAMERA_MAX_X 640
#define FORWARD_CAMERA_MAX_Y 480
#define UPWARD_CAMERA_MAX_X 640
#define UPWARD_CAMERA_MAX_Y 480
#define CAMERA_MAX_X 640
#define CAMERA_MAX_Y 480
#define FORWARD_CAMERA_ZOOM 1.0  // CAMERA_ZOOM typically 240% (2.4)
#define FORWARD_CAMERA_DISTANCE_CALIBRATION 260  // for Logitech Fusion webcam

#define UPWARD_CAMERA_ZOOM 1.0
#define UPWARD_CAMERA_PIXEL_PER_CM 2.74 //1.47 commented out the values for neel.
#define UPWARD_CAMERA_X 0 // 19.5
#define UPWARD_CAMERA_Y 0 // -4.8
#define UPWARD_CAMERA_ANGLE 0 //-14
#define UPWARD_CAMERA_ROTATION_ANGLE 0 // -3  // how much the camera is rotated with respect to the X,Y axis of the robot
#define MAX_PIXEL_VALUE 255
#define IS_OBSTACLE     70    //if the evidence grid is below this we assume it's an obstacle
#define OPEN_SPACE_NEEDED_TO_TRY_MOVING_PAST_OBSTACLE 30
#define MAX_NUM_LASER_MAPPING_POINTS 10 // maximum number of navigation points that we will specify around a laser nav target
#define LASER_TARGET_SEMI_LONG_AXIS_LENGTH_PIXELS 50
#define LASER_TARGET_RANGE_CM_X 80
#define LASER_TARGET_RANGE_CM_Y 50  //((UPWARD_CAMERA_MAX_Y / 2) - LASER_TARGET_SEMI_LONG_AXIS_LENGTH_PIXELS) / UPWARD_CAMERA_PIXEL_PER_CM
            // this is how far away we can be from a laser target and still see all of it

#define CLOSEST_APPROACH_TO_NAV_TARGET 40
#define NAV_TARGET_APPROACH_CUSHION 10
#define MAX_MOVE_IN_TOWARD_TARGET 40  // don't move too far in one step or might lose view of the target
#define DOCKING_DISTANCE_FROM_CIRCLE_TARGET 5
// following calibrations done with
//  calibration = (known zoom * known distance)/(known pixels/known target dimension)

//#define CAMERA_HEIGHT_WIDTH_DISTANCE_CALIBRATION 283  //  to break out area calib from linear dimensions
//#define CAMERA_AREA_DISTANCE_CALIBRATION 220 //
// robot specific
#define LCD_WIDTH	20 // number of characters across LCD screen
#define WHEEL_DIAMETER 15 // in cm



#define CM_TO_MOVE_CALIBRATION  0.96  // 0.95    // 1.12 makes up for having to use an integer value for cm per transisiton in uprocessor
            // we have a 6" wheel and 16 transitions per rotation, which is 3.36 cm per transition but we use
            // 3 cm per transition in the microprocessor, so this makes up the difference.
            // In practice, there is some variation, as short distances seem to come up a greater percentage shorter than the long distances
            // this suggests a constant as well as a multiplicative bias but it is hard to fix because adding a constant to REALLY short distances
            // e.g., 3 cm when you are trying to gently nudge into a position is a bad thing to do

#define HARD_FLOOR_MOVE_CALIBRATION 1.0
#define HARD_FLOOR_ADDED_CM 0
#define HEAVY_CARPET_MOVE_CALIBRATION 1.1
#define HEAVY_CARPET_ADDED_CM 5

#define DEGREES_TO_TURN_CALIBRATION 0.93   // 0.93 makes up for having to use an integer value for cm per transisiton in uprocessor
            // there are actually 8.5 degrees per transition and we use 9, so this factor makes up the difference
#define HARD_FLOOR_TURN_CALIBRATION 1.0
#define HARD_FLOOR_ADDED_DEGREES  -4
#define HEAVY_CARPET_TURN_CALIBRATION 1.2
#define HEAVY_CARPET_ADDED_DEGREES 9

#define MIN_WALL_LENGTH_TO_FOLLOW 250  // don't bother wall following for short distances
#define DEFAULT_DISTANCE_TO_KEEP_FROM_WALL 50


#define ROBOT_RADIUS 20  // in cm
#define DEGREES_PER_TRANSITION	9
#define CM_PER_TRANSITION	3
#define DEGREES_PER_SERVO_STEP 9  // number of degrees in each step of the servo sweep
#define SERVO_CENTER_POSITION 12  // index number for centered servo
#define MAX_US_DISTANCE	250	// maximum cm that the US can detect-- no detection will default to this distance of absent objects
            // note that it is OK for this to be shorter than the same parameter in the microprocessor routine, but it should not
            // be longer, since if the microprocessor reports 250 cm, we see that as longer than our max, we record it as our max
            // but if the longest the microprocessor reports is shorter than the max here, then we will always think an obstacle
            // was actually seen at the reported distance
#define MAX_MAPPING_US_DISTANCE 150  // this is the maximum distance used to see an obstacle when using the servo ultrasound to map local obstacles
                                    // this also determines how far away from the robot we extend the "no obstacles seen" line
#define ASSUMED_DEPTH_OF_OBJECTS_DETECTED_BY_US 30
#define DISTANCE_TO_BACKUP_ON_CONTACT	15  // cm to back away after contact
#define ROTATION_ON_CONTACT	45 // degrees to turn away after backing out of contact

#define BATTERY_DISCHARGED_VALUE 550
#define BATTERY_PERCENT_CHARGE_PER_AtoD_VALUE 2
#define BATTERY_LOW_PERCENT 40
#define BATTERY_CRITICAL_PERCENT 30
#define BATTERY_CHARGING_PERCENT 104
#define BATTERY_CHARGER_ROOM_NUMBER 6
#define BATTERY_CHARGER_X 450
#define BATTERY_CHARGER_Y 375
#define BATTERY_CHARGER_ANGLE 90
#define MIN_BATTERY_CHARGE_HOURS 2
#define BATTERY_CHARGER_TARGET_NUMBER 13
#define BATTERY_CHARGER_LASER_TARGET_NUMBER 20
#define LASER_TARGETS_STARTING_NUMBER   BATTERY_CHARGER_LASER_TARGET_NUMBER // sequentially number the laser targets starting with the battery charger one

// basic commands
// if you add something here, be sure to also add it to
// brainstem's InitVerbalParserArrays if it is a speech command
#define STOP			_T("0")
#define STOP_CHAR		'0'
#define GO  			_T("1")
#define GO_CHAR			'1'
#define SET_FORWARD 	_T("2")
#define SET_BACKWARD 	_T("3")
#define SET_TURN_CW		_T("4")
#define SET_TURN_CCW	 _T("5")
#define SCAN_IR			_T("6")
#define CHECK_DISTANCE	_T("7")
#define FOLLOW_ME		_T("8")
#define TRACK_IR		_T("9")
#define READY_FOR_STATE	_T("R")
#define SEND_STATE 		_T("S")
#define VECTOR_SENT		_T("V")
#define MAP_OBSTACLES	_T("M")
#define RESET_ALERT		_T("A")
#define IDLE_STATE			_T("I")  // ************note change here from IDLE to IDLE_STATE due to conflict with node.h**************************
#define MAX_NUM_IDLES	3



// contact directions
#define NO_CONTACT	0
#define FRONT		10
#define REAR		100
#define LEFT		3
#define RIGHT		4
#define LEFT_FRONT	13
#define RIGHT_FRONT	14
#define LEFT_REAR	103
#define RIGHT_REAR	104
#define ALERT_CONTACT 109  // if contact happened, contact direction code will be > 0 and < 109

#define ALERT_HELP 1000

// help messages
#define NO_HELP_NEEDED _T("N")
#define RESET_OCCURRED _T("R")
#define TURN_HELP_NEEDED _T("T")
#define MOVE_HELP_NEEDED _T("M")
#define SERVO_HELP_NEEDED _T("S")
#define WALL_FOLLOW_HELP_NEEDED _T("W")
#define CONTACT_HELP_NEEDED _T("C")
#define KEYPAD_PRESSED _T("K")
#define STALLED_MOTOR _T("L")
#define BATTERY_LOW _T("B")
#define ULTRASOUND_OBSTACLE _T("U")



// TALK COMMANDS  (just say something)
#define TALK_COMMAND_PREFIX		_T("T")
#define TELL_JOKE		1
#define ORANGE_WHO		2
#define WHOS_THERE		3
#define NOT_NOW			4
#define HEY_THERE		5
#define STOP_LISTENING	6
#define MEET_MY_FRIEND	7


// Voice commands (do something commanded by voice)
#define VOICE_COMMAND_PREFIX	_T("V")

// brainstem commands
#define BRAINSTEM_COMMAND_PREFIX	_T("B")
#define BRAINSTEM_COMMAND_CHAR	'B'
#define MOVE					_T("BM")
#define TURN					_T("BT")
#define ALIGN_PERPENDICULAR	_T("BP")
#define GO_TO_WALL    _T("BG")
#define WALL_FOLLOW		_T("BW")
#define WALL_FOLLOW_RIGHT_FOREVER _T("BWR0_")
#define WALL_FOLLOW_LEFT_FOREVER _T("BWL0_")
#define SET_SPEED _T("BS")
#define SET_ROTATION        _T("BR")
#define PARAMETER_SEPARATOR _T("_") // separates parameters in comm string when more than 1 parameter passed


//enum SerialCommand {
//STOP = 0, GO = 1, SET_FORWARD = 2 , SET_BACKWARD = 3, SET_TURN_CW //= 4,
// SET_TURN_CCW = 5, SCAN_IR = 6, CHECK_DISTANCE = 7, TRACK_PERSON //= //8,
 //TRACK_IR = 90};
 // next enum will be 91, then 92... up to 98, then next will be 990, 991, ....
 //  see robotmain.c in the embedded gnu files for details.

#endif

