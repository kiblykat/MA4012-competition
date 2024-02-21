#pragma config(Motor,  port2,           servoFront,  tmotorServoStandard, openLoop, driveLeft)    //servo_swinger is servo_front
#pragma config(Motor,  port5,           servoBack,  tmotorServoStandard, openLoop, driveRight)
#pragma config(Sensor, port4,  rearBumper1,         sensorAnalog)
#pragma config(Sensor, port6,  rearBumper2,         sensorAnalog)
#pragma config(Sensor, port8,  masterSwitch,         sensorAnalog) //TODO change pins
#pragma config(Sensor, in2,    B,              sensorAnalog)
#pragma config(Sensor, in3,    C,              sensorAnalog)
#pragma config(Sensor, in1,    A,              sensorAnalog)
#pragma config(Sensor, in5,    CheckBall, sensorAnalog)
#pragma config(Motor,  port8,           rightWheel,    tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port9,           leftWheel,     tmotorNormal, openLoop)
#pragma config(Sensor, dgtl9,  reflectiveFL,   sensorDigitalIn)
#pragma config(Sensor, dgtl10,  reflectiveFR,   sensorDigitalIn)
#pragma config(Sensor, dgtl11,  reflectiveBL,   sensorDigitalIn)
#pragma config(Sensor, dgtl12,  reflectiveBR,   sensorDigitalIn)
#pragma config(Sensor, dgtl7, compassSupply,  sensorDigitalOut)
#pragma config(Sensor, dgtl3,  compassWest,    sensorDigitalIn)
#pragma config(Sensor, dgtl4,  compassSouth,   sensorDigitalIn)
#pragma config(Sensor, dgtl5, compassEast,    sensorDigitalIn)
#pragma config(Sensor, dgtl6, compassNorth,   sensorDigitalIn)

#define FORWARD 1
#define BACKWARD -1

#define TIME_1_F 275       //turns front gate 180
#define TIME_2_F 2200       //turns 4 revolutions
#define TIME_1_B 370	//turns back gate 180

#define NORTH 0
#define NORTHEAST 1
#define EAST 2
#define SOUTHEAST 3
#define SOUTH 4
#define SOUTHWEST 5
#define WEST 6
#define NORTHWEST 7

#define STATE_MOVE_TO_BALL 1
#define STATE_SEARCH_BALL 2
#define STATE_GO_TO_COLLECTION_PLACE 3

/*-----------------------------------global variables------------------------------------*/
bool ballClose=false;				//specify whether ball is close enough to be grabbed or not
bool ballDetected=false;
bool robotDetected=false;
int batLevel;
float distanceB;
bool ballCollected=false; 				//specify whether ball is already scoped or not for debugging only
bool rearBumperPressed=false;
bool leftBumperPressed=false;
bool rightBumperPressed=false;
int global_orientation;
int codeState=0; 								//variable to tell where the code line is at

bool already_in_collection_place=false;
int debugVar=0;
int a=0;
void align_orientation_with_collection();
void start_move();
/*-----------------------------------end of global variables------------------------------------*/

bool is_limit_ball();
void read_orientation();
void servo_to_angle(int port, int direction, int time);
bool catch_ball();
void move(int direction, int speedMode);
void rotate(int direction, int speedMode);
void reset_servo();
bool go_to_collection_place();
bool move_to_ball();
bool search_ball();
task competition();
task emergency_stop();
void wait_for_on();
bool line_detection();
/*-----------------------------------end of global variables----------------------------*/

/* Use the upper sensor to detect whether ball has been successfully scoped on the grabber or not
@param return true if yes, false if no
*/

bool is_limit_ball()
{
    if(SensorValue[CheckBall] < 100) //iz_checkBall does not return 0 or 1 since its connected to Analog
    {
        ballCollected = 1; //debug
        return true;
    }
    else
    {
        ballCollected = 0; //debug
        return false;
    }
}

// void servo_to_angle(int port, int angle_destination, int delay_speed)
// {
// 	while(motor[port]>angle_destination)
// 	{
// 		motor[port]=motor[port]-1;
// 		wait1Msec(delay_speed);
// 	}
// 	while(motor[port]<angle_destination)
// 	{
// 		motor[port]=motor[port]+1;
// 		wait1Msec(delay_speed);
// 	}
// }

void servo_to_angle(int port, int direction, int time)
{
	int voltage=60;
	motor[port] = voltage*direction;
	wait1Msec(time);
	motor[port] = 0;
}


/* Catch ball
@param return true if sucessful, false if unsucessful
*/
bool catch_ball()
{
    servo_to_angle(servoFront,BACKWARD,TIME_2_F);   //sweep ball up
    wait1Msec(1000); //wait 2s for ball to hit limit
    if(is_limit_ball()) //check if ball pressing on limit
    {
        return true;
    }
    else
    {
        servo_to_angle(servoFront,FORWARD,TIME_1_F); //release abit incase ball stuck
        return false;
    }
}

bool line_detection()
{
	if(codeState==STATE_GO_TO_COLLECTION_PLACE)
	{
		//do some action about the line
		if (SensorValue[reflectiveFL]==0)
		{
            debugVar=12;
			move(1,0);
			move(-1,2);
			wait1Msec(1000);
			rotate(-1,1); //CW
			wait1Msec(500);
			move(1,1);
			wait1Msec(1000);
			align_orientation_with_collection();
		}
		else if(SensorValue[reflectiveFR]==0)
		{
            debugVar=13;
			move(1,0);
			move(-1,2);
			wait1Msec(1000);
			rotate(1,1);
			wait1Msec(500);
			move(1,2);
			wait1Msec(1000);
			align_orientation_with_collection();
		}
		else if(SensorValue[reflectiveBL]==0)
		{
            debugVar=14;
			move(1,0);

			clearTimer(T2);
			while(time1(T2)<700)
			{
				move(-1,1);
				if(rearBumperPressed)
				{
					clearTimer(T3);
					while(time1[T3] < 1000)
					{
						if(SensorValue[reflectiveBR]==1 && SensorValue[reflectiveBL]==1)
						{
							move(1,2);
						}
						else
						{
							move(1,0);
							already_in_collection_place=true;
							return true;
						}
					}//end of inner while
					align_orientation_with_collection();
					already_in_collection_place=false;
					return false;
				}//end of if bumper detected
			}//end of outer while

			move(1,2);
			wait1Msec(1000);
			rotate(1,1);
			wait1Msec(500);
			move(1,2);
			wait1Msec(1000);
			align_orientation_with_collection();
			already_in_collection_place=false;
			return false;
		}
		else if(SensorValue[reflectiveBR]==0)
		{
            debugVar=15;
			move(1,0);

			clearTimer(T2);
			while(time1(T2)<700)
			{
				move(-1,1);
				if(rearBumperPressed)
				{
					clearTimer(T3);
					while(time1[T3] < 1000)
					{
						if(SensorValue[reflectiveBR]==1 && SensorValue[reflectiveBL]==1)
						{
							move(1,2);
						}
						else
						{
							move(1,0);
							already_in_collection_place=true;
							return true;
						}
					}//end of inner while
					align_orientation_with_collection();
					already_in_collection_place=false;
					return false;
				}//end of if bumper detected
			}//end of outer while

			move(1,2);
			wait1Msec(1000);
			rotate(-1,1);
			wait1Msec(500);
			move(1,2);
			wait1Msec(1000);
			align_orientation_with_collection();
			already_in_collection_place=false;
			return false;
		}
	}
		//do some action about the line
		else if(SensorValue[reflectiveFL]==0 && SensorValue[reflectiveFR]==0) //iz_0 means detected
		{
			move(1,0);
			move(-1,1);
			wait1Msec(500);
			rotate(1,1);		//CCW
			wait1Msec(800);
		}
		else if (SensorValue[reflectiveFL]==0)
		{
			move(1,0);
			move(-1,1);
			wait1Msec(500);
			rotate(-1,1);		//CW
			wait1Msec(800);
		}
		else if(SensorValue[reflectiveFR]==0)
		{
			move(1,0);
			move(-1,1);
			wait1Msec(500);
			rotate(1,1);		//CCW
			wait1Msec(800);
		}
		else if(SensorValue[reflectiveBL]==0 && SensorValue[reflectiveBR]==0)
		{
			move(1,0);
			move(1,2);
			wait1Msec(500);
			rotate(1,1);		//CCW
			wait1Msec(800);
		}
		else if(SensorValue[reflectiveBL]==0)
		{
			move(1,0);
			move(1,2);
			wait1Msec(500);
			rotate(-1,1);		//CW
			wait1Msec(800);
		}
		else if(SensorValue[reflectiveBR]==0)
		{
			move(1,2);
			wait1Msec(500);
			rotate(1,1);		//CCW
			wait1Msec(800);
		}
		return false;
}

task detection_others()
{
	while(true)
    {
        read_orientation();

		if(SensorValue[rearBumper1]<100 || SensorValue[rearBumper2]<100)
		{
			rearBumperPressed=true;
		}
		else
		{
			rearBumperPressed=false;
		}
    }
}

void move(int direction, int speedMode)
{
	int voltageLeft;
	int voltageRight;
	if(speedMode==1)
	{
		voltageLeft=30;
		voltageRight=30;
	}
	else if(speedMode==2)
	{
		voltageLeft=60;
		voltageRight=60;
	}
	else if(speedMode==3)
	{
		voltageLeft=120;
		voltageRight=120;
	}
	else if(speedMode==0)
	{
		voltageLeft=0;
		voltageRight=0;
	}
	motor[rightWheel] = voltageRight*direction;
	motor[leftWheel]  = voltageLeft*direction;

}

void rotate(int direction, int speedMode)
{
	int voltage;
	if(speedMode==1)
	{
		voltage=40;
	}
	else if(speedMode==2)
	{
		voltage=60;
	}
	else if(speedMode==0)
	{
		voltage=0;
	}
	motor[rightWheel] = voltage*direction;
	motor[leftWheel]  = -voltage*direction;
}

task detection()
{
	while(true)
	{
		if(600<SensorValue[B] && SensorValue[B]< 3000) 		//lower sensor detects something (iz_change 500 to 800)
		{
			if (SensorValue[C] >600) 														//upper sensor detects something TODO change value
			{
				ballDetected=true;
				robotDetected=true;
			}
			else																								//upper sensor does not detect anything
			{
				ballDetected=true;
				robotDetected = false;
			}
		}
		else
		{
			ballDetected=false;
			robotDetected=false;
		}
		//If ball is detected, Check if the ball is close enough
		if(ballDetected)
		{
			//distanceB = (10000.0/SensorValue[B]-0.6685)/0.4255;

			if(SensorValue[B] >900) 		//side sensor detects within the range acceptable for grabber
			{
				ballClose =true;	//ball is close enough for the grabber
			}
			else
			{
				ballClose = false; //not close enough
			}
		}
	}//end of while
}//end of task


void read_orientation()
{
	int temp = 8*SensorValue(compassWest)+4*SensorValue(compassSouth)+2*SensorValue(compassEast)+SensorValue(compassNorth);
	switch(temp) {
	case 14:
		global_orientation = NORTH;
		break;
	case 13:
		global_orientation = EAST;
		break;
	case 11:
		global_orientation = SOUTH;
		break;
	case 7:
		global_orientation = WEST;
		break;
	case 12:
		global_orientation = NORTHEAST;
		break;
	case 9:
		global_orientation = SOUTHEAST;
		break;
	case 3:
		global_orientation = SOUTHWEST;
		break;
	case 6:
		global_orientation = NORTHWEST;
		break;
	default:
		global_orientation = -1;
	}
}


void reset_servo()
{
	servo_to_angle(servoBack,FORWARD,TIME_1_B);
	servo_to_angle(servoFront,BACKWARD,TIME_1_F);
}

bool search_ball()
{
	codeState=STATE_SEARCH_BALL; //indicate the code is now at search_ball function

	while(true)
	{
		debugVar=1;
		clearTimer(T1);
		while(time1[T1]<2*600)	//change the value 3000 TODO
		{
			line_detection();//AAA
			if(ballDetected && !robotDetected)
			{
				move(1,0);				//stop			
				return true;
			}
			rotate(1,2);				//rotate CCW
		}

		clearTimer(T1);
		while(time1[T1]<4*600)	//change the value 3000 TODO
		{
			line_detection();//AAA
			if(ballDetected && !robotDetected)
			{
				move(1,0);				//stop				
				return true;
			}
			rotate(-1,2);				//rotate CW
		}
		debugVar=2;

		clearTimer(T1);
		while(time1[T1]<2*600)	//change the value 3000 TODO
		{
			line_detection();//AAA
			if(ballDetected && !robotDetected)
			{
				move(1,0);				//stop				
				return true;
			}
			rotate(1,2);				//rotate CCW
		}
		move(1,0);

		//if ball is not detected during rotation
		clearTimer(T1);
		while(time1[T1]<1000)	//change tehe value 2000 TODO
		{
			move(1,2);
			line_detection();//iz_com
		}
		return false;//note
	}

	return false;
}

/*
@param return true, if ball is right in front of the ball (1,5,8,1) (1,3,11)
*/
bool move_to_ball()
{
	int aaa=0;
	codeState=STATE_MOVE_TO_BALL;							//indicate the code is now at move_to_ball function

		//move towards the ball
		//adjust the speed according to the distance
		if(ballClose)
		{
			debugVar=3;
            rotate(1,1);
            wait1Msec(400);
			move(1,1);											//stop
			wait1Msec(3000);
			move(1,0);
			return true;
		}
		else
		{
			move(1,2);
			wait1Msec(100);
		}

		if(global_orientation==0)
		{
			move(-1,1);
			wait1Msec(1000);
			align_orientation_with_collection();
			start_move();
		}
		return false;
}


void align_orientation_with_collection()
{
	//rotate to align the orientation with the collection place
	if(global_orientation==0||global_orientation==1||global_orientation==2||global_orientation==3)
	{
		while(global_orientation!=4)
		{
			rotate(-1,2);//CW
		}
	}
	else if(global_orientation==5||global_orientation==6||global_orientation==7||global_orientation==4)
	{
		while(global_orientation!=4)
		{
			rotate(1,2);//CCW
		}
	}
	move(1,0);
	rotate(1,1); //iz_rotate CCW
	wait1Msec(500);	//iz_for 500Msec
	move(1,0);
}
/*
*/
bool go_to_collection_place()
{
    debugVar=200;
	already_in_collection_place=false;
	codeState=STATE_GO_TO_COLLECTION_PLACE;			//indicate the code is now at go_to_collection_place function

	align_orientation_with_collection();
	wait1Msec(500);
	//move backward until bumper detected
	while(!already_in_collection_place) //move backward
	{
		line_detection(); //iz_com
		if(already_in_collection_place)
		{
			return true;
		}
		move(-1,3);
	}
	return false;
}

/*
	Search for ball
	@param stop and return true when ball is found
*/


/*
Initial action, move half of the arena
*/
void start_move()
{
	clearTimer(T1);
	while(time1(T1)<3500)//TODO tune me
	{
		move(1,3);
	}
}





/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/
task competition()
{
	startTask(detection);
	startTask(detection_others);

	start_move();	//TODO uncomment me

	while(true)
	{
		if(search_ball())
		{
			debugVar=10;
			if(move_to_ball())
			{
				debugVar=11;
				if(catch_ball())
				{
                    debugVar=100;
					while(!go_to_collection_place())
					{
					}
					servo_to_angle(servoBack,FORWARD,TIME_1_B);
					wait1Msec(1500);
					reset_servo();
					start_move();
				}
			}
		}
	}//end of while true
}

// task emergency_stop()
// {
// 	while(true)
// 	{
// 		if((SensorValue[masterSwitch]<100)&&(a==1))
// 		{
// 			stopTask(competition);
// 			move(1,0);
// 			servo_to_angle(servoFront,BACKWARD,TIME_1_F);
// 			servo_to_angle(servoBack,FORWARD,TIME_1_B);
// 			break;
// 		}
// 	}
// 	wait_for_on();
// }

void wait_for_on()
{
	while((SensorValue(masterSwitch)>200)&&(a==0))
	{
		//wait
	}
	a+=1;
	startTask(competition);
	// startTask(emergency_stop);
}

task main()
{

	wait_for_on();
	while(true)
	{
	}
}//end of main
