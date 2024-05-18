# NTU MA4012 Mechatronics Engineering Design (2022):

C Code logic for competition where rules are to search and collect a tennis ball within a play area, against an opponent.
Code is implemented on a VEX Cortex microcontroller utilizing various sensors and actuators.

Video: https://www.youtube.com/watch?v=dkav24JsHlE&ab_channel=ayampenyet

## 8    Programming logic

### 8.1     Software Architecture

The figure below shows the software flowchart that the robot will undergo during the competition.  The boxes in orange indicate tasks, in which the robot will continuously run throughout the entirety of the competition in separate threads that do not interfere with the main thread. This allows the robot to simultaneously run multiple algorithms in parallel. 

![image](https://github.com/kiblykat/ScrappyBot/assets/52589461/5af85864-4b12-40e3-b141-b54c33320905)

To prevent any conflict of which algorithm will run first, the tasks were only written to update global variables in the program without actuating any physical movements on the robot itself. Physical movements would be performed by functions within the main program to ensure a smooth continuous flow in the robot's movements. The code will initiate two tasks (orange) to run concurrently with the main program:

1.    Bumper/Orientation Detection:  The task detection_others()  continuously checks the orientation of the robot as well as whether the rear bumper was being depressed at any point in time.  This was important in situations where the robot might reverse into an opponent's robot during the delivery phase. Used concurrently with the rear line sensors, the bumper detection was able to ensure that the ball will be released only at the delivery area.
2.    Sensor Detection: The task sensor_detection() continuously checks the two front long distance sensors to detect whether our robot is either facing an opponent or a ball. In brief, if the bottom long-distance sensor detects an obstacle while the top distance sensor does not, it can be concluded that the object detected is a ball. However, if both were detected, then it results in the only possibility is that the object is a robot. To ensure that this algorithm works properly, the height of the bottom sensor had to be precisely positioned to prevent false positives (eg: detecting boundary as ball).

#### 8.1.1   Detection

1)   Limit switch on
2)  Check compass reading and store in variable first_dir (integer variable temp)
3)  Move forward for time tl  (lm)
4)  Check in straight direction:  if obstacle in front of left sensor, then go straight,  align with short range sensor and go to collect stage;  Else if in front of right sensor, align with left sensor, then same logic as above.
5)  Rotate 90 degrees left and 180 degrees right - while running task obstacle_detection (robot or ball)
  a.   If ball, stop rotate and move forward:
  b.   If in rotate_left_stage, use left sensor array to detect ball and move forward until in range of long-range sensor (+-5) then tum theta degrees (map theta to distance) to allow ball to come in front of short-range sensor then use that to get to best distance with        ball to collect (depends on dimensions)
  c.   If in rotate_right_stage,  use right sensor first to detect obstacle,  tum in place to get obstacle in line with left sensor array, detect ball or robot, if ball use same logic as rotate left stage to get to ball collection stage
  d.   Else if robot: continue sweep
  e.   Else nothing:  move forward O. lm,  reset position  first,  then do 270 sweep CW and run algorithm 5a,  setting rotate_right_stage  as true.
        a.    If still nothing, reset position then run program from [4] again.

#### 8.1.2    Collection

6)  Rotate scoop motor for time t2 and wait for t3 milliseconds

  a.    If limit  switch  returns   true  within   t3   ,    then  continue   with   flow  and  set codeState=STATE   GO  TO  COLLECTION
  b.    Else reset scoop motor for time t2, check ball with short range sensor:
  c.    If detected, then move to collection stage d.   Else run program from [4].

#### 8.1.3   Reversing

7)  Check compass direction dir, if dir!=first_dir, then use func rotate_initial to straighten robot:
  a.   In rotate_initial, based on temp value rotate until current temp= initial temp
8)  Move backward until 4 back limit switches pressed and yellow tape sensor(BL and BR) detected.
9)  Rotate back gate for time t4 to release ball into deposit box, reset back gate and run program from [3].

#### 8.2.2    Function Description
![image](https://github.com/kiblykat/ScrappyBot/assets/52589461/ffc6630e-407b-4536-9cc1-b242c88e1352)

Several specific functions which were important to the robot’s overall movements are displayed as follows, with their together with their role. 

| Function/Task Name            | Description           |
| ------------- |:-------------:| 
| void align_orientation_with_collection();            |       Aligns  robot  with  collection   point, rotating the robot until  it faces south based on compass readings |
| void start_move();                                  |                      Moves the robot forward for 3 seconds at high speed. This was done to get the robot to the middle of playing field as fast as possible before searching starts. | 
| void read_orientation();                             |                  Reads the robots current orientation |
| void   servo_to_angle(int   port,   int   direction, int time); | Controls      the      motors      that      are responsible  for collection  and deposit of ball from the robot. |
| bool catch_ball(); | Runs   the   front   collection   motor   to collect the ball into the robot |
| void move(int direction,  int speedMode); | Allows  precise  control  of speed  and direction      during      forward/reverse movement of robot. |
| void rotate(int direction, int speedMode);   |                  Allows  precise  control  of speed  and direction during Clockwise (CW)/Counter Clockwise(CCW) rotation of robot. |
| void reset_servo();                           |                                 Resets  the  collection  and  depositing motors   after   successfully   sconng  a ball. |
| bool go_to_collection_place();                 |                      Robot will continually  reverse until it reaches  the collection  place  with rear bumper  pressed  and  yellow  line detected on rear line sensors. |
| bool search_ball();                             |                               Runs   search  algorithm   until  ball   is detected. Search algorithm includes rotating 90 degrees CCW, 180 degrees CW and then 90 degrees CCW again to face its initial direction. |
| bool move_to_ball();                             |                           After ball is detected, robot will move slowly   towards   the  ball   until   it  is within range of the catcher motors. |
| void wait_for_on();                               |                           After    robot    is   powered    on,    this function allows starting the whole software algorithm with the flick of the master limit switch |
| bool line_detection();                             |                          Detects    the    yellow    tape    at    the boundary   of the  playing   field.   This function is called in a while loop whenever the robot is running a move/rotate function to prevent overshooting the boundary.                            |
| task competition();                                  |                          Main  task  that  starts  other  sub  tasks and mam software algorithm for searching, collection, aligning and deposit of ball. |
| task sensor_detection();                              |                     Continually receives readings from the top and bottom distance sensors to determine if robot or ball detected. |
| task detection_others();                               |                     Continually receives compass readings and rear bumper readings to determine if robot has reached deposit area or hit another robot. |

