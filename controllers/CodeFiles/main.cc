/*
EN2532-ROBOT DESIGN AND COMPETITION

Group:- TIKIRI MOLE
Group Members:- S.SANJITH
                M.N.M. NUSHATH
                K. KAJHANAN

MOST OF THE CODE USE COVENTION LEFT TO RIGH
*/
//-------------------------------------------------------------BEGIN-----------------------------------------------------------//
/* Including Libraries */
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Compass.hpp>

/* Specifying NamesSpaces */
using namespace webots;
using namespace std;

//------------------------------------------------------------------------------------------------------------------------------//
/* Variable Definitions */
#define TIME_STEP 16
#define MAX_SPEED 10
#define CENTER_DISTANCE 17
#define LEFT_CENTER_DISTANCE 17.1
#define WALL_LIMIT 20
#define LEFT_DISTANCE 10
#define DELAY_ARM 2
#define ARM_DISTANCE_FORWARD 9.5
#define ARM_DISTANCE_BACKWARD 7.6
#define CYLINDER_TUNE_ANGLE 5.75
#define ARM_BASE_DELAY 3
#define BALL_SELECTION BLUE
#define CYLINDER_HOLE_ALIGN -1.15
#define CUBE_HOLE_ALIGN -4.4
#define HOLE_DEPTH 3
#define ARM_DISTANCE_BALL 10

#define GAP 5

/* NAMES */
string motorNames[8] = {"left_motor", "right_motor", "front_arm_motor", "back_arm_motor", "fl_slider", "fr_slider", "bl_slider", "br_slider"};
string irNames[11] = {"ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7", "ir8"};
string psNames[8] = {"ps_left_motor", "ps_right_motor", "farm_base_position", "fl_position", "fr_position", "barm_base_position", "bl_position", "br_position"};
string camNames[3] = {"left_camera", "right_camera", "front_camera"};
string wirNames[4] = {"front_sonar", "left_sonar", "right_sonar", "right_sonar2"};
string laserNames = "right_laser";
string colorNames[7] = {"RED", "BLUE", "MAGENTA", "BLACK", "YELLOW", "CYAN", "WHITE"};
string compassNames = {"compass"};
string obIRNames[9] = {"front_ir", "front_ir_left", "front_ir_right", "back_ir", "front_object", "left_object", "right_object", "back_object", "front_ir_sharp"};
string holeObject[3] = {"CYLINDER_CIRCLE", "CYLINDER_CURVED", "CUBE"};

//------------------------------------------------------------------------------------------------------------------------------//
/* NUMBER VARIABLES */
enum rsMotors
{
    LEFT = 0,
    RIGHT,
    FRONT_ARM_BASE,
    BACK_ARM_BASE,
    FRONT_ARM_LEFT,
    FRONT_ARM_RIGHT,
    BACK_ARM_LEFT,
    BACK_ARM_RIGHT
};

enum pSensors
{
    psLeftMotor = 0,
    psRightMotor,
    psFBase,
    psFLSlider,
    psFRSlider,
    psBBase,
    psBLSlider,
    psBRSlider
};

enum aStates
{
    WITH_OBJECT = 0,
    WITH_OBJECT_F,
    WITHOUT_OBJECT,
    GUARD_OBJECT,
    PUSH_OBJECT
};

enum aCameras
{
    LEFT_CAMERA = 0,
    RIGHT_CAMERA,
    FRONT_CAMERA
};

enum wallSonars
{
    FRONT_WALL = 0,
    LEFT_WALL,
    RIGHT_WALL,
    RIGHT_WALL2,
};

enum laSensors
{
    RIGHT_LAS = 0
};

enum Colors
{
    RED = 0,
    BLUE,
    MAGENTA,
    BLACK,
    YELLOW,
    CYAN,
    WHITE
};

enum obIRSensors
{
    FRONT_IR = 0,
    LEFT_ALIGN_IR,
    RIGHT_ALIGN_IR,
    BACK_IR,
    FRONT_OBJECT,
    LEFT_OBJECT,
    RIGHT_OBJECT,
    BACK_OBJECT,
    FRONT_IR_SHARP
};

enum holeObjects
{
    CYLINDER_CIRCLE = 0,
    CYLINDER_CURVED,
    CUBE
};

enum directions
{
    NORTH = 0,
    EAST,
    WEST,
    SOUTH
};

enum mainTask
{
    LINE_FOLLOWING = 0,
    MAZE_SOLVE,
    HOLE_TASK,
    BALL_PICK,
    REACH_END,
    KICK_BALL
};
//------------------------------------------------------------------------------------------------------------------------------//
/* Defining Variables */
// Task variable
mainTask CURRENT_TASK = LINE_FOLLOWING;
/* Tuning parameters regarding robot body */
const float wheel_radius = 0.033, robot_width = 0.21, turn90_angle = (3.14 * robot_width) / (4 * wheel_radius);
// Motor Variables
static double left_speed = 0;
static double right_speed = 0;
static float base_speed_fast = 9;
static float base_speed = 6;
static float base_speed_slow = 4;

// variable to keep track of object to lift
/*
-1 : correct ball have been picked,
0 : picking objects tasks have been completed,
1: box detected while cylinder in front arm ,
2 : start state ,
3 : route to back arm,
4: back arm have the object}
*/
static short int object_state = 2;

// Error variables functions : {Line Following}
static float bright_scale_factor = 1;
static float I = 0;
static float last_error = 0;
static double maze_north = 180;

// color variable
Colors BALL_COLOR = BALL_SELECTION;
//------------------------------------------------------------------------------------------------------------------------------//
/* Intiating Attributes */

Robot *robot = new Robot();
/* SENSORS */
DistanceSensor *irPanel[8]; // IR Panel values from left to right
PositionSensor *psSensors[8];
DistanceSensor *wallSensors[4];
DistanceSensor *laserSensors[1];
Camera *cams[3];
Compass *compass;
DistanceSensor *obSensors[9];
InertialUnit *i_unit;
/* ACTUATORS */
Motor *motors[8]; // Motor[0] is the left motor, Motor[1] is the right motor
//------------------------------------------------------------------------------------------------------------------------------//
/* FUNCTIONS */
void TASK_MANAGER();

// SENSOR MODELLING FUNCTIONS
float OBJECT_IR_READ(obIRSensors ir_sensor);
float LASER_MAP(laSensors laser);
float SONAR_MAP(wallSonars sonar);

// PROCESSING DATA RELATED FUNCTIONS
Colors COLOR_DETECTION(aCameras color_sensor);
double READ_COMPASS();

// MOTION RELATED FUNCTIONS
int ERROR_CALC();
void LINE_FOLLOW(bool dotted = false);
void WALL_FOLLOW();
void ALIGN_TO_DIR(directions destination = NORTH);
void TURN_90(short int dir = 0);
void TURN_ANGLE(float angle = 45, short int dir = 0);
void GO_FORWARD(float distance = CENTER_DISTANCE, short int dir = 0, float deceleration = 0);

// ARM RELATED FUNCTIONS
void BASE_ARM_SWAP(short int f_position = 0, short int c_arm = 0);
void SLIDER_ARM_MOVEMENT(short int s_position = 0, short int c_arm = 0);

// OBJECT PICKING-PLACING FUNCTIONS
void ALIGN_TO_OBJECT(float distance = 15);
void OBJECT_CONFIRMATION();
void PICK_OBJECT();
holeObjects IDENTIFY_OBJECT();
void ALIGN_TO_CYLINDER();
bool DETECT_OBJECT();
bool DETECT_OBJECT_ULTRA(int step = 0.5);
bool ROUND_SEARCH();
void DETECT_BALL();

// CORRECTION FUNCTIONS
float APPROACH_VALUE(float c_value, float d_value, float a_step = 0.002);
bool IN_RANGE(float a, float b);
void MINOR_CORRECTION_WALL(float distance = LEFT_DISTANCE);
void FORWARD_CORRECTION(float dist = 21);

// ROBOT ESSENTIAL FUNCTIONS
double LIMIT(double &val);
void DELAY(double t = 1000);
void SET_VELOCITY();
void STOP_ROBOT();
void CLEAR_VARIABLES();

// FUN COMMENTS
void MAKE_FUN1(holeObjects var);

//------------------------------------------------------------------------------------------------------------------------------//
int main(int argc, char **argv)
{
    DELAY(500);

    // motors
    for (int i = 0; i < 8; i++)
    {
        motors[i] = robot->getMotor(motorNames[i]);
    }

    motors[LEFT]->setPosition(INFINITY);
    motors[LEFT]->setVelocity(0);
    motors[RIGHT]->setPosition(INFINITY);
    motors[RIGHT]->setVelocity(0);

    motors[FRONT_ARM_BASE]->setVelocity(INFINITY);
    motors[BACK_ARM_BASE]->setVelocity(INFINITY);
    motors[FRONT_ARM_LEFT]->setVelocity(INFINITY);
    motors[FRONT_ARM_RIGHT]->setVelocity(INFINITY);
    motors[BACK_ARM_LEFT]->setVelocity(INFINITY);
    motors[BACK_ARM_RIGHT]->setVelocity(INFINITY);

    motors[FRONT_ARM_BASE]->setPosition(0);
    motors[BACK_ARM_BASE]->setPosition(-0.2);
    motors[FRONT_ARM_LEFT]->setPosition(0.025);
    motors[FRONT_ARM_RIGHT]->setPosition(-0.025);
    motors[BACK_ARM_LEFT]->setPosition(0.025);
    motors[BACK_ARM_RIGHT]->setPosition(-0.047);

    // SENSOR INTIATIONS
    for (int i = 0; i < 8; i++)
    { // setting up the 8 pannel of ir sensors of the robot
        irPanel[i] = robot->getDistanceSensor(irNames[i]);
    }

    for (int i = 0; i < 8; i++)
    { // setting up positional sensors
        psSensors[i] = robot->getPositionSensor(psNames[i]);
    }

    for (int i = 0; i < 3; i++)
    {
        psSensors[i]->enable(TIME_STEP);
        cams[i] = robot->getCamera(camNames[i]);
        cams[i]->enable(TIME_STEP);
    }

    for (int i = 0; i < 4; i++)
    {
        wallSensors[i] = robot->getDistanceSensor(wirNames[i]);
        wallSensors[i]->enable(TIME_STEP);
    }

    laserSensors[RIGHT_LAS] = robot->getDistanceSensor(laserNames);

    compass = robot->getCompass("compass");

    for (int i = 0; i < 9; i++)
    {
        obSensors[i] = robot->getDistanceSensor(obIRNames[i]);
    }
    obSensors[FRONT_IR_SHARP]->enable(TIME_STEP);

    i_unit = robot->getInertialUnit("inertial_unit");

    // request to complete the task
    DETECT_BALL();
    /* TASK_MANAGER(); */
    delete robot;
    return 0;
}

//------------------------------------------------------FUNCTION DEFINITIONS----------------------------------------------------//
void TASK_MANAGER()
{
    float laserValue = 0, preValue = 0;

    switch (CURRENT_TASK)
    {
    case LINE_FOLLOWING:
        // Line following task
        GO_FORWARD();
        LINE_FOLLOW();
        CURRENT_TASK = MAZE_SOLVE;

    case MAZE_SOLVE:
        // Maze solving task
        WALL_FOLLOW();
        CURRENT_TASK = HOLE_TASK;

    case HOLE_TASK:
        // Hole task
        compass->enable(TIME_STEP);                 // enabling compass
        laserSensors[RIGHT_LAS]->enable(TIME_STEP); // enabling laser sensor

        GO_FORWARD(12, 1);
        TURN_ANGLE(asin((SONAR_MAP(RIGHT_WALL2) - SONAR_MAP(RIGHT_WALL)) / 8) * 180 / 3.14);
        maze_north = READ_COMPASS();

        // minor adjustment before assigning north
        GO_FORWARD(12);
        ALIGN_TO_DIR(NORTH);

        cout << "MAZE NORTH IS ASSIGNED TO :" << maze_north << endl;
        while (robot->step(TIME_STEP) != -1)
        {
            GO_FORWARD(3);
            if (COLOR_DETECTION(RIGHT_CAMERA) == CYAN)
            {
                cout << "REACHED CYAN AREA...." << endl;
                STOP_ROBOT();
                break;
            }
        }
        // PICKING OBJECT
        while (object_state != 0)
        {
            GO_FORWARD(20, 1);
            TURN_ANGLE(30), GO_FORWARD(10);
            if (DETECT_OBJECT())
            {
                PICK_OBJECT();
            }
            ALIGN_TO_DIR(NORTH);
            while (robot->step(TIME_STEP) != -1)
            {
                if (COLOR_DETECTION(RIGHT_CAMERA) == MAGENTA)
                {
                    STOP_ROBOT();
                    break;
                }
                left_speed = -base_speed, right_speed = -base_speed;
                SET_VELOCITY();
            }
        }
        // GOING FOR PLACING OBJECT
        ALIGN_TO_DIR(WEST);
        while (robot->step(TIME_STEP) != -1)
        {
            if (COLOR_DETECTION(RIGHT_CAMERA) == BLACK || COLOR_DETECTION(LEFT_CAMERA) == YELLOW)
            {
                STOP_ROBOT();
                break;
            }
            left_speed = base_speed, right_speed = base_speed;
            SET_VELOCITY();
        }
        GO_FORWARD(65);
        ALIGN_TO_DIR(SOUTH);
        GO_FORWARD(10, 1);
        // HOLE RIGHT ALIGN RIGHT
        laserValue = LASER_MAP(RIGHT_LAS);
        while (robot->step(TIME_STEP) != -1)
        {
            preValue = laserValue;
            laserValue = LASER_MAP(RIGHT_LAS);
            if (laserValue - preValue > HOLE_DEPTH)
            {
                STOP_ROBOT();
                GO_FORWARD(CYLINDER_HOLE_ALIGN);
                ALIGN_TO_DIR(WEST);
                GO_FORWARD(12.2);
                DELAY(500);
                break;
            }
            left_speed = base_speed_slow, right_speed = base_speed_slow;
            SET_VELOCITY();
        }
        BASE_ARM_SWAP(1);
        SLIDER_ARM_MOVEMENT(1);
        DELAY(500);
        BASE_ARM_SWAP();
        ALIGN_TO_DIR(WEST);
        // ALIGNING CYLINDER LITTLE BY LITTLE
        for (int var = 0; var < 2; var++)
        {
            DELAY(500);
            SLIDER_ARM_MOVEMENT(1);
            GO_FORWARD(0.75, 1);
            SLIDER_ARM_MOVEMENT();
            DELAY(500);
            GO_FORWARD(0.9);
        }
        // NEDRI ADI
        SLIDER_ARM_MOVEMENT(1);
        GO_FORWARD(5, 1);
        SLIDER_ARM_MOVEMENT(2);
        GO_FORWARD(6.8, 0, 3);
        DELAY(500);
        GO_FORWARD(15, 1);
        BASE_ARM_SWAP(2);

        // PUSH CUBE
        ALIGN_TO_DIR(SOUTH);
        GO_FORWARD(GAP);
        laserValue = LASER_MAP(RIGHT_LAS);
        while (robot->step(TIME_STEP) != -1)
        {
            preValue = laserValue;
            laserValue = LASER_MAP(RIGHT_LAS);
            if (laserValue - preValue > HOLE_DEPTH)
            {
                for (int c = 0; c < 20; c++)
                {
                    right_speed = base_speed_slow, left_speed = base_speed_slow;
                    SET_VELOCITY();
                    robot->step(TIME_STEP);
                }
                laserValue = LASER_MAP(RIGHT_LAS);
                if (laserValue - preValue > HOLE_DEPTH)
                {
                    STOP_ROBOT();
                    GO_FORWARD(CUBE_HOLE_ALIGN);
                    ALIGN_TO_DIR(EAST);
                    GO_FORWARD(13.5, 1);
                    break;
                }
            }
            right_speed = base_speed, left_speed = base_speed;
            SET_VELOCITY();
        }
        // directional adjustment
        BASE_ARM_SWAP(1, 1);
        SLIDER_ARM_MOVEMENT(1, 1);
        DELAY(500);
        BASE_ARM_SWAP(0, 1);
        SLIDER_ARM_MOVEMENT(0, 1);
        ALIGN_TO_DIR(EAST);

        // STRAIGHT PUSH
        SLIDER_ARM_MOVEMENT(1, 1);
        GO_FORWARD(8.5);
        DELAY(500);
        SLIDER_ARM_MOVEMENT(2, 1);
        GO_FORWARD(11, 1);
        GO_FORWARD(10);
        BASE_ARM_SWAP(2, 1);

        CURRENT_TASK = BALL_PICK;

    case BALL_PICK:

        CURRENT_TASK = REACH_END;
        compass->disable();                 // disabling compass
        laserSensors[RIGHT_LAS]->disable(); // diable laser sensor
    case REACH_END:
        LINE_FOLLOW(true);
        CURRENT_TASK = KICK_BALL;
    case KICK_BALL:
        cout << "All the tasks have been completed" << endl;
    }
    return;
}

// SENSOR MODELLING FUNCTIONS
float OBJECT_IR_READ(obIRSensors ir_sensor)
{
    float value = obSensors[ir_sensor]->getValue();
    float distance = 0;
    bool far = true;
    float ir_lookup[7][2] = {{3, 3.03}, {6, 2.01}, {8, 1.55}, {10, 1.25}, {20, 0.66}, {30, 0.42}, {40, 0.31}};
    float ir_lookup2[6][2] = {{15, 933}, {25, 750}, {40, 500}, {60, 333}, {90, 233}, {150, 167}};

    short int i = 0;
    if (ir_sensor == FRONT_IR_SHARP)
    {
        while (i <= 4)
        {
            if ((ir_lookup2[i][1] >= value) && (value > ir_lookup2[i + 1][1]))
            {
                distance = ir_lookup2[i][0] + (value - ir_lookup2[i][1]) * (ir_lookup2[i + 1][0] - ir_lookup2[i][0]) / (ir_lookup2[i + 1][1] - ir_lookup2[i][1]);
                far = false;
            }
            i = i + 1;
        }
        if (far)
        {
            distance = value < 167 ? ir_lookup2[5][0] : ir_lookup2[0][0];
        }
    }
    else
    {
        while (i <= 5)
        {
            if ((ir_lookup[i][1] >= value) && (value > ir_lookup[i + 1][1]))
            {
                distance = ir_lookup[i][0] + (value - ir_lookup[i][1]) * (ir_lookup[i + 1][0] - ir_lookup[i][0]) / (ir_lookup[i + 1][1] - ir_lookup[i][1]);
                far = false;
            }
            i = i + 1;
        }
        if (far)
        {
            distance = value > 3.03 ? ir_lookup[0][0] : ir_lookup[6][0];
        }
    }
    return distance;
}

float LASER_MAP(laSensors laser)
{
    float distance = 2 + (laserSensors[laser]->getValue() * 198 / 1000);
    return distance;
}

float SONAR_MAP(wallSonars sonar)
{
    float distance = 2 + (wallSensors[sonar]->getValue() * 398 / 1000);
    return distance;
}

// PROCESSING SENSOR DATA RELATED FUNCTIONS
Colors COLOR_DETECTION(aCameras color_sensor)
{
    Colors color;
    const unsigned char *image = cams[color_sensor]->getImage();
    // take hight and width of the image
    int width = cams[color_sensor]->getWidth();
    int height = cams[color_sensor]->getHeight();

    // detect colours
    // check the center pixel of the image
    int r = cams[color_sensor]->imageGetRed(image, width, (int)(width / 2), (int)(height / 2));
    int g = cams[color_sensor]->imageGetGreen(image, width, (int)(width / 2), (int)(height / 2));
    int b = cams[color_sensor]->imageGetBlue(image, width, (int)(width / 2), (int)(height / 2));

    // Determine threshold
    float threshold = bright_scale_factor * 18;

    // give detected output
    if (r < threshold && g < threshold && b < threshold)
    {
        color = BLACK;
    }
    else if (r > threshold && g > threshold && b < threshold)
    {
        color = YELLOW;
    }
    else if (r > threshold && g < threshold && b > threshold)
    {
        color = MAGENTA;
    }
    else if (r < threshold && g > threshold && b > threshold)
    {
        color = CYAN;
    }
    else if (r > threshold && g < threshold && b < threshold)
    {
        color = RED;
    }
    else if (r < threshold && g < threshold && b > threshold)
    {
        color = BLUE;
    }
    else
    {
        color = WHITE;
    }
    return color;
}

double READ_COMPASS()
{
    float current_dir = 0, current_angle = 0;
    current_dir = atan2(compass->getValues()[0], compass->getValues()[2]);
    current_angle = current_dir * 180 / 3.14;
    current_angle = (current_angle < 0) ? current_angle + 360 : current_angle;
    return current_angle;
}

// MOTION RELATED FUNCTIONS
int ERROR_CALC()
{
    int error = 0, value = 0, e_state = 0;
    for (int i = 0; i < 8; i++)
    {
        value = irPanel[i]->getValue();
        e_state = (value < 100) ? 1 : 0;
        // calculate weighted error from IR panel data
        if (i < 4)
        {
            error += e_state * (4 - i) * 100;
        }
        else
        {
            error += e_state * (3 - i) * 100;
        }
    }
    if (error == 0)
    {
        I = 0;
    }
    return error;
}

void LINE_FOLLOW(bool dotted)
{
    // sensor Intialization
    for (int i = 0; i < 8; i++)
    {
        irPanel[i]->enable(TIME_STEP);
    }

    int acceleration = 0;

    // Normal Line Following
    double kp = 2.71, kd = 0.3, ki = 0.01;
    int error, P, D;
    if (dotted)
    {
        i_unit->enable(TIME_STEP);
        kp = 1, kd = 1.05, ki = 0.05;
    }

    while (robot->step(TIME_STEP) != -1)
    {
        acceleration = 0;
        if (dotted)
        {
            if (i_unit->getRollPitchYaw()[1] < 0.25)
            {
                kp = 0.9, ki = 0, kd = 0.525;
                acceleration = 2;
            }
            else if (i_unit->getRollPitchYaw()[1] > 0.25)
            {
                kp = 0.01, ki = 0, kd = 0.006;
                acceleration = -3;
            }
            if (irPanel[0]->getValue() < 750 && irPanel[1]->getValue() < 750 && irPanel[2]->getValue() < 750)
            {
                cout << "DOTTED LINE FOLLOWING TERMINATION" << endl;
                i_unit->disable();
                break;
            }
        }
        else if (SONAR_MAP(LEFT_WALL) < 15 && SONAR_MAP(RIGHT_WALL) < 15)
        {
            cout << "LINE FOLLOWING COMPLETED SUCESSFULLY" << endl;
            break;
        }
        error = ERROR_CALC();
        P = error;
        D = error - last_error;
        I = I + error;
        last_error = error;
        if (error == 0)
        {
            I = 0;
        }
        double correction = (kd * D + kp * P + ki * I) / 80;
        left_speed = base_speed + correction + acceleration, right_speed = base_speed - correction + acceleration;
        SET_VELOCITY();
    }

    STOP_ROBOT(), CLEAR_VARIABLES();
    // sensor terminations
    for (int i = 0; i < 8; i++)
    {
        irPanel[i]->disable();
    }

    return;
}

void WALL_FOLLOW()
{
    bool turned = false, double_turn = false;
    float limit_wall = WALL_LIMIT;
    float left_ds_value = SONAR_MAP(LEFT_WALL), right_ds_value = SONAR_MAP(RIGHT_WALL), right_ds_value2 = SONAR_MAP(RIGHT_WALL2), front_ds_value = SONAR_MAP(FRONT_WALL);

    auto keep_position = [&]()
    {
        if (right_ds_value2 < limit_wall && right_ds_value < limit_wall)
        {
            TURN_ANGLE(asin((SONAR_MAP(RIGHT_WALL2) - SONAR_MAP(RIGHT_WALL)) / 8) * 180 / 3.14);
        }
    };
    GO_FORWARD(5);
    keep_position();
    MINOR_CORRECTION_WALL();

    while (robot->step(TIME_STEP) != -1)
    {
        // readings
        left_ds_value = SONAR_MAP(LEFT_WALL), right_ds_value = SONAR_MAP(RIGHT_WALL), right_ds_value2 = SONAR_MAP(RIGHT_WALL2), front_ds_value = SONAR_MAP(FRONT_WALL);

        // termination
        if (COLOR_DETECTION(LEFT_CAMERA) == MAGENTA || COLOR_DETECTION(RIGHT_CAMERA) == MAGENTA)
        {
            break;
        }
        keep_position();
        if (right_ds_value2 < limit_wall && right_ds_value < limit_wall && turned)
        {
            GO_FORWARD(3, 1);
            MINOR_CORRECTION_WALL();
            GO_FORWARD(3);
            turned = false;
        }

        if (right_ds_value > limit_wall)
        {
            turned = true;
            if (double_turn)
            {
                GO_FORWARD(5.5);
                TURN_ANGLE(30, 1);
                GO_FORWARD();
                TURN_ANGLE(60, 1);
                GO_FORWARD(12);
                double_turn = false;
            }
            else
            {
                GO_FORWARD(6);
                TURN_ANGLE(30, 1);
                GO_FORWARD();
                TURN_ANGLE(60, 1);
                GO_FORWARD(8);
                double_turn = true;
            }
            // cout << "RIGHT TURN :" << SONAR_MAP(RIGHT_WALL) << endl;
        }
        else if ((left_ds_value < limit_wall) && (front_ds_value < limit_wall) && (right_ds_value < limit_wall))
        {
            MINOR_CORRECTION_WALL(9);
            TURN_ANGLE(180);
            GO_FORWARD(5, 1);
            MINOR_CORRECTION_WALL();
            keep_position();
            turned = true;
            double_turn = false;
            // cout << "DEAD END :" << SONAR_MAP(RIGHT_WALL) << endl;
        }
        else if (front_ds_value < limit_wall && right_ds_value < limit_wall)
        {
            MINOR_CORRECTION_WALL();
            FORWARD_CORRECTION();
            TURN_ANGLE(30);
            GO_FORWARD(LEFT_CENTER_DISTANCE);
            TURN_ANGLE(55);
            turned = true;
            double_turn = false;
            // cout << "LEFT TURN :" << SONAR_MAP(RIGHT_WALL) << endl;
        }
        else
        {
            left_speed = base_speed_fast, right_speed = base_speed_fast;
            SET_VELOCITY();
            double_turn = false;
        }
    }
    cout << "I AM GLAD THAT... I MAKE IT TO THE CMYK REGION" << endl;
    STOP_ROBOT();
    CLEAR_VARIABLES();

    return;
}

void ALIGN_TO_DIR(directions destination)
{
    double c_a = READ_COMPASS(), t_angle = 0;
    int n_angle = 0;
    switch (destination)
    {
    case NORTH:
        t_angle = c_a - maze_north;
        break;
    case EAST:
        t_angle = c_a - (maze_north + 90);
        break;
    case SOUTH:
        t_angle = c_a - (maze_north + 180);
        break;
    case WEST:
        t_angle = c_a - (maze_north + 270);
        break;
    }
    n_angle = t_angle;
    n_angle = (abs(n_angle) > 180) ? n_angle % 360 : n_angle;
    n_angle = (abs(n_angle) > 180) ? n_angle + 360 : n_angle;

    TURN_ANGLE(n_angle);
    return;
}

void TURN_90(short int dir)
{
    /* dir : {left, right} */
    double l_position = psSensors[psRightMotor]->getValue(), r_position = psSensors[psLeftMotor]->getValue(), lc_position, rc_position;

    if (dir)
    {
        right_speed = -base_speed;
        left_speed = base_speed;
    }
    else
    {
        right_speed = base_speed;
        left_speed = -base_speed;
    }

    while (robot->step(TIME_STEP) != -1)
    {
        lc_position = psSensors[psRightMotor]->getValue(), rc_position = psSensors[psLeftMotor]->getValue();
        if ((lc_position - l_position < turn90_angle) && (rc_position - r_position < turn90_angle))
        {
            SET_VELOCITY();
        }
        else
        {
            STOP_ROBOT();
            return;
        }
    }
}

void TURN_ANGLE(float angle, short int dir)
{ /* dir : {left, right} */
    if (angle < 0)
    {
        dir = 1;
        angle = abs(angle);
    }
    double l_position = psSensors[psRightMotor]->getValue(), r_position = psSensors[psLeftMotor]->getValue(), lc_position, rc_position;

    if (dir)
    {
        right_speed = -base_speed;
        left_speed = base_speed;
    }
    else
    {
        right_speed = base_speed;
        left_speed = -base_speed;
    }

    while (robot->step(TIME_STEP) != -1)
    {
        lc_position = psSensors[psRightMotor]->getValue(), rc_position = psSensors[psLeftMotor]->getValue();
        if ((lc_position - l_position < turn90_angle * angle / 90) && (rc_position - r_position < turn90_angle * angle / 90))
        {
            SET_VELOCITY();
        }
        else
        {
            STOP_ROBOT();
            return;
        }
    }
};

void GO_FORWARD(float distance, short int dir, float deceleration)
{
    /* dir : {forward, backward} */
    double l_position = psSensors[psRightMotor]->getValue(), r_position = psSensors[psLeftMotor]->getValue(), lc_position, rc_position;
    double t_angle = abs(distance) / (wheel_radius * 100);
    if (!dir && distance > 0)
    {
        right_speed = base_speed - deceleration;
        left_speed = base_speed - deceleration;
    }
    else
    {
        right_speed = -base_speed + deceleration;
        left_speed = -base_speed + deceleration;
    }

    while (robot->step(TIME_STEP) != -1)
    {
        lc_position = psSensors[psRightMotor]->getValue(), rc_position = psSensors[psLeftMotor]->getValue();
        if ((abs(lc_position - l_position) + abs(rc_position - r_position)) / 2 < t_angle)
        {
            SET_VELOCITY();
        }
        else
        {
            STOP_ROBOT();
            return;
        }
    }
    STOP_ROBOT();
    return;
}

// ARM RELATED FUNCTIONS
void BASE_ARM_SWAP(short int f_position, short int c_arm)
{
    /* Arm Base Position Change Function
    c_arm : Arm to change position {front, back}
       f_position : Position to which arm must go {fold,short lift, straight} */
    // sensor intiation
    for (int i = 0; i < 6; i++)
    {
        psSensors[i + 2]->enable(TIME_STEP);
    }
    double f_b_position = psSensors[psFBase]->getValue(), b_b_position = psSensors[psBBase]->getValue();
    double cu_time = robot->getTime(), c_time = robot->getTime();
    while (robot->step(TIME_STEP) != -1)
    {
        c_time = robot->getTime();
        if (c_arm == 0)
        { // swapping front arm base
            f_b_position = psSensors[psFBase]->getValue();
            if (f_position == 0)
            {
                // check---------------------------------------------------------------------------------
                if ((c_time > cu_time + ARM_BASE_DELAY) && object_state != 0)
                {
                    motors[FRONT_ARM_BASE]->setPosition(f_b_position - 0.2);
                    TURN_ANGLE(5, 1);
                    cu_time = c_time;
                }
                if (1.3 - f_b_position > 0.001)
                {
                    motors[FRONT_ARM_BASE]->setPosition(APPROACH_VALUE(f_b_position, 1.33, 0.08));
                }
                else
                {
                    return;
                }
            }
            else if (f_position == 1)
            {
                if (abs(f_b_position - 1.05) > 0.06)
                {
                    motors[FRONT_ARM_BASE]->setPosition(APPROACH_VALUE(f_b_position, 1.05, 0.08));
                }
                else
                {
                    return;
                }
            }
            else if (f_position == 2)
            {
                if (f_b_position - 0.2 > 0.04)
                {
                    motors[FRONT_ARM_BASE]->setPosition(APPROACH_VALUE(f_b_position, 0.2, 0.08));
                }
                else
                {
                    return;
                }
            }
        }
        else
        { // swapping back arm base
            b_b_position = psSensors[psBBase]->getValue();
            if (f_position == 0)
            {
                if ((c_time > cu_time + ARM_BASE_DELAY) && object_state != 0)
                {
                    motors[BACK_ARM_BASE]->setPosition(b_b_position - 0.2);
                    TURN_ANGLE(10, 1);
                    GO_FORWARD(1);
                    cu_time = c_time;
                }

                if (1.47 + b_b_position > 0.004)
                {
                    motors[BACK_ARM_BASE]->setPosition(APPROACH_VALUE(b_b_position, -1.5, 0.08));
                }
                else
                {
                    return;
                }
            }
            else if (f_position == 1)
            {
                if (abs(b_b_position + 1.1) > 0.06)
                {
                    motors[BACK_ARM_BASE]->setPosition(APPROACH_VALUE(b_b_position, -1.1, 0.08));
                }
                else
                {
                    return;
                }
            }
            else if (f_position == 2)
            {

                if (b_b_position + 0.2 < 0.004)
                {
                    motors[BACK_ARM_BASE]->setPosition(APPROACH_VALUE(b_b_position, -0.2, 0.06));
                }
                else
                {
                    return;
                }
            }
        }
    }
    for (int i = 0; i < 6; i++)
    {
        psSensors[i + 2]->disable();
    }
}

void SLIDER_ARM_MOVEMENT(short int s_position, short int c_arm)
{ /* s_position : {gripped, wide, push} */
    double ref_time = robot->getTime();
    if (!c_arm)
    {
        double fl_s_position = psSensors[psFLSlider]->getValue(), fr_s_position = psSensors[psFRSlider]->getValue();

        while (robot->step(TIME_STEP) != -1)
        {
            fl_s_position = psSensors[psFLSlider]->getValue(), fr_s_position = psSensors[psFRSlider]->getValue();
            if (s_position == 0)
            {
                if (abs(fl_s_position + 0.0001) > 0.002 && abs(fr_s_position - 0.0001) > 0.002)
                {
                    if ((robot->getTime() > ref_time + DELAY_ARM) && object_state != 0)
                    {
                        SLIDER_ARM_MOVEMENT(1);
                        TURN_ANGLE(10);
                        GO_FORWARD(0.5);
                        ref_time = robot->getTime();
                    }
                    motors[FRONT_ARM_LEFT]->setPosition(APPROACH_VALUE(fl_s_position, -0.02));
                    motors[FRONT_ARM_RIGHT]->setPosition(APPROACH_VALUE(fr_s_position, +0.02));
                }
                else
                {
                    motors[FRONT_ARM_LEFT]->setPosition(APPROACH_VALUE(fl_s_position, -0.02));
                    motors[FRONT_ARM_RIGHT]->setPosition(APPROACH_VALUE(fr_s_position, +0.02));
                    return;
                }
            }
            else if (s_position == 1)
            {
                if (abs(fl_s_position - 0.025) > 0.002 && abs(fr_s_position + 0.025) > 0.002)
                {
                    motors[FRONT_ARM_LEFT]->setPosition(APPROACH_VALUE(fl_s_position, 0.025));
                    motors[FRONT_ARM_RIGHT]->setPosition(APPROACH_VALUE(fr_s_position, -0.025));
                }
                else
                {
                    return;
                }
            }
            else if (s_position == 2)
            {
                if (abs(fl_s_position + 0.005) > 0.002 && abs(fr_s_position - 0.005) > 0.002)
                {
                    motors[FRONT_ARM_LEFT]->setPosition(APPROACH_VALUE(fl_s_position, -0.006));
                    motors[FRONT_ARM_RIGHT]->setPosition(APPROACH_VALUE(fr_s_position, 0.006));
                }
                else
                {
                    return;
                }
            }
        }
    }
    else
    {
        double bl_s_position = psSensors[psBLSlider]->getValue(), br_s_position = psSensors[psBRSlider]->getValue();
        while (robot->step(TIME_STEP) != -1)
        {
            bl_s_position = psSensors[psBLSlider]->getValue(), br_s_position = psSensors[psBRSlider]->getValue();
            if (s_position == 0)
            {
                if (abs(bl_s_position) > 0.002 && abs(br_s_position + 0.022) > 0.002)
                {
                    if ((robot->getTime() > ref_time + DELAY_ARM) && object_state != 0)
                    {
                        SLIDER_ARM_MOVEMENT(1, 1);
                        TURN_ANGLE(10);
                        GO_FORWARD(0.5);
                        ref_time = robot->getTime();
                    }
                    motors[BACK_ARM_LEFT]->setPosition(APPROACH_VALUE(bl_s_position, -0.1));
                    motors[BACK_ARM_RIGHT]->setPosition(APPROACH_VALUE(br_s_position, -0.017));
                }
                else
                {
                    motors[BACK_ARM_LEFT]->setPosition(APPROACH_VALUE(bl_s_position, -0.1, 0.001));
                    motors[BACK_ARM_RIGHT]->setPosition(APPROACH_VALUE(br_s_position, -0.017, 0.001));
                    return;
                }
            }
            else if (s_position == 1)
            {
                if (abs(bl_s_position - 0.025) > 0.002 && abs(br_s_position + 0.047) > 0.002)
                {
                    motors[BACK_ARM_LEFT]->setPosition(APPROACH_VALUE(bl_s_position, 0.025));
                    motors[BACK_ARM_RIGHT]->setPosition(APPROACH_VALUE(br_s_position, -0.047));
                }
                else
                {
                    return;
                }
            }
            else if (s_position == 2)
            {
                if (abs(bl_s_position + 0.006) > 0.002 && abs(br_s_position + 0.012) > 0.002)
                {
                    motors[BACK_ARM_LEFT]->setPosition(APPROACH_VALUE(bl_s_position, -0.006));
                    motors[BACK_ARM_RIGHT]->setPosition(APPROACH_VALUE(br_s_position, -0.012));
                }
                else
                {
                    return;
                }
            }
        }
    }
}

// OBJECT PICKING-PLACING FUNCTIONS
void ALIGN_TO_OBJECT(float distance)
{
    // sensor intiation
    for (int i = 0; i < 3; i++)
    {
        obSensors[i]->enable(TIME_STEP);
    }
    double kp = 2.71, kd = 0.3, ki = 0.01, error = 0;
    int cofficient = 1;
    while (robot->step(TIME_STEP) != -1)
    {
        float front = OBJECT_IR_READ(FRONT_IR), left = OBJECT_IR_READ(LEFT_ALIGN_IR), right = OBJECT_IR_READ(RIGHT_ALIGN_IR);
        if (front < distance || left < distance || right < distance)
        {
            break;
        }

        error = (front - left) - (front - right);

        float P = error, D = error - last_error;
        I = I + error, last_error = error;
        if (error == 0)
        {
            I = 0;
        }
        double correction = (kd * D + kp * P + ki * I) * cofficient;
        left_speed = base_speed - correction, right_speed = base_speed + correction;
        SET_VELOCITY();
    }
    STOP_ROBOT();
    // sensor termination
    for (int i = 0; i < 3; i++)
    {
        obSensors[i]->disable();
    }
    return;
}

void OBJECT_CONFIRMATION()
{
    obSensors[FRONT_IR]->enable(robot->step(TIME_STEP) != -1);
    obSensors[BACK_IR]->enable(robot->step(TIME_STEP) != -1);
    if (object_state == 1 || object_state == 3)
    {
        BASE_ARM_SWAP(2, 1);
        if (OBJECT_IR_READ(BACK_IR) < ARM_DISTANCE_BACKWARD)
        {
            SLIDER_ARM_MOVEMENT(1, 1);
        }
        else
        {
            object_state = (object_state == 3) ? 4 : 0;
        }
    }
    else if (object_state == 2 || object_state == 4)
    {
        BASE_ARM_SWAP(2);
        if (OBJECT_IR_READ(FRONT_IR) < 0.7 * ARM_DISTANCE_FORWARD)
        {
            SLIDER_ARM_MOVEMENT();
        }
        else
        {
            object_state = (object_state == 4) ? 0 : 1;
        }
    }
    else
    {
        BASE_ARM_SWAP(2);
        if (OBJECT_IR_READ(FRONT_IR) < 0.7 * ARM_DISTANCE_FORWARD)
        {
            SLIDER_ARM_MOVEMENT(1);
        }
        else
        {
            object_state = -1;
        }
    }
    obSensors[FRONT_IR]->disable();
    obSensors[BACK_IR]->disable();
    return;
}

void PICK_OBJECT()
{
    holeObjects objt;
    (object_state != 0) ? ALIGN_TO_OBJECT(): STOP_ROBOT();
    if (object_state == 2 || object_state == 4)
    {
        GO_FORWARD(ARM_DISTANCE_FORWARD);
        BASE_ARM_SWAP();
        SLIDER_ARM_MOVEMENT();
        objt = IDENTIFY_OBJECT();
        cout << holeObject[objt] << endl;
        MAKE_FUN1(objt);
        if (objt == CUBE)
        {
            DELAY(300);
            SLIDER_ARM_MOVEMENT(1);
            BASE_ARM_SWAP(2);
            GO_FORWARD(6, 1);
            TURN_ANGLE(180, 1);
            GO_FORWARD(5, 1);
            BASE_ARM_SWAP(0, 1);
            SLIDER_ARM_MOVEMENT(0, 1);
            object_state = 3;
        }
        else if (objt == CYLINDER_CIRCLE)
        {
            ALIGN_TO_CYLINDER();
        }
    }
    else if (object_state == 1)
    {
        TURN_ANGLE(180);
        GO_FORWARD(ARM_DISTANCE_BACKWARD, 1);
        BASE_ARM_SWAP(0, 1);
        SLIDER_ARM_MOVEMENT(0, 1);
    }

    else
    {
        GO_FORWARD(ARM_DISTANCE_FORWARD);
        BASE_ARM_SWAP();
        SLIDER_ARM_MOVEMENT(2);
        return;
    }
    OBJECT_CONFIRMATION();
    return;
}

holeObjects IDENTIFY_OBJECT()
{
    float front = 0, left = 0, right = 0, back = 0;
    holeObjects objec;
    // intiating sensors
    for (int i = 4; i < 8; i++)
    {
        obSensors[i]->enable(TIME_STEP);
    }
    DELAY(250);
    front = obSensors[FRONT_OBJECT]->getValue(), left = obSensors[LEFT_OBJECT]->getValue(), right = obSensors[RIGHT_OBJECT]->getValue(), back = obSensors[BACK_OBJECT]->getValue();
    if (front > 850)
    {
        DELAY(250);
        SLIDER_ARM_MOVEMENT(1);
        while (front > 850)
        {
            GO_FORWARD(0.25, 1);
            front = obSensors[FRONT_OBJECT]->getValue();
        }
        SLIDER_ARM_MOVEMENT();
        DELAY(250);
    }
    if (back > 850)
    {
        DELAY(250);
        SLIDER_ARM_MOVEMENT(1);
        while (back > 850)
        {
            GO_FORWARD(0.25);
            back = obSensors[BACK_OBJECT]->getValue();
        }

        SLIDER_ARM_MOVEMENT();
        DELAY(250);
    }

    if (IN_RANGE(front, left) && IN_RANGE(front, right) && IN_RANGE(back, front))
    {
        objec = CUBE;
    }
    else if (IN_RANGE(right, left) && (front - right > 0 || back - left > 0))
    {
        objec = CYLINDER_CIRCLE;
    }
    else
    {
        objec = CYLINDER_CURVED;
    }
    DELAY(250);
    cout << "Left IR Reading : " << left << endl;
    cout << "Right IR Reading : " << right << endl;
    cout << "Straight IR Reading : " << front << endl;
    cout << "Back IR Reading : " << back << endl;
    // disabling sensor
    for (int i = 4; i < 8; i++)
    {
        obSensors[i]->disable();
    }

    return objec;
}

void ALIGN_TO_CYLINDER()
{
    DELAY(2000);
    SLIDER_ARM_MOVEMENT(1);
    BASE_ARM_SWAP(2);
    GO_FORWARD(8, 1);
    TURN_90();
    double l_position = psSensors[psRightMotor]->getValue(), lc_position, correction = 1.8;
    left_speed = base_speed_slow + correction;
    right_speed = base_speed_slow - correction;
    double limit = CYLINDER_TUNE_ANGLE;
    SET_VELOCITY();
    while (robot->step(TIME_STEP) != -1)
    {
        lc_position = psSensors[psRightMotor]->getValue();
        if (lc_position > l_position + limit)
        {
            STOP_ROBOT();
            break;
        }
    }
    TURN_ANGLE(85, 1);
    GO_FORWARD(8.25);
    BASE_ARM_SWAP();
    SLIDER_ARM_MOVEMENT();
    return;
}

bool DETECT_OBJECT()
{
    // sensor intiation
    obSensors[FRONT_IR_SHARP]->enable(TIME_STEP);
    float right_distance = 0;
    ALIGN_TO_DIR(WEST);
    GO_FORWARD(5, 1);
    while (robot->step(TIME_STEP) != -1)
    {
        right_distance = LASER_MAP(RIGHT_LAS);
        if (COLOR_DETECTION(LEFT_CAMERA) == YELLOW || COLOR_DETECTION(RIGHT_CAMERA) == BLACK)
        {
            STOP_ROBOT();
            ALIGN_TO_DIR(EAST);
            GO_FORWARD(30);
            obSensors[FRONT_IR_SHARP]->disable();
            return ROUND_SEARCH();
        }
        if (right_distance < 80) // Typical wall distance 81
        {
            STOP_ROBOT();
            break;
        }
        left_speed = base_speed, right_speed = base_speed;
        SET_VELOCITY();
    }
    TURN_ANGLE(77, 1);
    while (robot->step(TIME_STEP) != -1)
    {
        TURN_ANGLE(0.5, 1);
        if (OBJECT_IR_READ(FRONT_IR_SHARP) < 120)
        {
            break;
        }
    }
    if (right_distance > 36) // IR range
    {
        GO_FORWARD(right_distance - 36);
    }
    // sensor termination
    obSensors[FRONT_IR_SHARP]->disable();
    return true;
}

bool ROUND_SEARCH()
{
    // sensor intiation
    obSensors[FRONT_IR_SHARP]->enable(TIME_STEP);

    float preValue = 0, irValue = 0;
    bool breaked = false;

    TURN_ANGLE(120);
    irValue = OBJECT_IR_READ(FRONT_IR_SHARP);
    for (int c = 0; c < 120; c++)
    {
        preValue = irValue;
        TURN_ANGLE(1, 1);
        irValue = OBJECT_IR_READ(FRONT_IR_SHARP);
        if (abs(preValue - irValue) > 15)
        {
            breaked = true;
            break;
        }
    }
    if (irValue > 36)
    {
        GO_FORWARD(irValue - 36);
    }
    // sensor termination
    obSensors[FRONT_IR_SHARP]->disable();
    return breaked ? true : false;
}

void DETECT_BALL()
{
    float f_distance = 0, p_distance = 0;

    f_distance = SONAR_MAP(FRONT_WALL);
    for (float angle = 0; angle < 70; angle = angle + 0.5)
    {
        p_distance = f_distance;
        f_distance = SONAR_MAP(FRONT_WALL);
        if (f_distance < 35 && p_distance - f_distance > 5)
        {
            GO_FORWARD(f_distance - ARM_DISTANCE_BALL);
            STOP_ROBOT();
            object_state = 0;
            PICK_OBJECT();
            cout<<colorNames[COLOR_DETECTION(FRONT_CAMERA)]<<endl;
            return;
        }
        cout << angle << endl;
        TURN_ANGLE(-0.5);
    }
    GO_FORWARD(15);
    DETECT_BALL();
}

// CORRECTIONS FUNCTIONS
float APPROACH_VALUE(float c_value, float d_value, float a_step)
{
    if (c_value > d_value)
    {
        return c_value - a_step;
    }
    else
    {
        return c_value + a_step;
    }
}

bool IN_RANGE(float a, float b)
{
    if (abs(a - b) < 30)
    {
        return true;
    }
    return false;
}

void MINOR_CORRECTION_WALL(float distance)
{
    float distan = 0;
    distan = SONAR_MAP(RIGHT_WALL) * 1.155 - distance;
    if (distan < 0)
    {
        distan = abs(distan);
        TURN_ANGLE(30);
        GO_FORWARD(distan);
        TURN_ANGLE(30, 1);
    }
    else
    {
        TURN_ANGLE(30, 1);
        GO_FORWARD(distan);
        TURN_ANGLE(30);
    }
    return;
}

void FORWARD_CORRECTION(float dist)
{
    float c_distance = SONAR_MAP(FRONT_WALL);
    GO_FORWARD(c_distance - dist);
}

// ROBOT ESSENTIAL FUNCTIONS
double LIMIT(double &val)
{
    /* limit the velocity of the motors */
    if (val > MAX_SPEED)
    {
        return MAX_SPEED;
    }
    else if (val < -MAX_SPEED)
    {
        return -MAX_SPEED;
    }
    else
    {
        return val;
    }
}

void DELAY(double t)
{
    double c_time = robot->getTime();
    double f_time = c_time + t / 1000;
    double cu_time;

    while (robot->step(TIME_STEP) != -1)
    {
        cu_time = robot->getTime();
        if (cu_time > f_time)
        {
            break;
        }
    }
    return;
}

void STOP_ROBOT()
{
    left_speed = 0, right_speed = 0;
    motors[LEFT]->setVelocity(0);
    motors[RIGHT]->setVelocity(0);
}

void SET_VELOCITY()
{
    left_speed = LIMIT(left_speed), right_speed = LIMIT(right_speed);
    motors[LEFT]->setVelocity(left_speed);
    motors[RIGHT]->setVelocity(right_speed);
}

void CLEAR_VARIABLES()
{
    I = 0, last_error = 0;
}

// FUN COMMENTS
void MAKE_FUN1(holeObjects var)
{
    switch (var)
    {
    case CYLINDER_CIRCLE:
        cout << "OKAY...HOLD ON I AM COMING OTHER WAY AROUND" << endl;
        break;
    case CYLINDER_CURVED:
        cout << "OH! OKAY... MY TASK IS EASY NOW..." << endl;
        break;
    case CUBE:
        cout << "LET ME SEE FROM BACK SIDE...." << endl;
        break;
    }
    return;
}

//-----------------------------------------------------------------**THE END**-----------------------------------------------------------------------------//
void AIM_GOAL()
{
    if (BALL_COLOR == BLUE)
    {
        TURN_ANGLE(7);
    }
    while (robot->step(TIME_STEP) != -1)
    {
        continue;
    }
}