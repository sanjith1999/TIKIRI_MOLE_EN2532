void AIM_TO_GOAL()
{
    /* dir : {left, right} */
    double l_position, r_position, lc_position, rc_position;
    float laserValue = SONAR_MAP(sonarSensors[FRONT_SON]), preValue, correctionAngle;

    if (BALL_COLOR == BLUE)
    {
        TURN_ANGLE(7);
        left_speed = -base_speed_slow;
        right_speed = base_speed_slow;
    }
    else
    {
        TURN_ANGLE(7, 1);
        left_speed = base_speed_slow;
        right_speed = -base_speed_slow;
    }

    while (robot->step(TIME_STEP) != -1)
    {
        lc_position = psSensors[psRightMotor]->getValue();
        rc_position = psSensors[psLeftMotor]->getValue();
        preValue = laserValue;
        laserValue = SONAR_MAP(sonarSensors[FRONT_SON]);

        SET_VELOCITY();
        if (laserValue - preValue > 5)
        {
            l_position = lc_position;
            r_position = rc_position;
        }
        else if (preValue - laserValue < 5)
        {
            STOP_ROBOT;
            break;
        }
    }

    correctionAngle = ((lc_position - l_position) / 2 + (rc_position - r_position) / 2) * (4 * wheel_radius) / robot_width;
    if (BALL_COLOR == BLUE)
    {
        TURN_ANGLE(correctionAngle, 1);
    }
    else
    {
        TURN_ANGLE(correctionAngle);
    }
    return;
}

void DOUBLE_CHECK_OBJECT()
{
    double l_position = psSensors[psRightMotor]->getValue(), r_position = psSensors[psLeftMotor]->getValue(), lc_position, rc_position;
    float laserValue = SONAR_MAP(sonarSensors[FRONT_SON]), preValue;
    bool error = true; // turns false if no object is detected
    ALIGN_TO_DIR(NORTH);

    left_speed = base_speed;
    right_speed = -base_speed;

    while (robot->step(TIME_STEP) != -1)
    {
        preValue = laserValue;
        laserValue = (sonarSensors[FRONT_SON]);
        lc_position = psSensors[psRightMotor]->getValue(), rc_position = psSensors[psLeftMotor]->getValue();
        if (preValue - laserValue > 5)
        {
            STOP_ROBOT();
            break;
        }

        if ((lc_position - l_position < turn90_angle) && (rc_position - r_position < turn90_angle))
        {
            SET_VELOCITY();
        }
        else
        {
            STOP_ROBOT();
            error = false;
            break;
        }
    }

    if (error == false)
    {
        ALIGN_TO_DIR(WEST);
        return;
    }
    else
    {
        if (laserValue < 36)
        {
            GO_FORWARD(laserValue - 36);
        }
        ALIGN_TO_OBJECT();
        PICK_OBJECT();
        ALIGN_TO_DIR(SOUTH);
        laserValue = SONAR_MAP(sonarSensors[RIGHT_SON]);
        if (laserValue < 60) // second object might be in the way. This is a rare case
        {
            if (laserValue < 36)
            {
                GO_FORWARD(laserValue - 36);
            }
            ALIGN_TO_OBJECT();
            PICK_OBJECT();
            ALIGN_TO_DIR(SOUTH);
        }

        while (robot->step(TIME_STEP) != -1)
        {
            if (COLOR_DETECTION(cams[LEFT_CAMERA]) == MAGENTA || COLOR_DETECTION(cams[RIGHT_CAMERA]) == MAGENTA)
            {
                STOP_ROBOT();
                break;
            }
            left_speed = base_speed;
            right_speed = base_speed;
            SET_VELOCITY();
        }

        ALIGN_TO_DIR(WEST);

        while (robot->step(TIME_STEP) != -1)
        {
            if (COLOR_DETECTION(cams[LEFT_CAMERA]) == YELLOW || COLOR_DETECTION(cams[RIGHT_CAMERA]) == BLACK)
            {
                STOP_ROBOT();
                break;
            }
            left_speed = base_speed;
            right_speed = base_speed;
            SET_VELOCITY();
        }

    DOUBLE_CHECK_OBJECT();

    }
}