bool COLOR_BOUND_FOLLOWING(Colors color1, Colors color2, short int dirc)
{
    /* color1 : left_color, color2: right_color, dirc : {forward, backward} */
    short int error = 0;

    Colors left_color = COLOR_DETECTION(cams[LEFT_CAMERA]);
    Colors right_color = COLOR_DETECTION(cams[RIGHT_CAMERA]);
    if ((left_color != color1 && left_color != color2) || (right_color != color1 && right_color != color2))
    {
        STOP_ROBOT(), CLEAR_VARIABLES();
        return true;
    }
    if (left_color == color1 && right_color == color1)
    {
        error = dirc ? +1 : -1;
    }

    else if (left_color == color2 && right_color == color2)
    {
        error = dirc ? -1 : +1;
    }
    else
    {
        error = 0;
    }
    double correction = error;

    left_speed = 2 * (0.5 - dirc) * base_speed_slow, left_speed -= correction, left_speed = LIMIT(left_speed);
    right_speed = 2 * (0.5 - dirc) * base_speed_slow, right_speed += correction, right_speed = LIMIT(right_speed);

    SET_VELOCITY();
    return false;
}

void RIGHT_WALL_FOLLOW(float dist)
{
    /* wall_side : left, right
    distanc : distance in cm which should be maintained with respect to wall */
    float kp = 2.71, kd = 0.3, ki = 0.08, error = 0;
    int cofficient = 30;

    float r_w = WALL_IR_READ(RIGHT_WALL), r_w2 = WALL_IR_READ(RIGHT_WALL2);
    error = (abs(r_w - r_w2) > 0.05) ? 2 * (r_w - r_w2) : (r_w - dist / 100);

    float P = error, D = error - last_error;
    I = I + error, last_error = error;
    if (error == 0)
    {
        I = 0;
    }
    double correction = (kd * D + kp * P + ki * I) * cofficient;
    left_speed = base_speed + correction, right_speed = base_speed - correction;
    SET_VELOCITY();
    return;
}
