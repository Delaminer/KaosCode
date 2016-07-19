package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by HowellFTCUser1 on 10/30/2015.
 */
public class TestAutoNonDrive {

    /*move forward until until color sensor senses white line
    if it sees white line, slow down and continue moving forward
    if it does not see white line after five seconds, turn left
    if it does not see white line after three seconds after turning left, then turn right
    if it does see white, continue moving forward
    if it does not see white after turning right, repeat from line three to six
    if left touch sensor is pushed, stop left track
    if right touch sensor is pushed, stop right track
    if both front touch sensors are pushed, make sure that the robot is stopped
    if left front color sensor senses team color, turn button pushing servo clockwise
    if left front color sensor senses opposing color, turn button pushing servo counter-clockwise
    if button is pushed, back up
    */
}