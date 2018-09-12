#include "ECU.hpp"
#include "math_util.h"
#include "origin_vehicle.h"

#include <sys/sem.h>
#include <GL/glu.h>
#include <GL/glut.h>

void ECU::Draw_Org() {
    glPointSize(8);
    glColor3d(1, 1, 1);
    glBegin(GL_POINTS);
    glVertex2f(0, 0);
    glEnd();
    //
    glColor3d(1, 1, 1);
    glBegin(GL_LINES);
    glVertex2f(0 - 2, 0);
    glVertex2f(0 + 2, 0);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(0, 0 - 3);
    glVertex2f(0, 0 + 3);
    glEnd();
}

void ECU::DrawCar_e(double x, double y, double x_c, double y_c, double yaw, double yaw_c) {
    glLineWidth(2);
    glColor3d(0, 1, 0);
    glBegin(GL_LINES);
    glVertex2f(x, y);
    glVertex2f(x_c, y_c);
    glEnd();
}

void ECU::DrawCar(double x, double y, double yaw, double steer_angle) {
    glPointSize(8);
    glColor3d(1, 0, 0);
    glBegin(GL_POINTS);
    glVertex2f(x + (1.2) * sin(yaw / 180 * PI), y + (1.2) * cos(yaw / 180 * PI));
    glEnd();

    double frontwtht = to_radians(yaw);
    double wheel_direction = to_radians(yaw + steer_angle / 15.0 + 90); //

    double frontWhellCenterX = x + (1.2) * sin(frontwtht);
    double frontWhellCenterY = y + (1.2) * cos(frontwtht);
    double backWhellCenterX = x - (1.5) * sin(frontwtht);
    double backWhellCenterY = y - (1.5) * cos(frontwtht);

    glLineWidth(2);
    glColor3d(0, 1, 0);
    glBegin(GL_LINES);
    glVertex2f(x, y);
    glVertex2f(frontWhellCenterX, frontWhellCenterY);
    glEnd();

    glColor3d(1, 0, 0);
    glBegin(GL_LINES);
    glVertex2f(x, y);
    glVertex2f(backWhellCenterX, backWhellCenterY);
    glEnd();

    double frontWhellLeftX = frontWhellCenterX - ((front_track / 2.0) * cos(frontwtht));
    double frontWhellLeftY = frontWhellCenterY + ((front_track / 2.0) * sin(frontwtht));
    double frontWhellRightX = frontWhellCenterX + ((front_track / 2.0) * cos(frontwtht));
    double frontWhellRightY = frontWhellCenterY - ((front_track / 2.0) * sin(frontwtht));

    glLineWidth(2);
    glColor3d(1, 0, 1);

    glBegin(GL_LINES);
    glVertex2f(frontWhellLeftX, frontWhellLeftY);
    glVertex2f(frontWhellRightX, frontWhellRightY);
    glEnd();

    double frontLeftWhellUpX = frontWhellLeftX - ((front_wheel_wide / 2.0) * cos(wheel_direction));
    double frontLeftWhellUpY = frontWhellLeftY + ((front_wheel_wide / 2.0) * sin(wheel_direction));
    double frontLeftWhellDownX = frontWhellLeftX
                                 + ((front_wheel_wide / 2.0) * cos(wheel_direction));
    double frontLeftWhellDownY = frontWhellLeftY
                                 - ((front_wheel_wide / 2.0) * sin(wheel_direction));

    glLineWidth(2);
    glColor3d(1, 0, 1);

    glBegin(GL_LINES);
    glVertex2f(frontLeftWhellUpX, frontLeftWhellUpY);
    glVertex2f(frontLeftWhellDownX, frontLeftWhellDownY);
    glEnd();

    double frontrightWhellUpX = frontWhellRightX
                                - ((front_wheel_wide / 2.0) * cos(wheel_direction));
    double frontrightWhellUpY = frontWhellRightY
                                + ((front_wheel_wide / 2.0) * sin(wheel_direction));
    double frontrightWhellDownX = frontWhellRightX
                                  + ((front_wheel_wide / 2.0) * cos(wheel_direction));
    double frontrightWhellDownY = frontWhellRightY
                                  - ((front_wheel_wide / 2.0) * sin(wheel_direction));

    glLineWidth(2);
    glColor3d(1, 0, 1);

    glBegin(GL_LINES);
    glVertex2f(frontrightWhellUpX, frontrightWhellUpY);
    glVertex2f(frontrightWhellDownX, frontrightWhellDownY);
    glEnd();

    double BackWhellLeftX = backWhellCenterX - ((front_track / 2.0) * cos(frontwtht));
    double BackWhellLeftY = backWhellCenterY + ((front_track / 2.0) * sin(frontwtht));
    double BackWhellRightX = backWhellCenterX + ((front_track / 2.0) * cos(frontwtht));
    double BackWhellRightY = backWhellCenterY - ((front_track / 2.0) * sin(frontwtht));

    double backwtht = to_radians(yaw + 90);

    glLineWidth(2);
    glColor3d(1, 0, 1);

    glBegin(GL_LINES);
    glVertex2f(BackWhellRightX, BackWhellRightY);
    glVertex2f(BackWhellLeftX, BackWhellLeftY);
    glEnd();

    double BackLeftWhellUpX = BackWhellLeftX - ((back_wheel_wide / 2.0) * cos(backwtht));
    double BackLeftWhellUpY = BackWhellLeftY + ((back_wheel_wide / 2.0) * sin(backwtht));
    double BackLeftWhellDownX = BackWhellLeftX + ((back_wheel_wide / 2.0) * cos(backwtht));
    double BackLeftWhellDownY = BackWhellLeftY - ((back_wheel_wide / 2.0) * sin(backwtht));

    glLineWidth(2);
    glColor3d(1, 0, 1);

    glBegin(GL_LINES);
    glVertex2f(BackLeftWhellDownX, BackLeftWhellDownY);
    glVertex2f(BackLeftWhellUpX, BackLeftWhellUpY);
    glEnd();

    double BackrightWhellUpX = BackWhellRightX - ((back_wheel_wide / 2.0) * cos(backwtht));
    double BackrightWhellUpY = BackWhellRightY + ((back_wheel_wide / 2.0) * sin(backwtht));
    double BackrightWhellDownX = BackWhellRightX + ((back_wheel_wide / 2.0) * cos(backwtht));
    double BackrightWhellDownY = BackWhellRightY - ((back_wheel_wide / 2.0) * sin(backwtht));

    glLineWidth(2);
    glColor3d(1, 0, 1);

    glBegin(GL_LINES);
    glVertex2f(BackrightWhellUpX, BackrightWhellUpY);
    glVertex2f(BackrightWhellDownX, BackrightWhellDownY);
    glEnd();
}
