#include "ECU.hpp"
#include "origin_vehicle.h"
#include "math_util.h"

#include <GL/glu.h>
#include <GL/glut.h>

void ECU::draw_road_path(const Path &p, int every, int point_size,
                         float line_width, float r, float g, float b) {
    if (p.ref_points.size() < 3)
        return;

    unsigned int i;
    double x, xx;
    double y, yy;

    glPointSize(point_size);
    glColor3d(r, g, b);
    glBegin(GL_POINTS);

    for (i = 0; i < p.ref_points.size() - every; i += every) {
        xx = p.ref_points[i].position_x;
        yy = p.ref_points[i].position_y;
        glVertex2f(xx, yy);
    }
    glEnd();

    glLineWidth(line_width);
    glColor3d(r, g, b);
    glBegin(GL_LINES);

    for (i = 0; i < p.ref_points.size() - every; i += every) {
        x = p.ref_points[i].position_x;
        y = p.ref_points[i].position_y;

        xx = p.ref_points[i + every].position_x;
        yy = p.ref_points[i + every].position_y;

        glVertex2f(x, y);
        glVertex2f(xx, yy);
    }
    glEnd();

}

void ECU::draw_road_lane(const Path &lane, double lane_width, int every,
                         float line_width, float r, float g, float b) {
    unsigned int i;
    double direction;
    double direction_d;
    double cos_d;
    double sin_d;

    double x, xx, x11, x22, x33, x44;
    double y, yy, y11, y22, y33, y44;

    if (int(lane.ref_points.size()) < 3 * every)
        return;

    for (i = 0; i < lane.ref_points.size() - every; i += every) {
        glLineWidth(line_width);
        glColor3d(r, g, b);

        direction = lane.ref_points[i].heading;
        x = lane.ref_points[i].position_x;
        y = lane.ref_points[i].position_y;
        direction_d = to_radians(direction);
        cos_d = cos(direction_d);
        sin_d = sin(direction_d);

        x11 = x - lane_width / 2.0 * cos_d;
        y11 = y + lane_width / 2.0 * sin_d;
        x22 = x + lane_width / 2.0 * cos_d;
        y22 = y - lane_width / 2.0 * sin_d;

        direction = lane.ref_points[i + every].heading;
        xx = lane.ref_points[i + every].position_x;
        yy = lane.ref_points[i + every].position_y;

        direction_d = to_radians(direction);
        cos_d = cos(direction_d);
        sin_d = sin(direction_d);

        x33 = xx - lane_width / 2.0 * cos_d;
        y33 = yy + lane_width / 2.0 * sin_d;
        x44 = xx + lane_width / 2.0 * cos_d;
        y44 = yy - lane_width / 2.0 * sin_d;

        glBegin(GL_LINE_STRIP);
        glVertex2f(x11, y11);
        glVertex2f(x22, y22);
        glVertex2f(x44, y44);
        glVertex2f(x33, y33);
        glVertex2f(x11, y11);
        glEnd();
    }

}

void ECU::Draw_Virtual_lane(const Path &v_p, double lane_width, int every,
                            float r, float g, float b) {
    draw_road_lane(v_p, lane_width, every, 1.0, r, g, b);
}

void ECU::DrawPath_ChangeLane() {
    const int every = 10;

    if (frame_path.ref_points.size() < 3)
        return;

    unsigned int i;
    double x, xx;
    double y, yy;

    glPointSize(8);
    glColor3d(1, 1, 0);
    glBegin(GL_POINTS);

    for (i = 0; i < frame_path.ref_points.size(); i += every) {
        xx = frame_path.ref_points[i].position_x;
        yy = frame_path.ref_points[i].position_y;
        glVertex2f(xx, yy);
    }
    glEnd();

    glLineWidth(3);
    glColor3d(1, 0, 0);
    glBegin(GL_LINES);

    for (i = 0; i < frame_path.ref_points.size() - every; i += every) {
        x = frame_path.ref_points[i].position_x;
        y = frame_path.ref_points[i].position_y;

        xx = frame_path.ref_points[i + every].position_x;
        yy = frame_path.ref_points[i + every].position_y;

        glVertex2f(x, y);
        glVertex2f(xx, yy);
    }
    glEnd();

}

