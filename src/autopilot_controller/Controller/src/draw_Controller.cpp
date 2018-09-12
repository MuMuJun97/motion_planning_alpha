#include "draw_Controller.h"

#include <cmath>
#include <cstdlib>

#include <GL/glut.h>
#include <GL/freeglut_ext.h>

#include "origin_vehicle.h"
#include "Controller.hpp"

using namespace std;

Controller *g_car_controller;

int x_lbefore, y_lbefore;
int x_rbefore, y_rbefore;
int z_before1, z_before2;

bool buttonSaveLeft, buttonSaveMiddle, buttonSaveRight;
float x_move, y_move, z_move;
float x_move_save, y_move_save, z_move_save;
float x_rotate, y_rotate, z_rotate;
float x_rotate_save, y_rotate_save, z_rotate_save;
float m_zoom;

float m_aspect;

float m_eyex, m_eyey, m_eyez;
float m_centerx, m_centery, m_centerz;
float m_upx, m_upy, m_upz;

///////////////////OPEN GL control ///////////////////////////////////////
int g_frame;
bool g_pause;

void OpenGL_Draw() {
    x_move = 0, y_move = 0, z_move = 0;
    x_rotate = 1, y_rotate = 1, z_rotate = 1;
    m_zoom = 1;
    g_frame = 0;
}

void Reshape(int w, int h) {
    glViewport(0, 0, (GLint) w, (GLint) h);

    m_aspect = (GLfloat) w / (GLfloat) h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0f, m_aspect, 0.0f, 4000.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

void MouseMove(int x, int y) {
    int mod = glutGetModifiers();
    switch (mod) {
        case GLUT_ACTIVE_CTRL:
            x_rotate += (y - z_move_save) / 100;
            if (x_rotate > 360)
                x_rotate = x_rotate - 360;
            if (x_rotate < -360)
                x_rotate = x_rotate + 360;
            return;

        case GLUT_ACTIVE_SHIFT:
            y_rotate += (y - z_move_save) / 100;
            if (y_rotate > 360)
                y_rotate = y_rotate - 360;
            if (y_rotate < -360)
                y_rotate = y_rotate + 360;
            return;

        case GLUT_ACTIVE_ALT:
            float temp = (x - x_move_save) / 100;
            z_rotate += atanf(temp);
            return;

    }

    if (buttonSaveLeft) {
        x_move += (x - x_move_save) / 100;
        z_move += (y - z_move_save) / 100;
    }

    if (buttonSaveMiddle) {
        float multiplay = (y - z_move_save) / 1000000;
        m_zoom = m_zoom * (1 + multiplay);
    }

    if (buttonSaveRight) {
        float multiplay = (y - z_move_save) / 1000000;
        m_zoom = m_zoom * (1 + multiplay);
    }

}

void PassiveMouseMove(int x, int y) {

}

void MouseRotate(int x, int y, int z) {
    //    cout << "mouse Rotate "  << x  << "  "<< y << "  "<< z <<endl;
}

void MouseKey(int button, int state, int x, int y) {
    x_move_save = x;
//    y_move_save;
    z_move_save = y;

    switch (button) {
        case GLUT_LEFT_BUTTON:
            if (state == GLUT_DOWN)
                buttonSaveLeft = true;
            else
                buttonSaveLeft = false;
            break;

        case GLUT_MIDDLE_BUTTON:
            if (state == GLUT_DOWN)
                buttonSaveMiddle = true;
            else
                buttonSaveMiddle = false;
            break;

        case GLUT_RIGHT_BUTTON:
            if (state == GLUT_DOWN)
                buttonSaveRight = true;
            else
                buttonSaveRight = false;
            break;
    }
}

void Key(unsigned char key, int x, int y) {
    switch (key) {
        case KEY_ESC:
            glutLeaveMainLoop();
            break;
    }
}

void SpecialKey(int key, int x, int y) {
    int mod = 0;
    switch (key) {
        case GLUT_KEY_UP:
            mod = glutGetModifiers();
            if (mod == GLUT_ACTIVE_ALT) {

            } else if (mod == GLUT_ACTIVE_SHIFT) {

            } else if (mod == GLUT_ACTIVE_CTRL) {

            } else
                y_move++;

            break;

        case GLUT_KEY_DOWN:
            mod = glutGetModifiers();
            if (mod == GLUT_ACTIVE_ALT) {

            } else if (mod == GLUT_ACTIVE_SHIFT) {

            } else if (mod == GLUT_ACTIVE_CTRL) {

            } else
                y_move--;
            break;

        case GLUT_KEY_LEFT:
            mod = glutGetModifiers();
            if (mod == GLUT_ACTIVE_ALT) {

            } else if (mod == GLUT_ACTIVE_SHIFT) {

            } else if (mod == GLUT_ACTIVE_CTRL) {

            } else
                x_move--;
            break;

        case GLUT_KEY_RIGHT:
            mod = glutGetModifiers();
            if (mod == GLUT_ACTIVE_ALT) {

            } else if (mod == GLUT_ACTIVE_SHIFT) {

            } else if (mod == GLUT_ACTIVE_CTRL) {

            } else
                x_move++;
            break;

        case GLUT_KEY_PAGE_UP:
            m_zoom = 1.1 * m_zoom;
            break;

        case GLUT_KEY_PAGE_DOWN:
            m_zoom = m_zoom / 1.1;
            break;

        case GLUT_KEY_HOME:
            m_zoom = 1.5 * m_zoom;
            break;

        case GLUT_KEY_END:
            m_zoom = m_zoom / 1.5;
            break;

        case GLUT_KEY_F1:
            g_car_controller->auto_driver = true;

            /*    x_rotate += 3;
             if (x_rotate > 360)
             x_rotate=x_rotate - 360;
             if (x_rotate < -360)
             x_rotate=x_rotate + 360;
             */

            break;

        case GLUT_KEY_F2:
            x_rotate += -3;
            if (x_rotate > 360)
                x_rotate = x_rotate - 360;
            if (x_rotate < -360)
                x_rotate = x_rotate + 360;
            break;

        case GLUT_KEY_F3:
            y_rotate += 3;
            if (y_rotate > 360)
                y_rotate = y_rotate - 360;
            if (y_rotate < -360)
                y_rotate = y_rotate + 360;
            break;

        case GLUT_KEY_F4:
            y_rotate += -3;
            if (y_rotate > 360)
                y_rotate = y_rotate - 360;
            if (y_rotate < -360)
                y_rotate = y_rotate + 360;
            break;

        case GLUT_KEY_F5:
            //  g_car_controller->b_motion_plann_continue = false;
            //  cout << " b_motion_plann_continue " << g_car_controller->b_motion_plann_continue << endl;

            z_rotate += atanf(3);
            break;

        case GLUT_KEY_F6:
            //  g_car_controller->b_motion_plann_continue = true;
            //  cout << " b_motion_plann_continue " << g_car_controller->b_motion_plann_continue << endl;

            z_rotate += atanf(-3);
            break;

        case GLUT_KEY_F9:
            // g_car_controller->b_motion_plann_continue = false;
            // cout << " b_motion_plann_continue " << g_car_controller->b_motion_plann_continue << endl;
            break;

        case GLUT_KEY_F10:
            // g_car_controller->b_motion_plann_continue = true;
            //  cout << " b_motion_plann_continue " << g_car_controller->b_motion_plann_continue << endl;
            break;

        case GLUT_KEY_F11:
            // g_car_controller->b_motion_plann_continue = false;
            // cout << " b_motion_plann_continue " << g_car_controller->b_motion_plann_continue << endl;
            break;

        case GLUT_KEY_F12:
            // g_car_controller->b_motion_plann_continue = true;
            //  cout << " b_motion_plann_continue " << g_car_controller->b_motion_plann_continue << endl;
            break;

    }
    glutPostRedisplay();
}

void myDisplay(void) {
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    glLoadIdentity();

    m_eyex = g_car_controller->CurrentX;
    m_eyey = g_car_controller->CurrentY;
    m_eyez = g_car_controller->CurrentZ + 90.0;

    m_centerx = g_car_controller->CurrentX;
    m_centery = g_car_controller->CurrentY;
    m_centerz = g_car_controller->CurrentZ;

    m_upx = g_car_controller->CurrentZ;
    m_upy = g_car_controller->CurrentY + 15.0;
    m_upz = g_car_controller->CurrentX;

    gluLookAt(m_eyex, m_eyey, m_eyez, m_centerx, m_centery, m_centerz, m_upx, m_upy, m_upz);

    glScalef(1, 1, 1);

    glRotatef(x_rotate, 1, 0, 0);
    glRotatef(y_rotate, 0, 1, 0);
    glRotatef(z_rotate, 0, 0, 1);

    glTranslatef(x_move, y_move, z_move);
    glScalef(m_zoom, m_zoom, m_zoom);

    g_car_controller->Draw_Org();

    g_car_controller->DrawCar(g_car_controller->CurrentX, g_car_controller->CurrentY,
                              g_car_controller->Current_yaw, g_car_controller->steer_angle);

    g_car_controller->DrawCar_e(g_car_controller->CurrentX, g_car_controller->CurrentY,
                                g_car_controller->position_x_C, g_car_controller->position_y_C,
                                g_car_controller->Current_yaw, g_car_controller->Yaw_C);

    g_car_controller->Draw_Virtual_lane(*(g_car_controller->curr_path),
                                        origin_vehicle::CAR_WIDTH * 1.5, g_car_controller->draw_step_points);
    g_car_controller->Draw_Best_March_Point(*(g_car_controller->curr_path));

    glutPostRedisplay();

    glFlush();
    glutSwapBuffers();

}

void MyGLDispIni(Controller *car_controller, const std::string &controller_name) {
    g_car_controller = car_controller;

    GLenum type;

    m_eyex = 0, m_eyey = 0, m_eyez = 100;
    m_centerx = 0, m_centery = 0, m_centerz = 0;
    m_upx = 0, m_upy = 1, m_upz = 0;

    buttonSaveLeft = false;
    buttonSaveMiddle = false;
    buttonSaveRight = false;

    x_move = 0.0;
    y_move = 0.0;
    z_move = 0.0;
    x_rotate = 0.0;
    y_rotate = 0.0;
    z_rotate = 0.0;
    m_zoom = 1;

    x_lbefore = 0, y_lbefore = 0;
    x_rbefore = 0, y_rbefore = 0;
    z_before1 = 0, z_before2 = 0;

    type = GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE;
    glutInitDisplayMode(type);

    glutInitWindowSize(740, 860);
    glutCreateWindow(controller_name.c_str());

    glutReshapeFunc(Reshape);
    glutKeyboardFunc(Key);
    glutSpecialFunc(SpecialKey);
    glutMouseFunc(MouseKey);
    glutMotionFunc(MouseMove);
    glutPassiveMotionFunc(PassiveMouseMove);
    glutSpaceballRotateFunc(MouseRotate);

    glutDisplayFunc(&myDisplay);

    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
}
