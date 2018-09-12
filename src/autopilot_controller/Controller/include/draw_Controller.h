#ifndef _DRAW_CONTROLLER_H__
#define _DRAW_CONTROLLER_H__

#include "Controller.hpp"

#define KEY_ESC (27)

void OpenGL_Draw();

void MyGLDispIni(Controller *car_controller, const std::string &car_name);

void myDisplay();

void SpecialKey(int key, int x, int y);

void MouseKey(int button, int state, int x, int y);

void MouseRotate(int x, int y, int z);

void PassiveMouseMove(int x, int y);

void MouseMove(int x, int y);

void Reshape(int w, int h);

#endif /*_DRAW_CONTROLLER_H__*/
