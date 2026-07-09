#ifndef KEYBOARD_LISTENER_H
#define KEYBOARD_LISTENER_H

#include <libevdev/libevdev.h>
#include "woosh_robot_control.h"

class KeyboardListener {
public:
    KeyboardListener(const char *devicePath, WooshRobotControl& robotControl);
    ~KeyboardListener();
    void run();

private:
    void initDevice();
    void handleKeyPress(int code, int value);
    void updateKeyStates();

    const char *devicePath_;
    int fd_;
    libevdev *dev_;
    WooshRobotControl& robotControl_;
    bool wPressed, aPressed, sPressed, dPressed;
    bool isKeyPressedW;
    bool isKeyPressedA;
    bool isKeyPressedS;
    bool isKeyPressedD;
    bool comboPressed;
};

#endif // KEYBOARD_LISTENER_H
