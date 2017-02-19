#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include "WPILib.h"
#include "CANTalon.h"
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include <thread>
#include <doubleSolenoid.h>
#include <AnalogPotentiometer.h>
#include <functional>
#include "AnalogInput.h"
#include "Arcade.cpp"

START_ROBOT_CLASS(Robot)
