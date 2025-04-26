/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       unknown                                                   */
/*    Created:      12/1/2024, 11:02:31 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "competition/autonomous.h"
#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"
#include <thread>

using namespace vex;

// A global instance of competition
competition Competition;

//
// Main will set up the competition functions and callbacks.
//
int main() {
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(opcontrol);
    robot_init();
}
