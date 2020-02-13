#include "ColorSensor.hpp"
#include "ColorWheelStopper.hpp"

enum colorModePick {rotation, colorPick};
int colorThingy;
double primaryColor = (matchedColor); //need to have a way to make it not switch primary color
double counter;
double desiredColor;

//need to make it so some of the variables in color sensor work in here

switch (colorThingy) {
    case rotation:
        if detectedColor = primaryColor{
            counter = counter + 1;
            }
            // else (???) {
            //     ??????????? no clue what to put here.. maybe just counter = counter??
            // }
        break;

    case colorPick:
        //need a way to put in the desired color
        if (desiredColor == "Red") {
            //stop color wheel-- probably won't be able to code this until we have robot
        }
        else if (desiredColor == "Blue") {
            //stop color wheel-- probably won't be able to code this until we have robot
        }
        else if (desiredColor == "Green") {
            //stop color wheel-- probably won't be able to code this until we have robot
        }
        else if (desiredColor == "Yellow") {
            //stop color wheel-- probably won't be able to code this until we have robot
        }
        else {
            //color too far away-- basically make it so it does nothing
        }
        break;
    
}

