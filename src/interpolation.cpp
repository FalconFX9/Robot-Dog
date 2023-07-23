#include <Ramp.h>

class Interpolation {
    public:
        rampInt myRamp;
        int interpFlag = 0;
        int prev_value;

        int go(int input, unsigned long duration){
            
            if (input != prev_value){
                interpFlag = 0;
            }
            prev_value = input;
            if (interpFlag == 0){
                myRamp.go(input, duration, LINEAR, ONCEFORWARD);
                interpFlag = 1;
            }
            return myRamp.update();
        }
};