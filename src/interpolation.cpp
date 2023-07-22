#include <Ramp.h>

class Interpolation {
    public:
        rampInt myRamp;
        int interpFlag = 0;
        int prev_value;

        int go(int input, float duration){
            
            if (input != prev_value){
                interpFlag = 0;
            }

            if (interpFlag == 0){
                myRamp.go(input, duration, LINEAR, ONCEFORWARD);
                interpFlag = 1;
            }
            return myRamp.update();
        }
};