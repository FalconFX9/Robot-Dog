#include <Ramp.h>

class Interpolation {
    public:
        rampDouble myRamp;
        int interpFlag = 0;
        double prev_value;

        Interpolation() {
            myRamp.setGrain(1);
        }

        double go(double input, unsigned long duration){
            
            if (input != prev_value){
                interpFlag = 0;
            }
            prev_value = input;
            if (interpFlag == 0){
                myRamp.go(input, duration, LINEAR, ONCEFORWARD);
                interpFlag = 1;
            }
            myRamp.update();
            return myRamp.getValue();
        }
};