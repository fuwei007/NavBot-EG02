 

import time
import math
import numpy as np

import leg_kinematics_inverse as lk

def test():

    lk.leg_init()

    i = 0
    j = 0
    ex = 100
    while(ex) :
        
        # Linear stage: Initial coordinate 32,5, End coordinate 32, -5
        if(i<200):
            y = 32 + (32-32) /200 *i
            x = 5  + (-5-5)  /200 *i
        # Lifting leg stage: Initial coordinates 32, -5; End coordinates 28, -2
        elif(i<233):
            y = 32 + (28-32) /33 *  (i-200)
            x = -5  + (-2 - -5)  /33 * (i-200)
        # Leg movement stage: Initial coordinate 28, -2; End coordinate 28, 2
        elif(i<266):
            y = 28 + (28-28) /33 *   (i-233)
            x = -2  + (2 - -2)  /33 *(i-233)
        # Lower leg stage: Starting coordinate 28, 2; Ending coordinate 32, 5
        elif(i<299):
            y = 28 + (32-28) /33 *   (i-266)
            x = 2  + (5 - 2)  /33 *  (i-266)
        else:
            ex -= 1
            j = 1
            
        lk.move_toe_to_position(x, y)
        i += 1
        if j :
            i = 0
            j = 0
        


if __name__ == "__main__":
    test()


