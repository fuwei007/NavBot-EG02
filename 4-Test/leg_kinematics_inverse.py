
import time
import math
import numpy as np

import motormodule as mm
mmc = mm.MotorModuleController('COM8')

# Position deviation in writing and reading
CODER_OFFSET = [0,2800,1337,1400]

kp = 2250
kd = 2250

# electric machine parameter
THIGH_ID = 2
CALF_ID = 3
MAX_POSITION = 65535
CALF_INITIAL_POSITION = 33200
THIGH_INITIAL_POSITION = 33467
# Maximum stepping value of the motor
step_max = 30

# Mechanical parameters (unit: centimeters)
THIGH_LENGTH = 21.5
CALF_LENGTH = 21.5
MAX_REACH = THIGH_LENGTH + CALF_LENGTH

# kinematics parameter
MOTOR_RANGE = 4
DEGREES_PER_COUNT = [0,MAX_POSITION/(360 * 4),
                       MAX_POSITION/(360 * 4),
                       MAX_POSITION/(360 * 2.5)]

def set_location(motor_id, position):

    position = int(position)
    mmc.mysend_command(motor_id, position , 0, kp, kd, 0)


def get_location(motor_id):
    return (mmc.rx_values[motor_id][1] + CODER_OFFSET[motor_id]) & 0XFFFF


def on_leg():
    print("Enable motor...")
    mmc.enable_motor(THIGH_ID)
    mmc.enable_motor(CALF_ID)
    time.sleep(0.1)


def off_leg():
    print("Disabie motor...")
    mmc.disable_motor(THIGH_ID)
    mmc.disable_motor(CALF_ID)
    time.sleep(0.1)


def leg_init():

    print("Initialize the motor...")
    off_leg()
    time.sleep(2)
    # Refresh current location
    off_leg()
    thigh_start_position = get_location(THIGH_ID)
    calf_start_position = get_location(CALF_ID)
    print(f"Current position value of the motor  Thigh= {thigh_start_position} calf= {calf_start_position}")
    # Locate to the current position
    set_location(THIGH_ID, thigh_start_position)
    set_location(CALF_ID, calf_start_position)

    on_leg()
    # Maintain the same number of steps, and calculate the steps for each motor.
    thigh_position = thigh_start_position
    calf_position = calf_start_position
    if abs(thigh_position-THIGH_INITIAL_POSITION) > abs(calf_position-CALF_INITIAL_POSITION):
        thigh_step = step_max
        calf_step = (abs(calf_position-CALF_INITIAL_POSITION)/abs(thigh_position-THIGH_INITIAL_POSITION)*step_max)
    else :
        thigh_step = (abs(thigh_position-THIGH_INITIAL_POSITION)/abs(calf_position-CALF_INITIAL_POSITION)*step_max)
        calf_step = step_max
    # Smoothly move to the zero point
    thigh_ok = 0
    calf_ok = 0
    while(thigh_ok==0 or calf_ok==0):

        if thigh_position < (THIGH_INITIAL_POSITION-thigh_step):
            thigh_position += thigh_step
        elif thigh_position > (THIGH_INITIAL_POSITION+thigh_step):
            thigh_position -= thigh_step
        else :
            thigh_ok =1
        set_location(THIGH_ID, thigh_position)

        if calf_position < (CALF_INITIAL_POSITION-calf_step):
            calf_position += calf_step
        elif calf_position > (CALF_INITIAL_POSITION+calf_step):
            calf_position -= calf_step
        else :
            calf_ok =1
        set_location(CALF_ID, calf_position)

        get_thigh_position = get_location(THIGH_ID)
        get_calf_position = get_location(CALF_ID)     
        print(f"Get the value. Thigh.= {get_thigh_position} calf= {get_calf_position} Set the value. Thigh.= {thigh_position} Calf= {calf_position}")
        # time.sleep(0.03)

    thigh_start_position = get_location(THIGH_ID)
    calf_start_position = get_location(CALF_ID)
    print(f"Current position value of the motor  Thigh= {thigh_start_position} Calf= {calf_start_position}")

class LegKinematics:
    """Leg kinematics calculation category"""
    
    def __init__(self, thigh_length, calf_length):
        self.thigh_length = thigh_length
        self.calf_length = calf_length
        self.max_reach = thigh_length + calf_length
        
        # Calibration parameters - These will be set during the calibration process
        self.thigh_home_offset = 0
        self.calf_home_offset = 0
        self.actual_initial_thigh = 32767  # Default value will be updated after calibration.
        self.actual_initial_calf = 32767   # Default value will be updated after calibration.
    def forward_kinematics(self, thigh_encoder, calf_encoder):
        thigh_angle, calf_angle = self.encoder_to_angles(thigh_encoder, calf_encoder)
        
        thigh_rad = math.radians(thigh_angle)
        calf_rad = math.radians(calf_angle)
        
        thigh_x = self.thigh_length * math.sin(thigh_rad)
        thigh_y = -self.thigh_length * math.cos(thigh_rad)
        
        total_angle = thigh_rad + calf_rad
        toe_x = thigh_x + self.calf_length * math.sin(total_angle)
        toe_y = thigh_y - self.calf_length * math.cos(total_angle)
        
        return toe_x, toe_y
    
    def inverse_kinematics(self, target_x, target_y):
        """Inverse kinematics: Calculating joint angles based on the position of the toe tips"""
        distance = math.sqrt(target_x**2 + target_y**2) 
        
        if distance > self.max_reach:
            raise ValueError(f"The target position is beyond the reachable range! Maximum distance: {self.max_reach:.1f}cm")
        if distance < 10:
            raise ValueError("The target location is too close to the origin and cannot be reached.!")

        
        # Calculate the angle of the lower leg   cos_C (a^2 +b^2 - c^2) / 2ab
        cos_calf = (self.thigh_length**2 + self.calf_length**2 - (target_x**2 + target_y**2) ) / \
                  (2 * self.thigh_length * self.calf_length)
        cos_calf = max(min(cos_calf, 1), -1)
        calf_angle = (180 - math.degrees(math.acos(cos_calf))) *-1
        
        # Calculate the angle of the thigh
        alpha = math.atan2(target_x,target_y)

        cos_beta = ( self.thigh_length**2 + distance**2 - self.calf_length**2) / \
                   (2 *self.thigh_length * distance)
        cos_beta = max(min(cos_beta, 1), -1)
        beta = math.acos(cos_beta)
        thigh_angle = math.degrees(beta-alpha)


        # print(f"beta={beta}, alpha={alpha}")
        return thigh_angle, calf_angle
    def angles_to_encoder(self, thigh_angle, calf_angle):

        thigh_encoder = THIGH_INITIAL_POSITION - int(thigh_angle * DEGREES_PER_COUNT[THIGH_ID]) 
        calf_encoder = CALF_INITIAL_POSITION - int(calf_angle * DEGREES_PER_COUNT[CALF_ID])
        
        thigh_encoder = max(0, min(thigh_encoder, MAX_POSITION))
        calf_encoder = max(0, min(calf_encoder, MAX_POSITION))
        
        return thigh_encoder, calf_encoder
    
    def encoder_to_angles(self, thigh_encoder, calf_encoder):
        """Convert the encoder values to joint angles (taking into account the calibration offset)"""
        thigh_angle = (thigh_encoder - self.actual_initial_thigh - self.thigh_home_offset) * (360 * MOTOR_RANGE) / MAX_POSITION
        calf_angle = (calf_encoder - self.actual_initial_calf - self.calf_home_offset) * (360 * MOTOR_RANGE) / MAX_POSITION
        return thigh_angle, calf_angle


# Global Kinematics Example
kinematics = LegKinematics(THIGH_LENGTH, CALF_LENGTH)

def get_current_angles():
    """Obtain the current joint angle"""
    thigh_encoder = get_location(THIGH_ID)
    calf_encoder = get_location(CALF_ID)
    return kinematics.encoder_to_angles(thigh_encoder, calf_encoder)

def get_current_toe_position():
    """Obtain the current position of the toe tip"""
    thigh_encoder = get_location(THIGH_ID)
    calf_encoder = get_location(CALF_ID)
    print(f"Current code value: Thigh={thigh_encoder}, calf={calf_encoder}")
    return kinematics.forward_kinematics(thigh_encoder, calf_encoder)

def move_toe_to_position(target_x, target_y, duration=10.0, tolerance=0.5):
    """Move the tip of the foot to the designated position."""
    try:
        # Calculate the target angle
        thigh_angle, calf_angle = kinematics.inverse_kinematics(target_x, target_y)
        target_thigh_enc, target_calf_enc = kinematics.angles_to_encoder(thigh_angle, calf_angle)

        # Obtain the current position of the encoder
        current_thigh_enc = get_location(THIGH_ID)
        current_calf_enc = get_location(CALF_ID)

        # Maintain the same number of steps, and calculate the steps for each motor.
        thigh_position = current_thigh_enc
        calf_position = current_calf_enc
        if abs(thigh_position-target_thigh_enc) > abs(calf_position-target_calf_enc):
            thigh_step = step_max
            calf_step = (abs(calf_position-target_calf_enc)/abs(thigh_position-target_thigh_enc)*step_max)
        else :
            thigh_step = (abs(thigh_position-target_thigh_enc)/abs(calf_position-target_calf_enc)*step_max)
            calf_step = step_max
        # Smoothly move to the target value
        thigh_ok = 0
        calf_ok = 0
        while(thigh_ok==0 or calf_ok==0):

            if thigh_position < (target_thigh_enc-thigh_step):
                thigh_position += thigh_step
            elif thigh_position > (target_thigh_enc+thigh_step):
                thigh_position -= thigh_step
            else :
                thigh_ok =1
            set_location(THIGH_ID, thigh_position)

            if calf_position < (target_calf_enc-calf_step):
                calf_position += calf_step
            elif calf_position > (target_calf_enc+calf_step):
                calf_position -= calf_step
            else :
                calf_ok =1
            set_location(CALF_ID, calf_position)
        
        # Final verification
        final_x, final_y = get_current_toe_position()
        final_error = math.sqrt((final_x - target_x)**2 + (final_y - target_y)**2)
        
        return final_error <= tolerance
        
    except ValueError as e:
        print(f"error: {e}")
        return False

    
def main():
    print("Precise positioning control of the toes of the robot dog")
    print("=" * 70)

    leg_init()

    while True:
        print("\Please select an operation.:")
        print("1 - Move to the specified coordinates")
        print("8 - quit")
        
        choice = input("Please make a selection. (1-8): ")
        
        if choice == "1":
            try:
                x = float(input("X (positive forward, negative backward): "))
                y = float(input("Y (positive below, negative above): "))
                move_toe_to_position(x, y)
            except ValueError:
                print("Invalid input!")
        elif choice == "8":
            print("exit program")
            off_leg()
            break
        else:
            print("Invalid choice!")




def test():
    TEST_ID = 2
    leg_init()
    while True :

        angle = float(input("Please enter the angle. "))
        # Calculate the target value
        calf_encoder = THIGH_INITIAL_POSITION + int(angle * DEGREES_PER_COUNT[TEST_ID])
        # Obtain the current position of the encoder
        current_enc = get_location(TEST_ID)
        print(f"DEGREES_PER_COUNT = {DEGREES_PER_COUNT[TEST_ID]}")
        print(f"Now value{current_enc} Target value{calf_encoder}")
        # Smooth movement
        position = current_enc
        calf_ok = 0
        while( calf_ok==0):
            if position < (calf_encoder-step_max):
                position += step_max
            elif position > (calf_encoder+step_max):
                position -= step_max
            else :
                calf_ok =1
            set_location(TEST_ID, position)

if __name__ == "__main__":
    main()
    # test()