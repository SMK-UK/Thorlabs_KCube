"""
Connect to and control Thorlabs KCube

@author Sean Keenan 11/02/2024

Based on
https://github.com/rbrenesh/PyKinesis/blob/main/PyKinesis.py

"""
import os, sys
import time

try:
    import clr
    from System import Decimal 
except Exception as e:
    print(e)
    print("please install pythonnet with: pip install pythonnet")

# set the address for Thorlabs Kinesis folder
path = r"C:\Program Files\Thorlabs\Kinesis"
# Set correct DLL locations
clr.AddReference(os.path.join(path, r"Thorlabs.MotionControl.DeviceManagerCLI.dll"))
clr.AddReference(os.path.join(path, r"Thorlabs.MotionControl.GenericMotorCLI.dll"))
clr.AddReference(os.path.join(path, r"Thorlabs.MotionControl.KCube.DCServoCLI.dll"))
# import relevant DLL
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import KCubeMotor
from Thorlabs.MotionControl.GenericMotorCLI.ControlParameters import JogParametersBase
from Thorlabs.MotionControl.KCube.DCServoCLI import *

class KinesisMotor():
    
    def __init__(self, devicename: str, serial_no: str, homing: bool=True, pol_time: int=100, verbose: bool=True):
        '''
        Initialise the class for controlling the motor

        <device_name>:
            Name of device i.e. "PRM1-MZ8", "MTS25-MZ8" etc.
        <serial_num>:
            Serial number of your device
        <pol_time>:
            Set polling interval for communicating with the device (ms)
        <verbose>:
            Used to suppress print statements
        '''
        # build device list otherwise will fail to connect
        DeviceManagerCLI.BuildDeviceList()

        self.connected = False
        self.device_name = devicename
        self.home = homing
        self.lims = None
        self.pol_time = pol_time
        self.verbose = verbose

        # check serial number format
        if serial_no is not None:
            if type(serial_no)==int:
                serial_no = str(serial_no)
            if self.verbose: print ("Serial is", serial_no)
            self.serial_no = str(serial_no)
            # create device
            self.device = KCubeDCServo.CreateKCubeDCServo(self.serial_no)
        else:
            raise Exception('Please provide a serial number')
        
        # initialise the device and get params for later functions
        self.initialise()
        self.get_limits()
        self.get_vel_params()
         
    def disconnect(self):
        '''
        Disconnect from the device and shut down
        
        '''
        try:
            self.device.StopPolling()
            time.sleep(0.25)
            # This shuts down the controller. This will use the .NET Disconnect() function to close 
            # communications & will then close the used library.
            self.device.ShutDown()
            time.sleep(0.25)
            if self.verbose:
                print(f'{self.device_name} now disconnected')
        except Exception as e:
            print(f'Unexpected error occurred {e}')
       
    def go_to_home(self, time_out=60000):
        '''
        Return motor to home position

        '''
        if self.verbose:
            print('Homing Motor, please wait')
        self.device.Home(time_out)

    def get_limits(self):
        '''
        Get limits from the device

        '''
        try:
            # get all current limits of the motor
            self.lims = self.device.AdvancedMotorLimits
            self.max_vel_lim = Decimal.ToDouble(self.lims.get_VelocityMaximum())
            self.max_accel_lim = Decimal.ToDouble(self.lims.get_AccelerationMaximum())
            self.max_travel_lim = Decimal.ToDouble(self.lims.get_LengthMaximum())
            self.min_travel_lim = Decimal.ToDouble(self.lims.get_LengthMinimum())
            
            if self.verbose:
                print('*** Motor Limits ***')
                print(f'Max velocity limit {self.max_vel_lim} mm/s')
                print(f'Acceleration limit {self.max_accel_lim} mm/s^2')
                print(f'Max travel limit {self.max_travel_lim} mm')
                print(f'Min travel limit {self.min_travel_lim} mm')
                print('----------')
            
        except (NameError, AttributeError):
            raise Exception('Please initialize the device first')
        
    def get_pos(self):
        '''
        Return the current position of the device
        
        '''
        try:
            pos = Decimal.ToDouble(self.device.Position)

        except (NameError, AttributeError):
            raise Exception('Please initialize the device first')

        return pos
        
    def get_vel_params(self):
        '''
        Get the current velocity params for the device
        '''
        try:
            # get all current velocity params
            self.velPars = self.device.GetVelocityParams()
            self.accel = Decimal.ToDouble(self.velPars.get_Acceleration())
            self.min_vel = Decimal.ToDouble(self.velPars.get_MinVelocity())
            self.max_vel = Decimal.ToDouble(self.velPars.get_MaxVelocity())

            if self.verbose:
                print('**** Current Velocity Params ****')
                print('Min Velocity:',self.min_vel,'mm/s')
                print('Max Velocity:',self.max_vel,'mm/s')
                print('Acceleration:',self.accel,'mm/s^2')
                print('----------')

        except (NameError, AttributeError):
            raise Exception('Please initialize the device first')
        
    def initialise(self, home=True):
        '''
        Initialise the device and begin connection

        <home>:
            choose to return the motor to home position

        '''
        # attempt to connect to device
        try:
            self.device.Connect(self.serial_no)
            # wait for connection
            if not self.device.IsSettingsInitialized():
                self.device.WaitForSettingsInitialized(5000) # 5 sec timeout
                assert self.device.IsSettingsInitialized() is True
            # set configuration for the motor
            self.load_config()
            # begin communication with device
            self.start_polling()
            # print out connection success
            if self.verbose:
                deviceInfo = self.device.GetDeviceInfo()
                print(f'Motor Connected: {deviceInfo.Name}, SN: {deviceInfo.SerialNumber}')
            # return motor to home position          
            if self.home:
                self.go_to_home()  

        except Exception as e:
            print(f'Unexpected errror occured: {e}')
    
    def load_config(self):
        '''
        Get config from the device        

        '''
        # ensure motors configuration is loaded
        device_config = self.device.LoadMotorConfiguration(self.serial_no, 
                                                           DeviceConfiguration.DeviceSettingsUseOptionType.UseFileSettings)
        # set device name
        device_config.DeviceSettingsName = self.device_name
        device_config.UpdateCurrentConfiguration()
        self.device.SetSettings(self.device.MotorDeviceSettings, True, False)

    def move_to(self, position:float, tolerance:float=0.02):
        '''
        Move the stage to the desired position
        
        <position>:
            location to move to in units of stage
        <tolerance>:
            tolerance to change in actual position from desired
        
        '''
        try:
            if self.verbose:
                print(f'Moving Motor to {position}')
            # set move position and then move
            self.device.SetMoveAbsolutePosition(Decimal(position)) 
            self.device.MoveAbsolute(50000)
            # check correct location and 
            current_pos = self.get_pos()
            if current_pos < position * (1-tolerance) and current_pos > position*(1+tolerance):
                if self.verbose:
                    print(f'Move completed to {current_pos}')

        except Exception as e:
            print(f'Unexpected error occured: {e}')
        
    def set_acceleration(self, accel):
        '''
        set the acceleration of the device

        <accel>:
            acceleration in mm/s^2

        '''
        try:
            if accel > self.max_accel_lim:
                # raise that maximum accelration is exceeded
                print('Maximum acceleration of %.2f mm/s exceeded, setting to max acceleration' % self.max_accel_lim)
                accel = self.max_accel_lim
                         
            # set new acceleration
            self.velPars.set_Acceleration(Decimal(accel))
            # update velocity params
            self.device.SetVelocityParams(self.velPars)
            # wait for motor controller to update
            time.sleep(0.1)
            
            if self.verbose:
                print('Acceleration set')

        except Exception as e:
            print(f'Unexpected Error: {e}')

    def set_velocity(self, max_vel=5, min_vel=0):
        '''
        set the velocity of the device

        <max_vel>:
            maximum velocity in mm/s
        <min_vel>:
            minimum velocity in mm/s

        '''
        try:
            if max_vel > self.max_vel_lim:
                # raise that maximum velocity is exceeded
                print(f'Maximum velocity of {self.max_vel_lim} mm/s exceeded, setting to max velocity')
                max_vel = self.max_vel_lim
            
            # set new velocity 
            self.velPars.set_MinVelocity(Decimal(min_vel))
            self.velPars.set_MaxVelocity(Decimal(max_vel))
            # update velocity params
            self.device.SetVelocityParams(self.velPars)
            # wait for motor controller to update
            time.sleep(0.1)

            if self.verbose:
                print('Velocity set')
            
        except Exception as e:
            print(f'Unexpected Error: {e}')

    def start_polling(self):
        '''
        Begin communicating with the device
        
        '''
        # Start polling and enable channel
        self.device.StartPolling(self.pol_time)
        # wait for the device to get enabled
        time.sleep(.1)
        self.device.EnableDevice()
        time.sleep(.1)

    def resetConnection(self):
        '''
        TO DO: test this...

        Reset connection to the device without having 
        to reinitilaise the class
        
        '''
        try:
            self.device.ResetConnection(self.SerialNum)
            self.device.StartPolling(self.polling)
            self.device.EnableDevice()
        except (NameError,AttributeError):
            raise Exception('No device connected')


###################################################################################################################
        
     #  TO DO: Additional functions to work on
'''
    def moveTo(self,position, time_out = 60000, callback = None):
        if callback is None:
            if position>=self.min_travel_lim and position <= self.max_travel_lim:
                self.device.MoveTo(Decimal(position),time_out)

            else:
                raise Exception('Requested position outside of device travel range')
        else:
            if position>=self.min_travel_lim and position <= self.max_travel_lim:
                self.device.MoveTo(Decimal(position),time_out,System.Action[System.Int64](callback))

            else:
                raise Exception('Requested position outside of device travel range')

    #to return immediately, set time_out=0
    #callback is an optional function that will run after the device completes the operation
    def moveRel(self, relDist, time_out = 60000, callback = None):
        if callback is None:
            if relDist>=0:
                self.device.MoveRelative(MotorDirection.Forward,Decimal(abs(relDist)),time_out)
            else:
                self.device.MoveRelative(MotorDirection.Backward,Decimal(abs(relDist)),time_out)
        else:
            if relDist>=0:
                self.device.MoveRelative(MotorDirection.Forward,Decimal(abs(relDist)),time_out,System.Action[System.Int64](callback))
            else:
                self.device.MoveRelative(MotorDirection.Backward,Decimal(abs(relDist)),time_out,System.Action[System.Int64](callback))

    def setHomingVel(self,vel):
        self.device.SetHomingVelocity(Decimal(vel))

    #to return immediately, set time_out=0
    #callback is an optional function that will run after the device completes the operation
    def go_home(self,time_out=60000, callback = None):
        if callback is None:
            if self.device.CanHome:
                self.device.Home(time_out)
            else:
                raise Exception('Device unable to home')
        else:
            if self.device.CanHome:
                self.device.Home(time_out,System.Action[System.Int64](callback))
            else:
                raise Exception('Device unable to home')

    def identify(self):
        self.device.IdentifyDevice()

    def stop(self,time_out=60000):
        self.device.Stop(time_out)

    def stopImmediate(self):
        self.device.StopImmediate()
    
#returns True if motor hasn't been homed before and doesn't know it's absolute location. Otherwise returns false   
    def needsHoming(self):
        return self.device.get_NeedsHoming()

if __name__ == "__main__":

    serial = '83860125'
    mot = KinesisMotor(serial,polling=250,verbose=True)
    mot.set_velocity(2.6)
    mot.set_acceleration(4)
    
    init_pos = mot.getPos()
    t0 = time.time()
    # mot.moveTo(init_pos+0.05)
    # mot.moveTo(init_pos+0.05*2)
    mot.moveRel(-0.1)

    print('Initial pos: ' + str(init_pos))
    print('New pos: '+str(mot.getPos()))

    t1 = time.time()-t0
    
    print('Motor movement took %.4f seconds'%t1)
    
    mot.moveTo(init_pos)
    
    
    mot.disconnect()
'''