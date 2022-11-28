from datetime import datetime
import spidev


class Controller():

  def __init__(self):
    # true the first time the sketch is run after the Arduino power is cycled or the reset pushbutton is pressed
    self.startup = True

    # used to store the last time the SPI data was written
    self.previousMicros = 0

    # set the sample time (the interval between SPI transactions) to 1000us = 1ms
    self.sampleTime = 1000

    # set pin 10 as the slave select for the Quanser QUBE
    # NOTE: that if a different pin is used for the slave select, pin 10 should be set as
    #       an output to prevent accidentally putting the Arduino UNO into slave mode.)
    slaveSelectPin = 10

    # initialize the SPI data to be written
    self.mode = 0b1                     # normal mode = 1
    self.writeMask = 0b00011111         # Bxxxxxx11 to enable the motor, Bxxx111xx to enable the LEDs, Bx11xxxxx to enable writes to the encoders
    self.LEDRedMSB = 0b0                 # red LED command MSB
    self.LEDRedLSB = 0b0                 # red LED command LSB
    self.LEDGreenMSB = 0b0               # green LED command MSB
    self.LEDGreenLSB = 0b0               # green LED command LSB
    self.LEDBlueMSB = 0b0                # blue LED command MSB
    self.LEDBlueLSB = 0b0                # blue LED command LSB
    self.encoder0ByteSet = [0b0, 0b0, 0b0] # encoder0 is set to this value only when writes are enabled with writeMask
    self.encoder1ByteSet = [0b0, 0b0, 0b0] # encoder1 is set to this value only when writes are enabled with writeMask
    self.motorMSB = 0x80               # motor command MSB must be B1xxxxxxx to enable the amplifier
    self.motorLSB = 0b0                  # motor command LSB

    # initialize the SPI data to be read
    self.moduleIDMSB = 0b0               # module ID MSB (module ID for the QUBE Servo is 777 decimal)
    self.moduleIDLSB = 0b0               # module ID LSB
    self.encoder0Byte = [0b0, 0b0, 0b0]   # arm encoder counts
    self.encoder1Byte = [0b0, 0b0, 0b0]   # pendulum encoder counts
    self.tach0Byte = [0b0, 0b0, 0b0]      # arm tachometer
    self.moduleStatus = 0b0              # module status (the QUBE Servo sends status = 0 when there are no errors)
    self.currentSenseMSB = 0b0           # motor current sense MSB
    self.currentSenseLSB = 0b0           # motor current sense LSB

    # Global variables for LED intensity (999 is maximum intensity, 0 is off)
    self.LEDRed = 0
    self.LEDGreen = 0
    self.LEDBlue = 0

    # Custom Global Variables
    self.setPoint = 0 # Desired angle of pendulum is 0 degree
    self.theta_n= 0.0 # Theta angle (Motor arm)
    self.alpha_n= 0.0 # Angle of the pendulam returned by the encoder
    self.theta_dot= 0.0# Angular velocity of Motor arm obtained after passing theta through Low pass filter
    self.alpha_dot= 0.0# Angular velocity of pendulam arm obtained after passing alpha through Low pass filter

    # Previous values of theta, alpha, theta_dot, alpha_dot is stored for next iteration of the loop
    self.theta_n_k1 = 0
    self.alpha_n_k1 = 0
    self.theta_dot_k1 = 0
    self.alpha_dot_k1 = 0

    # PD gains (K values) calculated from Q matrix using LQR technique
    self.kp_theta = -2.8284    # Arm Proportional Gain (Kp)
    self.kp_alpha = 41.3361    # Pendulum Proportional Gain (Kp)
    self.kd_theta = -1.7222    # Arm Derivative Gain (Kd)
    self.kd_alpha = 3.7407     # Pendulum Derivative Gain (Kd)

    self.motorVoltage = 0 # Motor voltage calculated using PD gains

    self.SPI = spidev.SpiDev()
    self.bus = 0
    self.device = 0
    self.SPI.open(self.bus, self.device)

#Setup serial builder
# Display displayData

def setup():
    # set the slaveSelectPin as an output
    #pinMode (slaveSelectPin, OUTPUT)

    # initialize SPI
    #SPI.begin()

    # initialize serial communication at 115200 baud
    # (Note that 115200 baud must be selected from the drop-down list on the Arduino
    # Serial Monitor for the data to be displayed properly.)
    #Serial.begin(115200)

def loop():
    # after the Arduino power is cycled or the reset pushbutton is pressed, call the resetQUBEServo function
    if startup:
        resetQUBEServo()
        startup = False

  # if the difference between the current time and the last time an SPI transaction
  # occurred is greater than the sample time, start a new SPI transaction
    currentMicros = datetime.now().microsecond()

    if currentMicros - previousMicros >= sampleTime:
      previousMicros = previousMicros + sampleTime

      # initialize the SPI bus using the defined speed, data order and data mode
      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2))

      # take the slave select pin low to select the device
      digitalWrite(slaveSelectPin, LOW)

      # send and receive the data via SPI (except for the motor command, which is sent after the custom code)
      moduleIDMSB = SPI.xfer(mode)                    # read the module ID MSB, send the mode
      moduleIDLSB = SPI.xfer(0)                       # read the module ID LSB
      encoder0Byte[2] = SPI.xfer(writeMask)           # read encoder0 byte 2, send the write mask
      encoder0Byte[1] = SPI.xfer(LEDRedMSB)           # read encoder0 byte 1, send the red LED MSB
      encoder0Byte[0] = SPI.xfer(LEDRedLSB)           # read encoder0 byte 0, send the red LED LSB
      encoder1Byte[2] = SPI.xfer(LEDGreenMSB)         # read encoder1 byte 2, send the green LED MSB
      encoder1Byte[1] = SPI.xfer(LEDGreenLSB)         # read encoder1 byte 1, send the green LED LSB
      encoder1Byte[0] = SPI.xfer(LEDBlueMSB)          # read encoder1 byte 0, send the blue LED MSB
      tach0Byte[2] = SPI.xfer(LEDBlueLSB)             # read tachometer0 byte 2, send the blue LED LSB
      tach0Byte[1] = SPI.xfer(encoder0ByteSet[2])     # read tachometer0 byte 1, send encoder0 byte 2
      tach0Byte[0] = SPI.xfer(encoder0ByteSet[1])     # read tachometer0 byte 0, send encoder0 byte 1
      moduleStatus = SPI.xfer(encoder0ByteSet[0])     # read the status, send encoder0 byte 0
      currentSenseMSB = SPI.xfer(encoder1ByteSet[2])  # read the current sense MSB, send encoder1 byte 2
      currentSenseLSB = SPI.xfer(encoder1ByteSet[1])  # read the current sense LSB, send encoder1 byte 1
      SPI.xfer(encoder1ByteSet[0])                    # send encoder1 byte 0

      # combine the received bytes to assemble the sensor values
      # Module ID
      moduleID = (moduleIDMSB << 8) | moduleIDLSB

      # Motor Encoder Counts
      encoder0 = (encoder0Byte[2] << 16) | (encoder0Byte[1] << 8) | encoder0Byte[0]
      if (encoder0 & 0x00800000):
        encoder0 = encoder0 | 0xFF000000

      # convert the arm encoder counts to angle theta in radians
      theta = encoder0 * (-2.0 * M_PI / 2048)

      # Pendulum Encoder Counts
      encoder1 = (encoder1Byte[2] << 16) | (encoder1Byte[1] << 8) | encoder1Byte[0]

      if (encoder1 & 0x00800000):
        encoder1 = encoder1 | 0xFF000000

      # wrap the pendulum encoder counts when the pendulum is rotated more than 360 degrees
      encoder1 = encoder1 % 2048
      if (encoder1 < 0):
        encoder1 = 2048 + encoder1

      # convert the pendulum encoder counts to angle alpha in radians
      alpha = encoder1 * (2.0 * M_PI / 2048) - M_PI

      # Current Sense Value
      currentSense = (currentSenseMSB << 8) | currentSenseLSB

      # The controller works if the alpha (pendulum angle) is between -10 and +10 degrees
      if alpha >= -10*M_PI/180 and alpha <= 10*M_PI/180:

          # xfer function = 50s/(s+50)
          # z-transform at 1ms = (50z -  50)/(z-0.9512)

          theta_n = setPoint - theta
          alpha_n = setPoint - alpha

          theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.9512 * theta_dot_k1)
          theta_n_k1 = theta_n
          theta_dot_k1 = theta_dot

          alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.9512 * alpha_dot_k1)
          alpha_n_k1 = alpha_n
          alpha_dot_k1 = alpha_dot

          # Switch on the green LED to indicate the controller is stable
          LEDGreen = 999
          LEDRed = 0
          LEDBlue = 0

          # multiply the proportional and derivative gains to the respective alpha and theta values
          motorVoltage = - (theta * kp_theta) - (alpha * kp_alpha) + (theta_dot * kd_theta) + (alpha_dot * kd_alpha)


      else: # Switch on the red LED if the alpha (pendulum)  angle is not inbetween -10 and +10 degrees
          motorVoltage = 0
          LEDRed = 999
          LEDGreen = 0


     # set the saturation limit to +/- 10V
      if (motorVoltage > 10 ): # if condition activates if the motor voltage is more than 10
        motorVoltage = 10
      elif ( motorVoltage < -10 ): # if condition activates if the motor voltage is less than -10
        motorVoltage = -10

      # convert the LED intensities to MSB and LSB

      LEDRedMSB = bytes(LEDRed >> 8)
      LEDRedLSB = bytes(LEDRed & 0x00FF)
      LEDGreenMSB = bytes(LEDGreen >> 8)
      LEDGreenLSB = bytes(LEDGreen & 0x00FF)
      LEDBlueMSB = bytes(LEDBlue >> 8)
      LEDBlueLSB = bytes(LEDBlue & 0x00FF)

      #Invert motor voltage to provide balancing force in opposite direction.
      motorVoltage=-motorVoltage


      # convert the analog value to the PWM duty cycle that will produce the same average voltage
      motorPWM = motorVoltage * (625.0 / 15.0)

      motor = int(motorPWM)  # convert float to int (2 bytes)
      motor = motor | 0x8000  # motor command MSB must be B1xxxxxxx to enable the amplifier
      motorMSB = bytes(motor >> 8)
      motorLSB = bytes(motor & 0x00FF)

      # send the motor data via SPI
      SPI.xfer(motorMSB)
      SPI.xfer(motorLSB)

      # take the slave select pin high to de-select the device
      digitalWrite(slaveSelectPin, HIGH)
      SPI.endTransaction()

      displayData.buildString(theta, alpha, currentSense, moduleID, moduleStatus)

    # print data to the Arduino Serial Monitor in between SPI transactions
    # (Note that the Serial.print() function is time consuming.  Printing the entire
    # string at once would exceed the sample time required to balance the pendulum.)
    else:
      # only print if there's a string ready to be printed, and there's enough time before the next SPI transaction
      if ( (displayData.dDataReady) and (currentMicros - previousMicros <= (sampleTime - 100)) ) :
        # if there is room available in the serial buffer, print one character
        if (Serial.availableForWrite() > 0) :
          Serial.print(displayData.dData[displayData.dDataIndex])
          displayData.dDataIndex = displayData.dDataIndex + 1
          # if the entire string has been printed, clear the flag so a new string can be obtained
          if (displayData.dDataIndex == displayData.dData.length()) :
            displayData.dDataReady = false





# This function is used to clear the stall error and reset the encoder values to 0.
# The motor and LEDs are turned off when this function is called.
def resetQUBEServo():

  # enable the motor and LEDs, and enable writes to the encoders
  writeMask = 0b01111111

  # turn off the LEDs
  LEDRedMSB = 0
  LEDRedLSB = 0
  LEDGreenMSB = 0
  LEDGreenLSB = 0
  LEDBlueMSB = 0
  LEDBlueLSB = 0

  # reset the encoder values to 0
  encoder0ByteSet[2] = 0
  encoder0ByteSet[1] = 0
  encoder0ByteSet[0] = 0
  encoder1ByteSet[2] = 0
  encoder1ByteSet[1] = 0
  encoder1ByteSet[0] = 0

  # turn off the motor, and clear the stall error by disabling the amplifier
  motorMSB = 0  # motor command MSB is B0xxxxxxx to disable the amplifier
  motorLSB = 0

  # initialize the SPI bus using the defined speed, data order and data mode
  # SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2))
  # digitalWrite(slaveSelectPin, HIGH)  # take the slave select pin high to de-select the device
  # digitalWrite(slaveSelectPin, LOW)   # take the slave select pin low to select the device

  # send and receive the data via SPI
  moduleIDMSB = SPI.xfer(mode)                    # read the module ID MSB, send the mode
  moduleIDLSB = SPI.xfer(0)                       # read the module ID LSB
  encoder0Byte[2] = SPI.xfer(writeMask)           # read encoder0 byte 2, send the write mask
  encoder0Byte[1] = SPI.xfer(LEDRedMSB)           # read encoder0 byte 1, send the red LED MSB
  encoder0Byte[0] = SPI.xfer(LEDRedLSB)           # read encoder0 byte 0, send the red LED LSB
  encoder1Byte[2] = SPI.xfer(LEDGreenMSB)         # read encoder1 byte 2, send the green LED MSB
  encoder1Byte[1] = SPI.xfer(LEDGreenLSB)         # read encoder1 byte 1, send the green LED LSB
  encoder1Byte[0] = SPI.xfer(LEDBlueMSB)          # read encoder1 byte 0, send the blue LED MSB
  tach0Byte[2] = SPI.xfer(LEDBlueLSB)             # read tachometer0 byte 2, send the blue LED LSB
  tach0Byte[1] = SPI.xfer(encoder0ByteSet[2])     # read tachometer0 byte 1, send encoder0 byte 2
  tach0Byte[0] = SPI.xfer(encoder0ByteSet[1])     # read tachometer0 byte 0, send encoder0 byte 1
  moduleStatus = SPI.xfer(encoder0ByteSet[0])     # read the status, send encoder0 byte 0
  currentSenseMSB = SPI.xfer(encoder1ByteSet[2])  # read the current sense MSB, send encoder1 byte 2
  currentSenseLSB = SPI.xfer(encoder1ByteSet[1])  # read the current sense LSB, send encoder1 byte 1
  SPI.xfer(encoder1ByteSet[0])                    # send encoder1 byte 0
  SPI.xfer(motorMSB)                              # send the motor MSB
  SPI.xfer(motorLSB)                              # send the motor LSB

  digitalWrite(slaveSelectPin, HIGH)  # take the slave select pin high to de-select the device
  SPI.endTransaction()

  writeMask = 0b00011111  # enable the motor and LEDs, disable writes to the encoders
  motorMSB = 0x80  # enable the amplifier
