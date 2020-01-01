package frc.robot;

public class RobotMap {
  
  //DRIVE
    public static double defaultRightControlLeftEffectiveness = .3; //between 0 & 1, higher = more turning
    public static final int leftMasterPort = 3; //CAN port number
    public static final int leftSlavePort = 0; //CAN port number
    public static final int rightMasterPort = 2; //CAN port number
    public static final int rightSlavePort = 1; //CAN port number
    public static final double defaultDriveSmooth = .5; //between 0 & 1, higher = less smoothing
    public static final double defaultDriveSpeed = 1; //between 0 & 1; higher=faster

  //ARM:
    public static double defaultArmSpeed = .5; //between 1 & ~, higher = slower 
    public static double defaultArmSmooth = .4;  //between 0 & 1, lower = more smoothing
    public static final int armUpperLimitSwitchPort = 3; //DIO port number
    public static final int armLowerLimitSwitchPort = 4; //DIO port number
    public static final int armPort = 7; //CAN port number
    public static int armlimitBypassWristLock = 6; //BUTTON number
    public static int armBreakSolenoid = 4; //PCM port number

  //EXTENTENTION
    public static double defaultExtentionSpeed = .6; //between 0 & 1, higher = faster
    public static double defaultExtentionSmooth = .25; //between 0 & 1, lower = more smoothing
    public static final int armExtentionUpperLimitSwitch = 7; //DIO port number
    public static final int armExtentionLowerLimitSwitch =0; //DIO port number
    public static final int extentionEncoderSourceA = 8; //DIO port signal A number
    public static final int extentionEncoderSourceB = 9; //DIO port signal B number
    public static final int armExtentionPort = 8; //CAN port number
    public static int buttonArmExtendPort = 5; //BUTTON number
    public static int buttonArmRetractPort = 3; //BUTTON number

    public static int wristExtentionlimitBypass = 4; //BUTTON number "joystickArmPort"
  //WRIST
    public static int defaultWristSpeed = 1; //between 1 & ~, higher = slower 
    public static double defaultWristSmooth = .2;  //between 0 & 1, lower = more smoothing
    public static final int armWristUpperLimitSwitch = 1; //DIO port number
    public static final int armWristLowerLimitSwitch = 2; //DIO port number
    public static final int wristEncoderSourceA = 5; //DIO port signal A number
    public static final int wristEncoderSourceB = 6; //DIO port signal B number
    public static final int wristPort = 6; //CAN port number

  //VACUUM
    public static double defaultVacuumPumpSpeed = 1; //between 0 & 1, higher = more  
      public static int buttonVacuumPumpOnPort = 1; //BUTTON number
      public static int buttonVacuumPumpOffPort = 2; //BUTTON number
    public static final int vacuumPumpMasterPort = 0; //CAN port number
    public static final int vacuumPumpSlavePort = 1;  //CAN port number
    public static int vacuumSolenoid = 6; //PCM port number

  //LIFT
    public static int buttonLiftRobotPort = 4; //BUTTON number
    public static int liftExtendSolenoid = 1; //PCM port number
    public static int liftRetractSolenoid = 0; //PCM port number

  //RAMPS
    public static int buttonDeployRampPort = 2; //BUTTONnumber
    public static int rampExtendSolenoid = 3; //PCM port number
    public static int rampRetractSolenoid = 2; //PCM port number

  //CAMERA
    public static int cameraFaceFrontAngle = 165; //between 0 & 180
    public static int cameraFaceBackAngle = 5; //between 0 & 180
    public static int cameraXResolution = 2000; //X resolution
    public static int cameraYResolution = 480; //Y resolution
    public static int cameraServoPort = 9; //PWM port number
    public static int buttonCameraFaceFront = 3; //BUTTON number
    public static int buttonCameraFaceBack = 2; //BUTTON number

  // OTHER
    public static int PCM = 37; //PCM ID "37"

  //Outreach Mode
    public static double defaultOutreachArmSpeed = .8; //between 0 & 1, higher = more based of value above
    public static double defaultOutreachExtentionSpeed = .8; //between 0 & 1, higher = more based of value above
    public static double defaultOutreachWristSpeed = .8; //between 0 & 1, higher = more based of value above
    public static double defaultOutreachDriveSpeed = .8; //between 0 & 1, higher = more based of value above
    public static int buttonOutreachMode = 9; //BUTTON number

  //Outreach Mode
    public static int buttonNoArm = 8; //BUTTON number
  
  // PRESET POSITIONS 
      public static int defaultPresetPositionsTimeout = 3; //Whole Number (SECONDS)
    // PORTS
      public static int joystickPresetPositionsPort = 0; //USB port number
      public static final int PickUpHatchPort = 7; //BUTTON number
      public static final int pickUpBallPort = 9; //BUTTON number
      public static final int pickUpBallOffFloorPort = 12; //BUTTON number
      public static final int placeHatchPort = 8; //BUTTON number
      public static final int placeBallInRocketPort = 10; //BUTTON number
      public static final int placeBallInCargoShipPort = 0; //BUTTON number
      public static final int armVerticlePort = 11; //BUTTON number
}