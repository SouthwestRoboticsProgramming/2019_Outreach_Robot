package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.Looper.Looper;
import frc.lib.Looper.Loop;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmControl;

public class ArmSubsystem extends Subsystem {
  public boolean tunable = false;
  private boolean init = false;
  private boolean goingToPosition;

  private WPI_TalonSRX armMaster;

  private double armEncoderUpperLimit = -5; //whole number 
  private double armEncoderLowerLimit = 1720; //whole number

  private Solenoid armBrakeSolenoid;

  private DigitalInput armLowerLimitSwitch;
  private DigitalInput armUpperLimitSwitch;

  private Looper looper;

	public ArmSubsystem(boolean tunable) {
		this.tunable = tunable;

    //setup armMotor
    armMaster = new WPI_TalonSRX(RobotMap.armPort);

    // SETUP BRAKE SOLENOID
    armBrakeSolenoid = new Solenoid(RobotMap.PCM, RobotMap.armBreakSolenoid);

    // SETUP LIMITSWITCHES
    armLowerLimitSwitch = new DigitalInput(RobotMap.armLowerLimitSwitchPort);
    armUpperLimitSwitch = new DigitalInput(RobotMap.armUpperLimitSwitchPort);
  }

  public void init() {
    // armMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    armMaster.setSelectedSensorPosition(-(int)Robot.ShuffleBoard.armStartingPos.getDouble(1300));
    armMaster.configNeutralDeadband(0.001);
    armMaster.setSensorPhase(true);
    armMaster.setInverted(false);

    armMaster.config_kP(0, Robot.ShuffleBoard.armP.getDouble(0));
    armMaster.config_kI(0, Robot.ShuffleBoard.armI.getDouble(0));
    armMaster.config_kD(0, Robot.ShuffleBoard.armD.getDouble(0));
    armMaster.config_kF(0, Robot.ShuffleBoard.armF.getDouble(0));

    armMaster.configForwardSoftLimitThreshold(1300);
		armMaster.configForwardSoftLimitEnable(false);
		armMaster.configReverseSoftLimitThreshold(0);
		armMaster.configReverseSoftLimitEnable(false);

    init = true;


  }

  public double getArmRawEncoder() {
    return armMaster.getSelectedSensorPosition();
  }

  public double getArmOutput() {
    return armMaster.get();
  }

  public boolean getGoingToPosition() {
    return goingToPosition;
  }

  public void resetGoingToPosition( ) {
    goingToPosition = true;
  }
  
  //sets arm output based on "armOutput" from sends
  public void driveMoter(double armOutput, boolean armLimitBypass) {
    //upper soft limit
    if (getArmPercentage() >= .8 && armOutput >= 0 && !armLimitBypass) {
      // armOutput = (armOutput * (-getArmPercentage() + 1) * 2); 
      armOutput = Math.min(armOutput, .2);
      Robot.ShuffleBoard.armUpperSoftLimit.setValue(true);
    } else {
      Robot.ShuffleBoard.armUpperSoftLimit.setValue(false);
    }
    //upper hard limit
    if (armOutput > 0 && getArmPercentage() >= 1 && !armLimitBypass) {
      armOutput = 0;
      Robot.ShuffleBoard.armUpperHardLimit.setValue(true);
    } else {
      Robot.ShuffleBoard.armUpperHardLimit.setValue(false);
    }
    //lower soft limit
    if (getArmPercentage() <= .2 && armOutput <= 0 && !armLimitBypass) {
      armOutput = (armOutput / (Math.abs(1 - getArmPercentage()) * 7)); 
      Robot.ShuffleBoard.armLowerSoftLimit.setValue(true);
    } else {
      Robot.ShuffleBoard.armLowerSoftLimit.setValue(false);
    }
    //lower hard limit
    if (armOutput < 0 && getArmPercentage() <= 0 && !armLimitBypass) {
      armOutput = 0;
      Robot.ShuffleBoard.armLowerHardLimit.setValue(true);
    } else {
      Robot.ShuffleBoard.armLowerHardLimit.setValue(false);
    }
    //lower limit, to stop wires from being pinched, and the suction cup does not get jamed
    if (Robot.extentionSubsystem.getExtentionPosition() > .1 && Robot.extentionSubsystem.getExtentionPosition() < .9 && getArmPercentage() < .2 && armOutput <= .1) {
      armOutput = 0;
      setBrake(true);
    }
    //send "armOutput" to motors
    if (!Robot.oi.isNoArm && init) {
      armMaster.set(ControlMode.PercentOutput, armOutput);
      armBrake(armOutput);
    } else {
      armMaster.set(0);
    }
    Robot.ShuffleBoard.armOutput.setValue(armMaster.getMotorOutputPercent());
  }

  //decides when the arm break should be applyed
  public void armBrake(double armOutput) {
    if (Math.abs (armOutput) < .05 ) {
      setBrake(true);
    } else {
      setBrake(false);
    } 
  }    

  //sends "setbrake" to arm
  public void setBrake(boolean state) {
    armBrakeSolenoid.set(state);
    Robot.ShuffleBoard.armBrake.setValue(state);
    Robot.ShuffleBoard.solenoidBrakeRetract.setValue(state);
  }

  //resets arm encoder upper value when limit switch is pressed
  public void armEncoderUpperReset(boolean armBypass, double POV){
    boolean povUp;
    if (POV == 315 || POV == 0 || POV == 45) {
      povUp = true;
    } else {
      povUp = false;
    }

    if (armBypass && povUp) {
      armEncoderUpperLimit = getArmRawEncoder();
      armEncoderLowerLimit = getArmRawEncoder() + 1725;
    }

  }

  //gets the arm percentage (0-1) value from encoder
  public double getArmPercentage() {
    double armPercentage = ((armEncoderLowerLimit - getArmRawEncoder()) * (1/(armEncoderLowerLimit - armEncoderUpperLimit)));
    Robot.ShuffleBoard.armPercent.setValue(armPercentage);
    Robot.ShuffleBoard.armEncoder.setValue(getArmRawEncoder());
    return armPercentage;
  }

  private Long armAngleToTicks(double angle){
    // double unRounded = ((1725 * angle) + armEncoderUpperLimit);
    // return Math.round(unRounded);
    return Math.round((Math.abs(armEncoderLowerLimit * angle) + Math.abs(armEncoderUpperLimit * angle) * angle) + armEncoderUpperLimit);
  }


  //sets the arm's angle for preset positions
  public void armPIDToAngle(double angle) {
    System.out.println("setAngle = " + angle);
    long encodorTicks = armAngleToTicks(angle);
    int tolerance = 10;
    goingToPosition = true;
    setBrake(false);
    Loop loop = new Loop(){

      @Override public void onStart() {
        armMaster.set(ControlMode.Position, encodorTicks);
        System.out.println("start");
      }
      @Override public void onLoop() {
        if (Math.abs(armMaster.getSelectedSensorPosition() - encodorTicks) < tolerance){
          looper.stop();
        }
        System.out.println("loop");
        System.out.println("encodorTicks = " + encodorTicks + " act = " + armMaster.getSelectedSensorPosition());
      }
      @Override public void onStop() {
        System.out.println("end");
        armMaster.set(ControlMode.PercentOutput, 0);
        goingToPosition = false;
        setBrake(true);
      }
    };
    looper = new Looper(loop, 50);
    looper.start();
  }

  //stops motor
  public void stop() {
    armMaster.set(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualArmControl());
  
  }
}