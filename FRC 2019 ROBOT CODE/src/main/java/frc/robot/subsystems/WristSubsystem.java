package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.PID;
import frc.lib.TimeOutTimer;
import frc.lib.Looper.Loop;
import frc.lib.Looper.Looper;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualWristControl;

public class WristSubsystem extends Subsystem {
  public boolean tunable = false;
  private boolean init = false;
  private boolean goingToPosition;

  private WPI_TalonSRX wristMaster;

  private double wristOffset;

  private final Encoder wristEncoder;

  private final DigitalInput wristLowerLimitSwitch;
  private final DigitalInput wristUpperLimitSwitch;

  private PID pid;
  private Looper looper;

  public WristSubsystem(boolean tunable) {
  this.tunable = tunable;
    //setup armMotor
    wristMaster = new WPI_TalonSRX(RobotMap.wristPort);

    // SETUP ENCODER
    wristEncoder = new Encoder(RobotMap.wristEncoderSourceA, RobotMap.wristEncoderSourceB, false, EncodingType.k4X);

    // SETUP LIMITSWITCHES
    wristLowerLimitSwitch = new DigitalInput(RobotMap.armWristLowerLimitSwitch);
    wristUpperLimitSwitch = new DigitalInput(RobotMap.armWristUpperLimitSwitch);
  }

  public void init() {
    wristMaster.setSelectedSensorPosition((int)Robot.ShuffleBoard.wristStartingPos.getDouble(0));
    double kp = Robot.ShuffleBoard.extentionP.getDouble(0);
    double ki = Robot.ShuffleBoard.extentionP.getDouble(0);
    double kd = Robot.ShuffleBoard.extentionP.getDouble(0);
    pid = new PID(kp, ki, kd, "extention");
    init = true;
  }

  public void calibrateWrist() {
    TimeOutTimer timer = new TimeOutTimer(1500);
    double previousPosition = getWristPosition();
    wristMaster.set(1);
    timer.start();
    while (wristUpperLimitSwitch.get() && !timer.getTimedOut()) {}
    timer.reset();
    wristMaster.set(-1);
    timer.start();
    while (wristLowerLimitSwitch.get() && !timer.getTimedOut()) {}
    timer.stop();
    timer.reset();
    wristMaster.stopMotor();
    double currentPosition = getWristPosition();
    wristOffset = previousPosition - currentPosition + .1;
    // wristMaster.set(.1);
    // while (wristPosition() < wristOffset) {
    // System.out.println("wristPosition = " + wristPosition() + " wrist offset = " + wristOffset);
    // }
    // wristMaster.stopMotor();
  }

  public double getWristOutput() {
    return wristMaster.get();
  }

  public boolean getGoingToPosition() {
    return goingToPosition;
  }

  public void resetGoingToPosition( ) {
    goingToPosition = true;
  }
  
  //sets arm output based on "armOutput" from sends
  public void driveMoter(double wristOutput, boolean wristLimitBypass) {
    
    //upper soft limit
    // if (getWristPosition() >= .8 && wristOutput >= 0 && wristLimitBypass) {
    //     wristOutput = (wristOutput / (getWristPosition() * 2)); 
    //     if (getWristPosition() >= 1) {
    //       wristOutput = 0;
    //       Robot.ShuffleBoard.wristUpperSoftLimit.setValue(true);
    //     } else {
    //       Robot.ShuffleBoard.wristUpperSoftLimit.setValue(false);
    //     }
    // }

    // lower soft limit
    // if (getWristPosition() <= .2 && wristOutput <= 0 && wristLimitBypass) {
    //   wristOutput = (wristOutput / (Math.abs(1 - getWristPosition()) * 2)); 
    //   if (getWristPosition() <= 0) {
    //     wristOutput = 0;
    //     Robot.ShuffleBoard.wristLowerSoftLimit.setValue(true);
    //   } else {
    //     Robot.ShuffleBoard.wristLowerSoftLimit.setValue(false);
    //   }
    // }

    //send "armOutput" to motors
    if (!Robot.oi.isNoArm || init) {
      wristMaster.set(wristOutput);
    } else {
      wristMaster.set(0);
    }
    Robot.ShuffleBoard.wristOutput.setValue(wristMaster.getMotorOutputPercent());
  }

  //sets default wrist upper encoder values
  public void wristEncoderUpperReset() {
    // boolean upperLimitSwitch = !wristUpperLimitSwitch.get();
    //   if (upperLimitSwitch && !wristUpperSet) {
    //       wristOffset = -wristRawEncoder + SmartDashboard.getNumber("wristEncoderTravelDistence", 0);
    //       wristUpperSet = true;
    //   } else {
    //       wristUpperSet = false;
    //   }
  }

  // sets default wrist lower encoder values
  public void wristEncoderLowerReset() {
  //   boolean lowerLimitSwitch = !wristLowerLimitSwitch.get();
  //     if (lowerLimitSwitch && !wristLowerSet) {
  //         wristOffset = -wristRawEncoder;
  //         wristLowerSet = true;
  //     } else {
  //         wristLowerSet = false;
  //     }
  }

  //get wrist percentage
  public double getWristPosition() {
    double wristRawEncoder = wristEncoder.get();
    double wristPercentage = ((wristRawEncoder / 650) + wristOffset);
    Robot.ShuffleBoard.wristPercent.setValue(wristPercentage);
    return wristPercentage;
  }

  //sets the arm's angle for preset positions
  public boolean wristSetAngle(double wristPosition) {
    if (Math.abs(wristPosition - getWristPosition()) > .02) {
      //safty, incase the arm is set to o past 100%
      if (wristPosition > 1) {
        wristPosition = 1; 
      } else if(wristPosition < 0){
          wristPosition = 0; 
      }

      //finds the diffrence between where the arm is set to go, and where it is
      double wristDifference = ((wristPosition - getWristPosition()) * 3 );

      //does not allow the arm to move faseer than .6
      if (wristDifference >= .4) {
        wristDifference = .4;
      } else if (wristDifference <=-.4) {
        wristDifference = -.4;
      }

      //does not allow the arm to move slower than .1, so it can make it to its end point
      if (wristDifference <= .2 && wristDifference > 0) {
        wristDifference = .2;
      } else if (wristDifference >=-.2 && wristDifference < 0) {
        wristDifference = -.2;
      }

      //sends the arm's power to the moter
      driveMoter(wristDifference, false);
      Robot.ShuffleBoard.wristGoto.setValue(true);
      return true;
    //called when the arm is at set position
    } else {
      driveMoter(0, false);
      Robot.ShuffleBoard.wristGoto.setValue(false);
      return false;
    }
  }

  public void wristPIDToAngle(double wristPercent) {
    int tolerance = 1;
    goingToPosition = false;
    Loop loop = new Loop(){
      @Override public void onStart() {
        pid.setSetPoint(wristPercent);
      }
      @Override public void onLoop() {
        if(Math.abs(getWristPosition() - wristPercent) < tolerance) {
          looper.stop();
        }
        pid.setActual(getWristPosition());
        driveMoter(pid.getOutput(), false);
      }
      @Override public void onStop() {
        driveMoter(0, false);
        goingToPosition = false;
      }
    };
    looper = new Looper(loop, 50);
    looper.start();
  }

  //stops motor
  public void stop() {
    wristMaster.set(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualWristControl());
  }
}
