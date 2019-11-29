package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualExtentionControl;

public class ExtentionSubsystem extends Subsystem {

	public boolean tunable = false;

  private WPI_TalonSRX extentionMaster;

  private double extentionEncoderLowerLimit = 0; //should be 0

  private final Encoder extentionEncoder;

  private final DigitalInput extentionLowerLimitSwitch;

	public ExtentionSubsystem(boolean tunable) {
    this.tunable = tunable;
    
    //setup motor
    extentionMaster = new WPI_TalonSRX(RobotMap.armExtentionPort);
    
    // SETUP ENCODER
    extentionEncoder = new Encoder(RobotMap.extentionEncoderSourceA, RobotMap.extentionEncoderSourceB, false, EncodingType.k4X);

    // SETUP LIMITSWITCHES
    extentionLowerLimitSwitch = new DigitalInput(RobotMap.armExtentionLowerLimitSwitch);
  }

  public double getExtentionRawEncoder() {
    return extentionMaster.getSelectedSensorPosition();
  }

  public double getExtentionOutput() {
    return extentionEncoder.get();
  }

  //extention manual drive
  public void extendControl(Boolean extend, boolean retract, boolean extentionLimitBypass) {
      final double offset = .1;
      //set "extentionSpeed" to extend
      double extentionSpeed;
      if (extend) {
        extentionSpeed = RobotMap.ExtentionSpeed + offset;
      } else if (retract) {
        extentionSpeed = -RobotMap.ExtentionSpeed - offset;
      } else {
        extentionSpeed = 0;
      }

      //set extentionMove to extentionSpeed, plus calibration
      double extentionMove = ((extentionSpeed) / 2);

      if (Robot.oi.isOutreachMode) {
        extentionMove = extentionMove * RobotMap.extentionSmooth;
      }

      //extention smoothing
      double extentionFinal = getExtentionOutput() + ((extentionMove - getExtentionOutput())* RobotMap.extentionSmooth);

      //send extentionFinal to ouput
      driveMoter(extentionFinal, extentionLimitBypass);
   
    }

  // send extentionOutput to motor
  public void driveMoter(double extentionOutput, boolean extentionLimitBypass) {
    
    //sets soft upper limit
    if (getExtentionPosition() >=.95 && extentionOutput >= 0 && !extentionLimitBypass) {
      extentionOutput = (extentionOutput * (Math.abs(getExtentionPosition() - 1))); 
    }
    
    //sets soft lower limit
    if (getExtentionPosition() <=.07 && extentionOutput <= 0 && !extentionLimitBypass) {
      extentionOutput = (extentionOutput / Math.abs(getExtentionPosition() - 10)); 
    }

    //set extention max speed
    if (extentionOutput >= .6) {
      extentionOutput = .6;
    }

    // sends extentionOutput to motor
    if (!Robot.oi.isNoArm) {
      extentionMaster.set(extentionOutput);
    } else {
      extentionMaster.set(0);
    }
    
  }

  //reset lower limit
  public void extentionEncoderLowerReset(){
    boolean lowerLimit = extentionLowerLimitSwitch.get();
    if (!lowerLimit) {
      extentionEncoderLowerLimit = getExtentionRawEncoder() - 3000;  
    }

  }

  //get extention percentage
  public double getExtentionPosition() {
    double extentionPercentage = ((getExtentionRawEncoder() - extentionEncoderLowerLimit) / (110132 ));
    return extentionPercentage;
  }

  public boolean extentionSetPosition(double extentionPosition) {
    
    //if extention is not at set position
    if (Math.abs(extentionPosition - getExtentionPosition()) > .05) {
      
      //safty, so the extention can not be over, or under extended
      if (extentionPosition > 1) {
        extentionPosition = 1; 
      } else if(extentionPosition < 0){
      
      }

      //finds the diffrence between where the extention shound be, and where it is
      double extentionDifference = ((extentionPosition - getExtentionPosition()));
      
      //sets the extention max speed
      if (extentionDifference >= .7) {
        extentionDifference = .7;
      } else if (extentionDifference <=-.7) {
        extentionDifference = -.7;
      }

      //sets the extention slowest speed
      if (extentionDifference <= .2 && extentionDifference > 0) {
        extentionDifference = .2;
      } else if (extentionDifference >=-.2 && extentionDifference < 0) {
        extentionDifference = -.2;
      }

      driveMoter(extentionDifference, false);
      return true;
    //if extention is at set position
    } else {
      driveMoter(0, false);
      return false;
    }

  }

  public void stop() {
    extentionMaster.set(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualExtentionControl());
  }
  
}