package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class CameraServoSubsystem extends Subsystem {
  public boolean tunable = false;

  private Servo cameraServo;
  
  public CameraServoSubsystem(boolean tunable){
    this.tunable = tunable;

    // SETUP SERVO
    cameraServo = new Servo(RobotMap.cameraServoPort);
  }

  public void init() {
    setCameraAngle((int)Robot.ShuffleBoard.cameraFrontAngle.getDouble(0));
  }

  public void setCameraAngle(int angle) {
      cameraServo.setAngle(angle);
  }

  @Override
  public void initDefaultCommand() {
  }
}