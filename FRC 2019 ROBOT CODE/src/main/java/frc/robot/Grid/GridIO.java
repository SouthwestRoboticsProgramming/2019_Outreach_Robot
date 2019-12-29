package frc.robot.Grid;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.sensors.ADIS16448_IMU;
import jdk.jfr.Description;

/**
 * Add your docs here.
 */
public class GridIO extends Subsystem {
  private double positionX = 0;
  private double positionY = 0;
  private double acceleration = 1;
  private double deceleration = 1;
  private double accelDistence;
  private double decelDistence;


  private boolean positionUpdate = true;
  private int pivot = 0;
  private double encodorGyroRatio = .5;
  private double angleOffset = 0;
  private int gyroPlane = 0;
  private double maxSpeed = 1;
  private double minSpeed = 0;
  private double weelBase = 0;

  private double setAngle = 0;
  private double setLeft = 0;
  private double setRight = 0;
  
  /**
   * Sets the weel base of the robot
   *
   * @param distence the distence between the centers of the weels on a robot in feet.
   */
  public void setWeelBase(double distence) {
    weelBase = distence;
  }
  
  /**
   * Sets the position of the robot.
   *
   * @param positionX the X position of the robot in feet from drivers station 1 corner to center of robot. X axis is short side of field;
   *  @param positionY the Y position of the robot in feet from drivers station 1 corner to center of robot. Y axis is long side of field;
   */
  public void setPosition(double positionX, double positionY) {
    this.positionX = positionX;
    this.positionY = positionY;
  }

   /**
   * Sets the max robot acceleration.
   *
   * @param seconds time in second to accelerate from 0% to 100% (Assuming 20 millisecond update rate)
   */
  public void setAcceleration(double seconds) {
    this.acceleration = seconds;
  }

  /**
   * Sets the max robot deceleration.
   *
   * @param seconds time in second to decelerate from 100% to 0% (Assuming 20 millisecond update rate)
   */
  public void setDeceleration(double seconds) {
    this.deceleration = seconds;
  }

  /**
   * Sets the maximum output to the motors.
   *
   * @param speed between 0 & 1;
   */
  public void setMaxSpeed(double speed) {
    maxSpeed = speed;
  }

  /**
   * Sets the minimum output to the motors, so that the robot still moves.
   *
   * @param speed between 0 & 1;
   */
  public void setMinSpeed(double speed) {
    minSpeed = speed;
  }

  /**
   * Sets robot to pivot whenturning on left side.
   */
  public void setPivotLeft() {
    pivot = 0;
  }

  /**
   * Sets robot to pivot when turning on right side.
   */
  public void setPivotRight() {
    pivot = 1;
  }

  /**
   * Sets which plane of the gyro is used for the robots angle.
   *
   * @param plane "x" = X axis. "y" = Y axis. "z" = Z axis. 
   */
  public void setXPlaneGyroPlane(String plane) {
    if (plane.equalsIgnoreCase("x")) {
      gyroPlane = 0;
    } else if (plane.equalsIgnoreCase("x")){
      gyroPlane = 1;
    } else if (plane.equalsIgnoreCase("x")) {
      gyroPlane = 2;
    }
  }

  private double getAngle() {
    double angle = 0;
    if (gyroPlane == 0) {
      // angle = Robot.Gyro.getAngleX();
    } else if (gyroPlane == 1){
      // angle = Robot.Gyro.getAngleY();
    } else if (gyroPlane == 2) {
      // angle =  Robot.Gyro.getAngleZ();
    } else {
      return 0;
    }
    return angle + angleOffset;
  }

  /**
   * Sets the current physical angle of the robot.
   *
   * @param angle current angle in degrees. between -180 & 180. 
   */
  public void setCurentAngle(double angle) {
    angleOffset = angle - getAngle();
  }

  /**
   * Distences found using "getAccelDecelDistence()"
   *
   * @param accelerationDistence distence robot takes to accelerate from 0% to 100%
   * @param decelerationDistence distence robot takes to decelerate from 100% to 0%
   */
  public void setAccelDecelDistence(double accelerationDistence, double decelerationDistence) {
    accelDistence = accelerationDistence;
    decelDistence = decelerationDistence;
  }

  /**
   * WARNING robot will move forward.
   * 
   * Returns the the distence the robot takes to accelerate to 100% power to the wheels.
   * 
   * use "System.out.print(getAccelDecelDistence());", then run "setAccelDecelDistence(double acceleration, double deceleration)".
   *
   * @return String acceleration & deceleration distence.
   */
  public String getAccelDecelDistence() {
    double leftAccelerateBase = Robot.driveSubsystem.getLeftDriveFeet();
    double rightAccelerateBase = Robot.driveSubsystem.getRightDriveFeet();
    setMotorOutput(1, 1);
    double leftAccelerated = Robot.driveSubsystem.getLeftDriveFeet();
    double rightAccelerated = Robot.driveSubsystem.getRightDriveFeet();
    Timer.delay(1);
    double leftDecelerateBase = Robot.driveSubsystem.getLeftDriveFeet();
    double rightDecelerateBase = Robot.driveSubsystem.getRightDriveFeet();
    setMotorOutput(0, 0);
    double leftDecelerated = Robot.driveSubsystem.getLeftDriveFeet();
    double rightDecelerated = Robot.driveSubsystem.getRightDriveFeet();
    double acceleration = ((leftAccelerated - leftAccelerateBase) + (rightAccelerated - rightAccelerateBase)) / 2;
    double deceleration = ((leftDecelerated - leftDecelerateBase) + (rightDecelerated - rightDecelerateBase)) / 2;
    return "acceleration = " + acceleration + " | deceleration = " + deceleration;
  }

  /**
   * Moves the robot based on time, not position. 
   *
   * @param leftPercent percent power being sent to left drive. between -1 & 1.
   * @param rightPercent percent power being sent to right drive. between -1 & 1.
   * @param seconds amount of time in seconds the power will be applied to weels. Acceneration, decelleration is not factored into the time, but is present. 
   */
  public void moveByTime(double leftPercent, double rightPercent, double seconds) {
      setMotorOutput(leftPercent, rightPercent);
      Timer.delay(seconds);
  }

  /**
   * Moves the robot to (x,y) position on field.
   *
   * @param X x position. 
   * @param Y y position.
   */
  public void trackPosition() {
    
  }

  /**
   * Moves the robot to (x,y) position on field.
   *
   * @param X x position. 
   * @param Y y position.
   */
  public void gotoPosition(double setX, double setY) {
    double weelBase = 0;
    double halfWeelBase = weelBase/2;
    double leftWheelX = weelBase/2 * Math.sin(getAngle() - 90);
    double leftWheelY = weelBase/2 * Math.cos(getAngle() - 90);
    double rightWheelX = weelBase/2 * Math.sin(getAngle() - 90);
    double rightWheelY = weelBase/2 * Math.cos(getAngle() + 90);
    double leftToSet = Math.sqrt((setY-leftWheelY) + (setX-leftWheelX));
    double rightToSet = Math.sqrt((setY-rightWheelY) + (setX-rightWheelX));
    double D = Math.sqrt((setX - positionX) + (setY - positionY));
    double x = (halfWeelBase * (D + halfWeelBase - weelBase)) / (weelBase - halfWeelBase);
    double h = D + halfWeelBase + x;
    double angleSin = weelBase / h;
    double d = Math.sqrt((h*h) - (halfWeelBase*halfWeelBase));
    double dRise = setY - positionY;
    double dRun = setX - positionX;
    double a = Math.atan((dRise/dRun));
    double c = 90 - angleSin;
    double C = 180 - (a + c);
    double t = (Math.PI * 2 * weelBase) * (360-C);
    double T = (halfWeelBase / weelBase) * d;
    if (leftToSet < rightToSet) {
      setLeftDistence(t, C);
    } else {
      setRightDistence(t, C);
    }
    driveDistence(T);
  }

  /**
   * turns robot to specified angle. 
   * 
   * Moves the robot to (x,y) position on field.
   *
   * @param angle between -180 & 180.
   */
  public void turnToAngle(double angle) {
 
  }

  /**
   * drives the robot straight for specified distence.
   *
   * @param feet feet robot moves
   */
  public void driveDistence(double feet) {

  }

  /**
   * drives the left motor for specified distence.
   *
   * @param feet feet robot moves
   */
  public void setLeftDistence(double feet, double angle) {

  }

  /**
   * drives the right motor for specified distence.
   *
   * @param feet feet robot moves
   */
  public void setRightDistence(double feet, double angle) {

  }

  private void setMotorOutput(double left, double right) {
    // left
    double leftSmoothing;
    if (Robot.driveSubsystem.getLeftOutput() < left) {
      leftSmoothing = acceleration / 50;
    } else {
      leftSmoothing = -deceleration / 50;
    }

    // right
    double rightSmoothing;
    if (Robot.driveSubsystem.getLeftOutput() < right) {
      rightSmoothing = acceleration / 50;
    } else {
      rightSmoothing = -deceleration / 50;
    }

    double rightOutput = 0;
    double leftOutput = 0;
    while (Math.abs(leftOutput - left) > .02 && Math.abs(rightOutput - right) > .02) {
      Robot.driveSubsystem.driveMotors(leftOutput, rightOutput);
      rightOutput = rightOutput + rightSmoothing;
      leftOutput = leftOutput + leftSmoothing;
      Timer.delay(.02);
    }    
    Robot.driveSubsystem.driveMotors(left, right);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
