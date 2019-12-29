package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class RobotClimbSubsystem extends Subsystem {
  private Solenoid robotLiftPistonExtend;
  private Solenoid robotLiftPistonRetract;

  private Solenoid rampPistonExtend;
  private Solenoid rampPistonRetract;

  // private boolean robotLifted = false;

  public RobotClimbSubsystem() {

    // SETUP SOLENOIDS
    robotLiftPistonExtend = new Solenoid(RobotMap.PCM, RobotMap.liftExtendSolenoid);
    robotLiftPistonRetract = new Solenoid(RobotMap.PCM, RobotMap.liftRetractSolenoid);
    robotLiftPistonExtend.set(false);
    robotLiftPistonRetract.set(true);

    rampPistonExtend = new Solenoid(RobotMap.PCM, RobotMap.rampExtendSolenoid);
    rampPistonRetract = new Solenoid(RobotMap.PCM, RobotMap.rampRetractSolenoid);
    rampPistonExtend.set(false);
    rampPistonRetract.set(true);
  }

  public void init() {
    robotLiftPistonExtend.set(false);
    robotLiftPistonRetract.set(true);
    rampPistonExtend.set(false);
    rampPistonRetract.set(true);
  }

  //lift robot
  public void setLiftRobot(boolean lift) {
    robotLiftPistonExtend.set(lift);
    robotLiftPistonRetract.set(!lift);
    Robot.ShuffleBoard.solenoidLiftExtend.setValue(robotLiftPistonExtend.get());
    Robot.ShuffleBoard.solenoidLiftRetract.setValue(robotLiftPistonRetract.get());
  }  

  //deploy ramps
  public void rampDeploy() {
    rampPistonExtend.set(true);
    rampPistonRetract.set(false); 
    Robot.ShuffleBoard.solenoidRampsExtend.setValue(rampPistonExtend.get());
    Robot.ShuffleBoard.solenoidRampsRetract.setValue(rampPistonRetract.get());
  
    Timer.delay(.5);

    rampPistonExtend.set(false);
    rampPistonRetract.set(true);
    Robot.ShuffleBoard.solenoidRampsExtend.setValue(rampPistonExtend.get());
    Robot.ShuffleBoard.solenoidRampsRetract.setValue(rampPistonRetract.get());
  }

  public boolean getRobotLifted() {
    return robotLiftPistonExtend.get();
  }

  @Override
  public void initDefaultCommand() {

  }
}