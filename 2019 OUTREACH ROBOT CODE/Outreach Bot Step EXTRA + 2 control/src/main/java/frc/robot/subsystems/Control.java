package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.ControlCommand;

public class Control extends Subsystem {

  public void rumbleController(GenericHID.RumbleType type, boolean wait) {
    rumble(Robot.OI.controller , type, wait);
  }

  public void rumbleCoach(GenericHID.RumbleType type, boolean wait) {
    rumble(Robot.OI.coach , type, wait);
  }

  synchronized public void rumble(XboxController controller, GenericHID.RumbleType type, boolean wait) {

    if (wait) {
      
      controller.setRumble(type, 1);
      Timer.delay(.5);
      controller.setRumble(type, 0);
      Timer.delay(.5);    

    } else {
      
      new Thread(new Runnable() {
        public void run() {
  
          controller.setRumble(type, 1);
          Timer.delay(.5);
          controller.setRumble(type, 0);
          Timer.delay(.5);
  
        }
      }).start();
    } 
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ControlCommand());
  }
}
