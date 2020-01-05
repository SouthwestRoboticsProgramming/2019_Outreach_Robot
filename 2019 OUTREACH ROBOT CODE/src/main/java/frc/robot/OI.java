package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
  public int joystick = 0; //which joystick to use
  public XboxController controller = new XboxController(0); // joystick ID = 1
  public XboxController coach = new XboxController(1); // joystick ID = 2
  
  public OI() {
  }
}
