// package frc.robot;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.buttons.JoystickButton;
// import frc.robot.commands.DeployRamps;
// import frc.robot.commands.PresentPositions;
// import frc.robot.commands.SetCameraAngle;
// import frc.robot.commands.SetVacuum;
// import frc.robot.commands.ToggleLiftRobot;
// import frc.robot.commands.ToggleNoArm;
// import frc.robot.commands.ToggleOutreachMode;

// public class OI {
//   public boolean isOutreachMode = false;
//   public boolean isNoArm = false;

//   private Joystick manipulator = new Joystick(0);
//   private Joystick left = new Joystick(1);
//   private Joystick right = new Joystick(2);
//   private XboxController xBox = new XboxController(3);

//   public OI() {
//     button(manipulator, RobotMap.PickUpHatchPort).whenPressed(new PresentPositions(1));
//     button(manipulator, RobotMap.pickUpBallPort).whenPressed(new PresentPositions(2));
//     button(manipulator, RobotMap.pickUpBallOffFloorPort).whenPressed(new PresentPositions(3));
//     button(manipulator, RobotMap.placeHatchPort).whenPressed(new PresentPositions(4));
//     button(manipulator, RobotMap.placeBallInRocketPort).whenPressed(new PresentPositions(5));
//     button(manipulator, RobotMap.placeBallInCargoShipPort).whenPressed(new PresentPositions(6));
//     button(manipulator, RobotMap.armVerticlePort).whenPressed(new PresentPositions(7));

//     // LIFT / RAMPS
//     button(left, RobotMap.buttonLiftRobotPort).whenPressed(new ToggleLiftRobot());
//     button(left, RobotMap.buttonDeployRampPort).whenPressed(new DeployRamps());

//     // OUTREACH MODE / NO ARM
//     button(right, RobotMap.buttonOutreachMode).whenPressed(new ToggleOutreachMode());
//     button(right, RobotMap.buttonNoArm).whenPressed(new ToggleNoArm());
    
//     // CAMERA SERVO
//     button(right, RobotMap.buttonCameraFaceFront).whenPressed(new SetCameraAngle((int)Robot.ShuffleBoard.cameraFrontAngle.getDouble(RobotMap.cameraFaceFrontAngle)));
//     button(right, RobotMap.buttonCameraFaceBack).whenPressed(new SetCameraAngle((int)Robot.ShuffleBoard.cameraBackAngle.getDouble(RobotMap.cameraFaceBackAngle)));

//     // VACUUM
//     button(manipulator, RobotMap.buttonVacuumPumpOnPort).whenPressed(new SetVacuum(Robot.ShuffleBoard.vacuumPumpSpeed.getDouble(RobotMap.defaultVacuumPumpSpeed)));
//     button(manipulator, RobotMap.buttonVacuumPumpOffPort).whenPressed(new SetVacuum(0));
//   }

//   private JoystickButton button(Joystick joy, int button) {
//     JoystickButton jb = new JoystickButton(joy, button);
//     return jb;
//   }

//   // TWO DRIVE
//   public double leftDrive() {
//     return left.getY();
//   }

//   public double rightDrive() {
//     return right.getY();
//   }

//   public boolean LimelightControl() {
//     return left.getTrigger();
//   }

//   public boolean rightControl() {
//     return right.getTrigger();
//   }

//   // ONE DRIVE
//   public double oneDrive() {
//     return xBox.getY(Hand.kLeft);
//   }

//   public double oneTurn() {
//     return xBox.getX(Hand.kLeft);
//   }

//   public boolean oneQuickTurn() {
//     return xBox.getStickButton(Hand.kLeft);
//   }

//   // ARM
//   public double armControl() {
//     return manipulator.getY();
//   }

//   public double armPOV() {
//     return manipulator.getY();
//   }

//   public boolean armLimitBypass() {
//     return manipulator.getRawButton(RobotMap.armlimitBypassWristLock);
//   }

//   // EXTENTION
//   public double extentionControl() {
//     double output = 0;
//     if (manipulator.getRawButton(RobotMap.buttonArmExtendPort)) {output += 1;}
//     if (manipulator.getRawButton(RobotMap.buttonArmRetractPort)) {output -= 1;}
//     return output;
//   }

//   public boolean wristExtentionlimitBypass() {
//     return manipulator.getRawButton(RobotMap.wristExtentionlimitBypass);
//   }

//   // WRIST
//   public double wristControl() {
//     return manipulator.getTwist();
//   }
// }