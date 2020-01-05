package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleBoard {
    public static ShuffleboardTab main = Shuffleboard.getTab("Main");
        public NetworkTableEntry outreachSpeedMultiplyer = main.add("outreachSpeedMultiplyer", .5).getEntry();
        public NetworkTableEntry outreachMode = main.add("outreachMode", false).getEntry();
        public NetworkTableEntry shooterSpeed = main.add("shooterSpeed", 1).getEntry();
        public NetworkTableEntry turnSpeed = main.add("turnSpeed", 1).getEntry();
        public NetworkTableEntry driveSpeed = main.add("driveSpeed", 1).getEntry();
    }
