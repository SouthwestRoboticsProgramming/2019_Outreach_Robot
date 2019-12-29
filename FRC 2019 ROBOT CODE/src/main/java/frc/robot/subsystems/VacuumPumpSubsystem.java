package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class VacuumPumpSubsystem extends Subsystem {
    public boolean tunable = false;

    private final WPI_VictorSPX vacuumPumpMaster, vacuumPumpSlave;
    
    private Solenoid vacuumSolenoid = new Solenoid(RobotMap.PCM, RobotMap.vacuumSolenoid);

	public VacuumPumpSubsystem(boolean tunable) {
        this.tunable = tunable;
        
        //setup motors
        vacuumPumpMaster = new WPI_VictorSPX(RobotMap.vacuumPumpMasterPort);
        vacuumPumpSlave = new WPI_VictorSPX(RobotMap.vacuumPumpSlavePort);

        vacuumPumpSlave.follow(vacuumPumpMaster);

    }

    // set pump output
    public void setVacuum(double speed) {
        vacuumPumpMaster.set(speed);
        if (vacuumPumpMaster.get() == 0) {
            airEqualizerClose();
        } else {
            airEqualizerOpen();
        }
        Robot.ShuffleBoard.vacuumPumpsOutput.setValue(vacuumPumpMaster.get());
    }

    private void airEqualizerOpen() {
        Equalizer(false);
    }
  
    private void airEqualizerClose() {
    
        Equalizer(true);
    }
  
    private void Equalizer(boolean state) {
        vacuumSolenoid.set(state);
        Robot.ShuffleBoard.solenoidVacuumEqualizer.setValue(state);
    }

    public void stop() {
      vacuumPumpMaster.set(ControlMode.PercentOutput, 0);
      vacuumPumpSlave.set(ControlMode.PercentOutput, 0);
      vacuumSolenoid.set(false);
    }

	@Override
	public void initDefaultCommand() {
    //   setDefaultCommand(new SetVacuum(0));
	}
}