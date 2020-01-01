package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.StopUpdateShuffleBoard;
import frc.robot.commands.UpdateShuffleBoard;

/**
 * Add your docs here.
 */
public class ShuffleBoard{

    public boolean update = false;
    public ShuffleboardTab GamePlay = Shuffleboard.getTab("GamePlay");
        public NetworkTableEntry outreachMode = GamePlay.add("outreachMode", false).getEntry();
        public NetworkTableEntry noArm = GamePlay.add("noArm", false).getEntry();
        
    public ShuffleboardTab Tune = Shuffleboard.getTab("Tune");
        private ShuffleboardLayout presetsTune = Tune.getLayout("presets", BuiltInLayouts.kList);
            public NetworkTableEntry presetPositionsTimeoutSeconds = presetsTune.addPersistent("presetPositionsTimeoutSeconds", 0).getEntry();
            
            private ShuffleboardLayout pickUpHatch = presetsTune.getLayout("pickUpHatch", BuiltInLayouts.kList);
                public NetworkTableEntry armPickUpHatchPosition = pickUpHatch.addPersistent("armPickUpHatchPosition", 0).getEntry();
                public NetworkTableEntry extentionPickUpHatchPosition = pickUpHatch.addPersistent("extentionPickUpHatchPosition", 0).getEntry();
                public NetworkTableEntry wristPickUpHatchPosition = pickUpHatch.addPersistent("wristPickUpHatchPosition", 0).getEntry();
                public NetworkTableEntry pipelinePickUpHatchPosition = pickUpHatch.addPersistent("pipelinePickUpHatchPosition", 0).getEntry();

            private ShuffleboardLayout pickUpBall = presetsTune.getLayout("pickUpBall", BuiltInLayouts.kList);
                public NetworkTableEntry armPickUpBallPosition = pickUpBall.addPersistent("armPickUpBallPosition", 0).getEntry();
                public NetworkTableEntry extentionPickUpBallPosition = pickUpBall.addPersistent("extentionPickUpBallPosition", 0).getEntry();
                public NetworkTableEntry wristPickUpBallPosition = pickUpBall.addPersistent("wristPickUpBallPosition", 0).getEntry();
                public NetworkTableEntry pipelinePickUpBallPosition = pickUpBall.addPersistent("pipelinePickUpBallPosition", 0).getEntry();

            private ShuffleboardLayout placeHatch = presetsTune.getLayout("placeHatch", BuiltInLayouts.kList);
                public NetworkTableEntry armPlaceHatchPosition = placeHatch.addPersistent("armPlaceHatchPosition", 0).getEntry();
                public NetworkTableEntry extentionPlaceHatchPosition = placeHatch.addPersistent("extentionPlaceHatchPosition", 0).getEntry();
                public NetworkTableEntry wristPickUpBallOffFloorPosition = placeHatch.addPersistent("wristPickUpBallOffFloorPosition", 0).getEntry();
                public NetworkTableEntry pipelinePlaceHatchPosition = placeHatch.addPersistent("pipelinePlaceHatchPosition", 0).getEntry();

            private ShuffleboardLayout pickUpBallOffFloor = presetsTune.getLayout("pickUpBallOffFloor", BuiltInLayouts.kList);
                public NetworkTableEntry armpickUpBallOffFloorPosition = pickUpBallOffFloor.addPersistent("armpickUpBallOffFloorPosition", 0).getEntry();
                public NetworkTableEntry extentionPickUpBallOffFloorPosition = pickUpBallOffFloor.addPersistent("extentionPickUpBallOffFloorPosition", 0).getEntry();
                public NetworkTableEntry wristPlaceHatchPosition = pickUpBallOffFloor.addPersistent("wristPlaceHatchPosition", 0).getEntry();
                public NetworkTableEntry pipelinePickUpBallOffFloorPosition = pickUpBallOffFloor.addPersistent("pipelinePickUpBallOffFloorPosition", 0).getEntry();

            private ShuffleboardLayout placeBallInRocket = presetsTune.getLayout("placeBallInRocket", BuiltInLayouts.kList);
                public NetworkTableEntry armPlaceBallInRocketPosition = placeBallInRocket.addPersistent("armPlaceBallInRocketPosition", 0).getEntry();
                public NetworkTableEntry extentionPlaceBallInRocketPosition = placeBallInRocket.addPersistent("extentionPlaceBallInRocketPosition", 0).getEntry();
                public NetworkTableEntry wristPlaceBallInRocketPosition = placeBallInRocket.addPersistent("wristPlaceBallInRocketPosition", 0).getEntry();
                public NetworkTableEntry pipelinePlaceBallInRocketPosition = placeBallInRocket.addPersistent("pipelinePlaceBallInRocketPosition", 0).getEntry();

            private ShuffleboardLayout placeBallInCargoShip = presetsTune.getLayout("placeBallInCargoShip", BuiltInLayouts.kList);
                public NetworkTableEntry armPlaceBallInCargoShipPosition = placeBallInCargoShip.addPersistent("armPlaceBallInCargoShipPosition", 0).getEntry();
                public NetworkTableEntry extentionPlaceBallInCargoShipPosition = placeBallInCargoShip.addPersistent("extentionPlaceBallInCargoShipPosition", 0).getEntry();
                public NetworkTableEntry wristPlaceBallInCargoShipPosition = placeBallInCargoShip.addPersistent("wristPlaceBallInCargoShipPosition", 0).getEntry();
                public NetworkTableEntry pipelinePlaceBallInCargoShipPosition = placeBallInCargoShip.addPersistent("pipelinePlaceBallInCargoShipPosition", 0).getEntry();

            private ShuffleboardLayout verticle = presetsTune.getLayout("verticle", BuiltInLayouts.kList);
                public NetworkTableEntry armVerticlePosition = verticle.addPersistent("armVerticlePosition", 0).getEntry();
                public NetworkTableEntry extentionVerticlePosition = verticle.addPersistent("extentionVerticlePosition", 0).getEntry();
                public NetworkTableEntry wristVerticlePosition = verticle.addPersistent("wristVerticlePosition", 0).getEntry();
                public NetworkTableEntry pipelineVerticlePosition = verticle.addPersistent("pipelineVerticlePosition", 0).getEntry();

        private ShuffleboardLayout driveTune = Tune.getLayout("driveTune", BuiltInLayouts.kList);
            public NetworkTableEntry driveRightControlLeftJoyEffectiveness = driveTune.addPersistent("driveRightControlLeftJoyEffectiveness", 0).getEntry();
            public NetworkTableEntry driveSpeed = driveTune.addPersistent("driveSpeed", 0).getEntry();
            public NetworkTableEntry driveSmooth = driveTune.addPersistent("driveSmooth", 0).getEntry();
            public NetworkTableEntry cheezyDrive = driveTune.addPersistent("cheezyDrive", true).getEntry();

        private ShuffleboardLayout armTune = Tune.getLayout("armTune", BuiltInLayouts.kList);
            private ShuffleboardLayout armPID = armTune.getLayout("armPID", BuiltInLayouts.kList);
                public NetworkTableEntry armP = armPID.addPersistent("armPID-P", 0).getEntry();
                public NetworkTableEntry armI = armPID.addPersistent("armPID-I", 0).getEntry();
                public NetworkTableEntry armD = armPID.addPersistent("armPID-D", 0).getEntry();
                public NetworkTableEntry armF = armPID.addPersistent("armPID-F", 0).getEntry();

            private ShuffleboardLayout armValues = armTune.getLayout("armValues", BuiltInLayouts.kList);
                public NetworkTableEntry armSpeed = armValues.addPersistent("armSpeed", 0).getEntry();
                public NetworkTableEntry armSmooth = armValues.addPersistent("armSmooth", 0).getEntry();
                public NetworkTableEntry armStartingPos = armValues.addPersistent("armStartingPos", 0).getEntry();

        private ShuffleboardLayout extentionTune = Tune.getLayout("extentionTune", BuiltInLayouts.kList);
            private ShuffleboardLayout extentionPID = extentionTune.getLayout("extentionPID", BuiltInLayouts.kList);
                public NetworkTableEntry extentionP = extentionPID.addPersistent("extentionPID-P", 0).getEntry();
                public NetworkTableEntry extentionI = extentionPID.addPersistent("extentionPID-I", 0).getEntry();
                public NetworkTableEntry extentionD = extentionPID.addPersistent("extentionPID-D", 0).getEntry();
                public NetworkTableEntry extentionF = extentionPID.addPersistent("extentionPID-F", 0).getEntry();

            private ShuffleboardLayout extentionValues = extentionTune.getLayout("extentionValues", BuiltInLayouts.kList);
                public NetworkTableEntry extentionSpeed = extentionValues.addPersistent("extentionSpeed", 0).getEntry();
                public NetworkTableEntry extentionSmooth = extentionValues.addPersistent("extentionSmooth", 0).getEntry();
                public NetworkTableEntry extentionStartingPos = extentionValues.addPersistent("extentionStartingPos", 0).getEntry();

        private ShuffleboardLayout wristTune = Tune.getLayout("wristTune", BuiltInLayouts.kList);
            private ShuffleboardLayout wristPID = wristTune.getLayout("wristPID", BuiltInLayouts.kList);
                public NetworkTableEntry wristP = wristPID.addPersistent("wristPID-P", 0).getEntry();
                public NetworkTableEntry wristI = wristPID.addPersistent("wristPID-I", 0).getEntry();
                public NetworkTableEntry wristD = wristPID.addPersistent("wristPID-D", 0).getEntry();
                public NetworkTableEntry wristF = wristPID.addPersistent("wristPID-F", 0).getEntry();
            
            private ShuffleboardLayout wristValues = wristTune.getLayout("wristValues", BuiltInLayouts.kList);
                public NetworkTableEntry wristSpeed = wristValues.addPersistent("wristSpeed", 0).getEntry();
                public NetworkTableEntry wristSmooth = wristValues.addPersistent("wristSmooth", 0).getEntry();
                public NetworkTableEntry wristStartingPos = wristValues.addPersistent("wristStartingPos", 0).getEntry();

        private ShuffleboardLayout vacuumCameraTune = Tune.getLayout("vacuumCameraTune", BuiltInLayouts.kList);
            public NetworkTableEntry vacuumPumpSpeed = vacuumCameraTune.addPersistent("vacuumPumpSpeed", 0).getEntry();
            public NetworkTableEntry cameraFrontAngle = vacuumCameraTune.addPersistent("cameraFrontAngle", 0).getEntry();
            public NetworkTableEntry cameraBackAngle = vacuumCameraTune.addPersistent("cameraBackAngle", 0).getEntry();


        private ShuffleboardLayout outreachModeTune = Tune.getLayout("outreachModeTune", BuiltInLayouts.kList);
            public NetworkTableEntry outreachModeDriveSpeed = outreachModeTune.addPersistent("outreachModeDriveSpeed", 0).getEntry();
            public NetworkTableEntry outreachModeArmSpeed = outreachModeTune.addPersistent("outreachModeArmSpeed", 0).getEntry();
            public NetworkTableEntry outreachModeExtentionSpeed = outreachModeTune.addPersistent("outreachModeExtentionSpeed", 0).getEntry();
            public NetworkTableEntry outreachModeWristSpeed = outreachModeTune.addPersistent("outreachModeWristSpeed", 0).getEntry();
            public NetworkTableEntry controllerID = outreachModeTune.addPersistent("controllerID", 0).getEntry();

            


    public ShuffleboardTab Debug = Shuffleboard.getTab("Debug");
        private ShuffleboardLayout armDebug = Debug.getLayout("armDebug", BuiltInLayouts.kList);
            public NetworkTableEntry armPercent = armDebug.add("armPercent", 0).getEntry();
            public NetworkTableEntry armOutput = armDebug.add("armOutput", 0).getEntry();
            public NetworkTableEntry armUpperHardLimit = armDebug.add("armUpperHardLimit", false).getEntry();
            public NetworkTableEntry armUpperSoftLimit = armDebug.add("armUpperSoftLimit", false).getEntry();
            public NetworkTableEntry armLowerHardLimit = armDebug.add("armLowerHardLimit", false).getEntry();
            public NetworkTableEntry armLowerSoftLimit = armDebug.add("armLowerSoftLimit", false).getEntry();
            public NetworkTableEntry armBrake = armDebug.add("armBrake", false).getEntry();
            public NetworkTableEntry armGoto = armDebug.add("armGotoAngle", false).getEntry();

        private ShuffleboardLayout extentionDebug = Debug.getLayout("extentionDebug", BuiltInLayouts.kList);
            public NetworkTableEntry extentionPercent = extentionDebug.add("extentionPercent", 0).getEntry();
            public NetworkTableEntry extentionOutput = extentionDebug.add("extentionOutput", 0).getEntry();
            public NetworkTableEntry extentionUpperHardLimit = extentionDebug.add("extentionUpperHardLimit", false).getEntry();
            public NetworkTableEntry extentionUpperSoftLimit = extentionDebug.add("extentionUpperSoftLimit", false).getEntry();
            public NetworkTableEntry extentionLowerHardLimit = extentionDebug.add("extentionLowerHardLimit", false).getEntry();
            public NetworkTableEntry extentionLowerSoftLimit = extentionDebug.add("extentionLowerSoftLimit", false).getEntry();
            public NetworkTableEntry extentionGoto = extentionDebug.add("extentionGotoAngle", false).getEntry();
            
        private ShuffleboardLayout wristDebug = Debug.getLayout("wristDebug", BuiltInLayouts.kList);
            public NetworkTableEntry wristPercent = wristDebug.add("wristPercent", 0).getEntry();
            public NetworkTableEntry wristOutput = wristDebug.add("wristOutput", 0).getEntry();
            public NetworkTableEntry wristUpperHardLimit = wristDebug.add("wristUpperHardLimit", false).getEntry();
            public NetworkTableEntry wristUpperSoftLimit = wristDebug.add("wristUpperSoftLimit", false).getEntry();
            public NetworkTableEntry wristLowerHardLimit = wristDebug.add("wristLowerHardLimit", false).getEntry();
            public NetworkTableEntry wristLowerSoftLimit = wristDebug.add("wristLowerSoftLimit", false).getEntry();
            public NetworkTableEntry wristGoto = wristDebug.add("wristGotoAngle", false).getEntry();

        private ShuffleboardLayout driveDebug = Debug.getLayout("driveDebug", BuiltInLayouts.kList);
            public NetworkTableEntry driveLeftOutput = driveDebug.add("driveLeftOutput", 0).getEntry();
            public NetworkTableEntry driveRightOutput = driveDebug.add("driveRightOutput", 0).getEntry();

        private ShuffleboardLayout pneumaticDebug = Debug.getLayout("pneumaticDebug", BuiltInLayouts.kList);
            public NetworkTableEntry solenoidLiftExtend = pneumaticDebug.add("solenoidLiftExtend", false).getEntry();
            public NetworkTableEntry solenoidLiftRetract = pneumaticDebug.add("solenoidLiftRetract", false).getEntry();
            public NetworkTableEntry solenoidRampsExtend = pneumaticDebug.add("solenoidRampsExtend", false).getEntry();
            public NetworkTableEntry solenoidRampsRetract = pneumaticDebug.add("solenoidRampsRetract", false).getEntry();
            public NetworkTableEntry solenoidBrakeRetract = pneumaticDebug.add("solenoidBrakeRetract", false).getEntry();
            public NetworkTableEntry vacuumPumpsOutput = pneumaticDebug.add("vacuumPumpsOutput", 0).getEntry();
            public NetworkTableEntry solenoidVacuumEqualizer = pneumaticDebug.add("solenoidVacuumEqualizer", false).getEntry();

        private ShuffleboardLayout pdpDebug = Debug.getLayout("pdpDebug", BuiltInLayouts.kList);
            public NetworkTableEntry pdpVoltage = pdpDebug.add("pdpVoltage", 0).getEntry();
            public NetworkTableEntry pdpTotalAmp = pdpDebug.add("pdpTotalAmp", 0).getEntry();
            public NetworkTableEntry pdpArmAmp = pdpDebug.add("pdpArmAmp", 0).getEntry();
            public NetworkTableEntry pdpExtentionAmp = pdpDebug.add("pdpExtentionAmp", 0).getEntry();
            public NetworkTableEntry pdpWristAmp = pdpDebug.add("pdpWristAmp", 0).getEntry();
            public NetworkTableEntry pdpLeftDriveAmp = pdpDebug.add("pdpLeftDriveAmp", 0).getEntry();
            public NetworkTableEntry pdpRightDriveAmp = pdpDebug.add("pdpRightDriveAmp", 0).getEntry();
            public NetworkTableEntry pdpTemperature = pdpDebug.add("pdpTemperature", 0).getEntry();

        private ShuffleboardLayout limeLightDebug = Debug.getLayout("limeLightDebug", BuiltInLayouts.kList);
            public NetworkTableEntry limelightX = limeLightDebug.add("limelightX", 0).getEntry();
            public NetworkTableEntry limelightY = limeLightDebug.add("limelightY", 0).getEntry();
            public NetworkTableEntry limelightArea = limeLightDebug.add("limelightArea", 0).getEntry();

            public void init() {
                Debug.add(new UpdateShuffleBoard());
                Debug.add(new StopUpdateShuffleBoard());
            }
}
