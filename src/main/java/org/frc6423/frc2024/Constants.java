package org.frc6423.frc2024;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    private static RobotType robotType = RobotType.SIMBOT;

    /**
     ** ----- ROBOT TYPE -----
     * DEVBOT - DriveBase with vision
     * COMPBOT - Actual Robot
     * SIMBOT - Simulated Robot
     */
    public static enum RobotType {
        DEVBOT,
        COMPBOT,
        SIMBOT
    }
    
    public static enum RobotMode {
        REAL,
        SIMULATED,
        REPLAY
    }

    public static RobotMode getRobotMode() {
        return switch (robotType) {
            case DEVBOT, COMPBOT -> Robot.isReal() ? RobotMode.REAL : RobotMode.REPLAY; 
            case SIMBOT -> Robot.isReal() ? RobotMode.REAL : RobotMode.SIMULATED;
        };
    }

    public static final class KIntakeConstants {

        public static final int kPivotMotorID = 13;
        public static final int kRollerMotorID = 14;
        public static final int kIntakeLimitSwitchID = 9; // ! Add

        public static final int kPivotABSEncoder = 5;
        public static final Rotation2d kPivotABSOffset = Rotation2d.fromDegrees(0);

        public static final double kG = 0.0; //0.58;//
        public static final double kV = 0;//0.00765858;
        public static final double kS = 0;//0.23125;
        public static final double kA = 0;//0.00086773;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final Rotation2d kPivotOutAngle = Rotation2d.fromDegrees(0);
        public static final Rotation2d kPivotInAngle = Rotation2d.fromDegrees(0);
        public static final Rotation2d kPivotSpitAngle = Rotation2d.fromDegrees(0);

        public static final double kRollerFeedSpeed = 0.0;
        public static final double kRollerFeedAmpSpeed = 0.0;
        public static final double kRollerIntakeSpeed = 0.0;
        public static final double kSuckBackSpeed = 0.0;

        public static final double kPivotMaxVelDegPerSec = 0.0;
        public static final double kPivotMaxAccelDegPerSecSqrd = 0.0;

    }
    
    public static final class KDriveConstants {

        public static final record ModuleConfig(int id, int pivotMotorID, int driveMotorID, int pivotABSEncoderID, Rotation2d pivotABSOffset) {}

        public static final double kWheelRadius = Units.inchesToMeters(4);

        // ! check in CAD
        public static final double kDriveBaseWidth = Units.inchesToMeters(25.0);
        public static final double kDriveBaseLength = Units.inchesToMeters(25.0);
        public static final double kDriveBaseRadius = Math.hypot(kDriveBaseWidth/2.0, kDriveBaseLength/2.0);

        public static final double kMaxLinearSpeed = Units.feetToMeters(10); 
        public static final double kMaxAngularSpeed = kMaxLinearSpeed/kDriveBaseRadius;

        // ! check in CAD
        public static final double kPivotGearRatio = 18/1;
        public static final double kDriveGearRatio = 6.11;

        public static final int kPivotMotorCurrentLimit = 30;
        public static final int kDriveMotorCurrentLimit = 50;

        public static final double kModulePivotP = 1;
        public static final double kModulePivotI = 0;
        public static final double kModulePivotD = 0;

        public static final double kModuleDriveP = 0;
        public static final double kModuleDriveI = 0;
        public static final double kModuleDriveD = 0;

        public static final double kDriveS = 0.02; //0.1849;
        /**volts per meter per second*/
        public static final double kDriveV = 0.02; //2.5108;
        public static final double kDriveA = 0.02; // 0.24017;
  
        // Motor and Encoder IDS for modules
        public static final int kFLPivotID = 2;
        public static final int kFLDriveID = 1;
        public static final int kFRPivotID = 4;
        public static final int kFRDriveID = 3;
        public static final int kBLPivotID = 6;
        public static final int kBLDriveID = 5;
        public static final int kBRPivotID = 8;
        public static final int kBRDriveID = 7;
        
        public static final int kFLABS = 0;
        public static final int kFRABS = 1;
        public static final int kBLABS = 2;
        public static final int kBRABS = 3;

        // Absolute encoder offsets of the swerve modules
        public static final Rotation2d kFLABSOffset = Rotation2d.fromDegrees(317);
        public static final Rotation2d kFRABSOffset = Rotation2d.fromDegrees(246);
        public static final Rotation2d kBLABSOffset = Rotation2d.fromDegrees(236);
        public static final Rotation2d kBRABSOffset = Rotation2d.fromDegrees(275);

        // ! Check CAD
        // Location of the swerve modules relative to the center of the robot
        public static final Translation2d kFLLocation = new Translation2d(0.381, 0.381);
        public static final Translation2d kFRLocation = new Translation2d(0.381, -0.381);
        public static final Translation2d kBLLocation = new Translation2d(-0.381, 0.381);
        public static final Translation2d kBRLocation = new Translation2d(-0.381, -0.381);

        public static final ModuleConfig[] kSimConfig = {
            new ModuleConfig(1, kFLPivotID, kFLDriveID, kFLABS, kFLABSOffset),
            new ModuleConfig(2, kFRPivotID, kFRDriveID, kFRABS, kFRABSOffset),
            new ModuleConfig(3, kBLPivotID, kBLDriveID, kBLABS, kBLABSOffset),
            new ModuleConfig(4, kBRPivotID, kBRDriveID, kBRABS, kBRABSOffset)
        };

    }
    
}
