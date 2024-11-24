package org.frc6423.frc2024;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    private static RobotType robotType = RobotType.SIMBOT;
    
    public static RobotType getRobotType() {
        if (Robot.isReal() && robotType == robotType.SIMBOT) {
            System.out.println("ERROR: invalid robot type");
            return RobotType.COMPBOT;
        } 
        return robotType;
    }

    public static RobotMode getRobotMode() {
        return switch (robotType) {
            case DEVBOT, COMPBOT -> Robot.isReal() ? RobotMode.REAL : RobotMode.REPLAY;
            case SIMBOT -> RobotMode.SIMULATED;
        };
    }

    public static enum RobotMode {
        REAL,
        SIMULATED,
        REPLAY
    }

    /**
     ** ----- ROBOT TYPE -----
     * DEVBOT - DriveBase with vision
     * SIMBOT - Simulation Robot
     * COMPBOT - Actual Robot
     */
    public static enum RobotType {
        SIMBOT,
        DEVBOT,
        COMPBOT
    }

    public static final class KDriveConstants {

        public static final record ModuleConfig(int pivotMotorID, int driveMotorID, int pivotABSEncoderID) {}

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

        public static final Rotation2d kFLABSOffset = Rotation2d.fromDegrees(317);
        public static final Rotation2d kFRABSOffset = Rotation2d.fromDegrees(246);
        public static final Rotation2d kBLABSOffset = Rotation2d.fromDegrees(236);
        public static final Rotation2d kBRABSOffset = Rotation2d.fromDegrees(275);

    }
    
}
