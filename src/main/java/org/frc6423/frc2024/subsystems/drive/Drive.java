package org.frc6423.frc2024.subsystems.drive;

import static org.frc6423.frc2024.Constants.KDriveConstants.*;

import java.util.ArrayList;

import org.frc6423.frc2024.Robot;
import org.frc6423.frc2024.subsystems.drive.gyro.GyroIO;
import org.frc6423.frc2024.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import org.frc6423.frc2024.subsystems.drive.gyro.GyroIONavX;
import org.frc6423.frc2024.subsystems.drive.module.Module;
import org.frc6423.frc2024.subsystems.drive.module.ModuleIONeo;
import org.frc6423.frc2024.subsystems.drive.module.ModuleIOSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    private final StructArrayPublisher<SwerveModuleState> publisher;
    private final Field2d f2d;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final ArrayList<Module> m_Modules = new ArrayList<>();
    
    private final SwerveDriveKinematics m_DriveKinematics;
    private final SwerveDriveOdometry m_DriveOdometry;

    private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

    private final ChassisSpeeds m_SetpointSpeeds;

    private final PIDController m_FeedbackPivot;
    private final PIDController m_FeedbackDrive;

    private Rotation2d m_RobotAngle = new Rotation2d();

    public Drive(ModuleConfig[] configs) {

        f2d = new Field2d();
        publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

        gyroIO = new GyroIONavX();

        if (Robot.isReal()) {

            for(ModuleConfig config : configs) {

                m_Modules.add(
                    new Module(new ModuleIONeo(config), config.id())
                );

            }

        } else {

            for(ModuleConfig config : configs) {

                m_Modules.add(
                    new Module(new ModuleIOSim(config), config.id())
                );

            }

        }

        m_DriveKinematics = new SwerveDriveKinematics(
            kFLLocation,
            kFRLocation,
            kBLLocation,
            kBRLocation
        );

        m_DriveOdometry = new SwerveDriveOdometry(
            m_DriveKinematics, 
            new Rotation2d(), 
            getModulePositions()
        );


        m_FeedbackPivot = new PIDController(
            0,
            0,
            0
        );
        m_FeedbackDrive = new PIDController(
            0, 
            0, 
            0
        );

        m_SetpointSpeeds = new ChassisSpeeds();

    }

    @Override
    public void periodic() {

        gyroIO.updateInputs(gyroInputs);
        for (Module module : m_Modules) {

            module.periodic();

        }

        if (DriverStation.isDisabled()) {

            for (Module module : m_Modules) {

                module.stop();

            }

        }
        
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[m_Modules.size()];
        for (int moduleIndex = 0; moduleIndex < m_Modules.size(); moduleIndex++) {
            moduleDeltas[moduleIndex] =
                new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                        - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        if (gyroInputs.enabled = true) {

            m_RobotAngle = gyroInputs.yaw;

        } else {

            Twist2d twist = m_DriveKinematics.toTwist2d(moduleDeltas);
            m_RobotAngle = m_RobotAngle.plus(new Rotation2d(twist.dtheta));

        }

        m_DriveOdometry.update(m_RobotAngle, modulePositions);
        publisher.set(getModuleStates());
        f2d.setRobotPose(getPose());
        SmartDashboard.putData(f2d);

    }

    public Pose2d getPose() {

        return new Pose2d();

    }

    public Rotation2d getRotation() {

        return new Rotation2d();

    }

    private SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = new SwerveModuleState[m_Modules.size()];

        for (int i = 0; i < m_Modules.size(); i++) {

            states[i] = m_Modules.get(i).getModuleState();

        }

        return states;

    }

    private SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] positions = new SwerveModulePosition[m_Modules.size()];

        for (int i = 0; i < m_Modules.size(); i++) {

            positions[i] = m_Modules.get(i).getModulePosition();

        }

        return positions;

    }

    /**
     * Set {@link ChassisSpeeds} setpoint
     * 
     * @param speeds {@link ChassisSpeeds}
     */
    public void setSpeedsSetpoint(ChassisSpeeds speeds) {

        ChassisSpeeds desiredSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        
        // Converts speeds to Field Oriented
        desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            desiredSpeeds.vxMetersPerSecond, 
            desiredSpeeds.vyMetersPerSecond,
            desiredSpeeds.omegaRadiansPerSecond,
            new Rotation2d() // ! heading should go here
        );

    }

    // TODO add X pose stop
    public void stop() {

        setSpeedsSetpoint(new ChassisSpeeds());

    }

}
