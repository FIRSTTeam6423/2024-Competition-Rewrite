package org.frc6423.frc2024.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerUtil {

  public static class IronController extends CommandXboxController {

    public final double joystickDeadband, axisDeadband;

    /**
     * Creates a new Iron Controller
     *
     * @param port port controller is connected too
     * @param joystickDeadband Joystick deadzone
     * @param triggerDeadband Trigger deadzone
     */
    public IronController(int port, double joystickDeadband, double triggerDeadband) {
      super(port);

      this.joystickDeadband = joystickDeadband;
      this.axisDeadband = triggerDeadband;
    }

    /**
     * Joystick deadband
     *
     * @param rawAxis
     * @return double
     */
    public double joystickDeadbandOutput(int rawAxis) {
      return MathUtil.applyDeadband(
          Math.abs(Math.pow(super.getRawAxis(rawAxis), 2)) * Math.signum(super.getRawAxis(rawAxis)),
          joystickDeadband);
    }

    /**
     * Trigger deadband
     *
     * @param rawAxis
     * @return double
     */
    public double triggerDeadbandOutput(int rawAxis) {
      return MathUtil.applyDeadband(super.getRawAxis(rawAxis), axisDeadband);
    }

    /**
     * Returns the circular angle of the joystick
     *
     * @param yAxis
     * @return {@link Rotation2d}
     */
    public Rotation2d flickStickOutput(int yAxis, int xAxis) {
      double y = joystickDeadbandOutput(yAxis);
      double x = joystickDeadbandOutput(xAxis);
      return (y == 0.0 && x == 0.0) ? new Rotation2d(): Rotation2d.fromRadians(Math.atan2(y, x));
    }

    /**
     * Rumble Controller
     *
     * @param rmb
     * @param n
     * @return {@link Command}
     */
    public Command rumbleController(GenericHID.RumbleType rmb, double n) {
      return new InstantCommand(() -> super.getHID().setRumble(rmb, n));
    }

  }
}