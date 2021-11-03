
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.JoystickWrapper;

import java.util.Map;

import static java.util.Map.entry;

public class RobotContainer {
  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  static JoystickWrapper leftJoystick = new JoystickWrapper(Constants.leftJoystick);
  static JoystickWrapper rightJoystick = new JoystickWrapper(Constants.rightJoystick);

  private static boolean init = false;

  public RobotContainer() {
    initializeSubsystems();
    configureButtonBindings();
  }

  public static boolean getInitializationState() {
    return init;
  }

  public static void setInitializationState(boolean state) {
    init = state;
  }

  public void initializeSubsystems() {
    m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
      () -> testController.getRawAxis(1), //left y
      () -> testController.getRawAxis(0), //left x
      () -> testController.getRawAxis(2))); //right x
  }

  private void configureButtonBindings() {
      leftJoystick.invertRawAxis(1, true);
      rightJoystick.invertRawAxis(0, true);
  }

  public void disabledInit() {
    setInitializationState(true);
    m_swerveDrive.setSwerveDriveNeutralMode(true);
  }

  public void robotPeriodic() {
  }

  public void teleOpInit() {
      m_swerveDrive.resetEncoders();
      m_swerveDrive.resetOdometry();
      m_swerveDrive.setSwerveDriveNeutralMode(false);
  }

  public void teleOpPeriodic() {

  }

  public void autonomousInit() {
  }

  public void autonomousPeriodic() {
  }

}
