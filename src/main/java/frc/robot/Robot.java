// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.team1699.Constants.InputConstants;
import frc.team1699.subsystems.Swerve;
import frc.team1699.subsystems.Swerve.DriveState;

public class Robot extends TimedRobot {
  private Swerve swerve;
  private XboxController driveController;

  @Override
  public void robotInit() {
    driveController = new XboxController(InputConstants.kDriverControllerID);
    swerve = new Swerve(driveController);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    swerve.update();
  }

  @Override
  public void teleopInit() {
    swerve.setWantedState(DriveState.TELEOP);
  }

  @Override
  public void teleopPeriodic() {
    swerve.update();
  }

  @Override
  public void disabledInit() {
    swerve.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
