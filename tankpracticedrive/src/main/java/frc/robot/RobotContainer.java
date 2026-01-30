// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Spit;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Roller;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(Constants.driveControllerId);
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  private final Drivetrain drivetrain = new Drivetrain();
  private final Roller roller = new Roller();
  private final Arm arm = new Arm();
  
  public RobotContainer() {
    autoChooser.addDefaultOption("Do Nothing", new WaitCommand(1));
    configureBindings();
  }
  private void configureBindings() {
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> -controller.getLeftY(),
      () -> -controller.getRightX()
    ));

    controller.leftBumper().whileTrue(new Spit(roller));
    controller.leftTrigger().onTrue(new InstantCommand(() -> arm.setPivotDutyCycle(0.1)));
    controller.leftTrigger().onFalse(new InstantCommand(() -> arm.setPivotDutyCycle(-0.15)));
    controller.rightBumper().onTrue(new InstantCommand(() -> arm.setDutyCycle(-0.3)));
    controller.rightTrigger().onTrue(new InstantCommand(() -> arm.setDutyCycle(0.3)));
    controller.rightBumper().onFalse(new InstantCommand(() -> arm.setDutyCycle(0)));
    controller.rightTrigger().onFalse(new InstantCommand(() -> arm.setDutyCycle(0)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
