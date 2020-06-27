/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Button;
import frc.robot.commands.Aim;
import frc.robot.commands.FastShoot;
import frc.robot.commands.Intakecom;
import frc.robot.commands.LongShoot;
import frc.robot.commands.Auto.Easyauto;
import frc.robot.commands.Auto.Easyauto1;
import frc.robot.commands.Auto.Easyauto2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Powercell.Aimer;
import frc.robot.subsystems.Powercell.Arm;
import frc.robot.subsystems.Powercell.Intake;
import frc.robot.subsystems.Powercell.Shooter;
import frc.robot.subsystems.Powercell.Turret;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drivetrain drivetrain = new Drivetrain();
  private final Turret m_turret = new Turret();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Aimer m_aimer = new Aimer();
  private final Arm m_arm = new Arm();
  private final Vision m_vision = new Vision();
  private final Joystick joystick = new Joystick(0);
  private final Joystick drivestation = new Joystick(2);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Easyauto m_easyauto = new Easyauto(m_turret, drivetrain, m_vision, m_intake, m_shooter, m_aimer, m_arm);
  private final Easyauto1 m_easyauto1;
  private final Easyauto2 m_easyauto2 = new Easyauto2(m_shooter, drivetrain, m_vision);

  public RobotContainer() throws IOException {

    // Configure the button bindings
    configureButtonBindings();
    
    m_easyauto1 = new Easyauto1(drivetrain, m_vision);
    drivetrain.setDefaultCommand(new RunCommand(()->
    drivetrain.curvaturedrive(joystick.getY(), 0.3*joystick.getZ(),true),drivetrain));
    //m_intake.setDefaultCommand(new Intakecom(m_arm, m_intake,joystick.getRawAxis(3)));
    //m_shooter.setDefaultCommand(new RunCommand(()->m_shooter.stock(),m_shooter));
    m_chooser.addOption("0.2 go 1sec", m_easyauto);
    m_chooser.addOption("nothing haha", m_easyauto1);
    m_chooser.addOption("shoot & drive", m_easyauto2);
    //m_chooser.addOption("Simple AutoDrive", new InstantCommand(()-> m_drivetrain.drivedist(2),m_drivetrain));


    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //new JoystickButton(drivestation, Button.turrethoming).whenHeld(new InstantCommand(m_powercell::turrethoming,m_powercell));
    new JoystickButton(drivestation, Button.aim)                  .whenHeld(new Aim(m_turret, m_vision));
    new JoystickButton(drivestation, Button.shoot)                .whenHeld(new FastShoot(m_shooter));
    new JoystickButton(drivestation, 3)                           .whenHeld(new LongShoot(m_shooter));
    new JoystickButton(drivestation, 4)                           .whenHeld(new InstantCommand(()->m_aimer.getunit(), m_aimer).andThen(new RunCommand(()->m_aimer.aimunit()).withTimeout(1)));
    new JoystickButton(joystick,     Button.emergencyshooter)     .whenPressed(new InstantCommand(()->m_shooter.emergencyshot()).andThen(new WaitCommand(2).andThen(new InstantCommand(()->m_shooter.emergencyconyor())))).whenReleased(new InstantCommand(()->m_shooter.flywheelstop()).andThen(()->m_shooter.conveyorstop()));
    new JoystickButton(joystick,     Button.emergencyintake)      .whenPressed(new InstantCommand(m_intake::intake,m_intake)).whenReleased(new InstantCommand(m_intake::intakestop,m_intake));
    new JoystickButton(joystick,     Button.emergencyarmup)       .whenPressed(new InstantCommand(m_arm::armup,m_arm)).whenReleased(new InstantCommand(m_arm::armstop,m_arm));
    new JoystickButton(joystick,     Button.emergencyarmdown)     .whenPressed(new InstantCommand(m_arm::armdown,m_arm)).whenReleased(new InstantCommand(m_arm::armstop,m_arm));
    new JoystickButton(drivestation, Button.turretleft)           .whenPressed(new InstantCommand(m_turret::turretleft,m_turret)).whenReleased(new InstantCommand(m_turret::turretstop,m_turret));
    new JoystickButton(drivestation, Button.turretright)          .whenPressed(new InstantCommand(m_turret::turretright,m_turret)).whenReleased(new InstantCommand(m_turret::turretstop,m_turret));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous //new Easyauto2(m_shooter,m_drivetrain,m_vision);
    return m_chooser.getSelected();
  }
}
