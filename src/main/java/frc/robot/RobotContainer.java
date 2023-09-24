// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Controller libraries
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

//Button libraries
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

//Shuffleboard libraries
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Camera library
import edu.wpi.first.cameraserver.CameraServer;

//NAVX libraries
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

//Constants
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ArmConstants;

//Commands
import frc.robot.commands.chassis.DefaultDrive;
import frc.robot.commands.chassis.AutoBalance;
import frc.robot.commands.chassis.AutoBalanceSmooth;
import frc.robot.commands.chassis.BrakeDrive;
import frc.robot.commands.auton.AutonTest;
import frc.robot.commands.auton.AutonSide;
import frc.robot.commands.auton.AutonStartup;
import frc.robot.commands.auton.AutonDoNothing;
import frc.robot.commands.auton.AutonMiddle;
import frc.robot.commands.auton.AutonOnePieceSide;
import frc.robot.commands.auton.AutonOnePieceMiddle;
import frc.robot.commands.auton.AutonOnePieceMiddle180;
import frc.robot.commands.auton.AutonOnePieceMiddleNoCommunity;
import frc.robot.commands.arm.ArmGotoAngle;
import frc.robot.commands.arm.DefaultArmState;

import frc.robot.commands.vision.AutoAlignBottom;
import frc.robot.commands.vision.AutoAlignTop;
import frc.robot.commands.vision.DefaultLimelightPipeline;

//Subsystems
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.NavSubsystem;

//Command libraries
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.concurrent.TimeUnit;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //Subsystems:
  private final ChassisSubsystem m_chassisSubsystem = new ChassisSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  private final NavSubsystem m_navSubsystem;
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final LEDSubsystem m_LEDSubsystem;
  
  
  //Controllers:
  XboxController m_driverController = new XboxController(IOConstants.kDriverPort);
  XboxController m_coDriverController = new XboxController(IOConstants.kCoDriverPort);
  
  //NavX:
  public AHRS ahrs;
  public double kInitialPitchOffset = 0;

  SendableChooser<Command> m_autonChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    //Start-up of USB cameras for drivers
    CameraServer.startAutomaticCapture();

    //Configure the button bindings
    configureBindings();

    m_chassisSubsystem.setDefaultCommand(
      new DefaultDrive(m_chassisSubsystem,
      () -> -m_driverController.getLeftY(),
      () -> m_driverController.getRightX(),
      () -> m_driverController.getRightTriggerAxis() > IOConstants.kTriggerThreshold)
    );

    m_visionSubsystem.setDefaultCommand(new DefaultLimelightPipeline(m_visionSubsystem));

    /*m_armSubsystem.setDefaultCommand(new DefaultArmState(m_armSubsystem, 
    () -> -m_driverController.getLeftY(),
    () -> m_driverController.getRightX()));*/
    
    //Make it so we can select the auton mode from shuffleboard
    m_autonChooser.setDefaultOption("AutonDoNothing",new AutonDoNothing());
    m_autonChooser.addOption("AutonMiddle",new AutonOnePieceMiddleNoCommunity(m_chassisSubsystem, m_clawSubsystem, m_armSubsystem, ahrs, kInitialPitchOffset));
    m_autonChooser.addOption("AutonSide",new AutonOnePieceSide(m_chassisSubsystem, m_clawSubsystem, m_armSubsystem));
    m_autonChooser.addOption("AutonTest",new AutonTest(m_chassisSubsystem, m_clawSubsystem, m_armSubsystem));
    m_autonChooser.addOption("AutonMiddleLeaveCommunity",new AutonOnePieceMiddle(m_chassisSubsystem, m_clawSubsystem, m_armSubsystem, ahrs, kInitialPitchOffset));

    Shuffleboard.getTab("Autonomous").add(m_autonChooser).withSize(2,1);
  }

  /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
  private void configureBindings() {


    //Use this for when we get LED code working:
    //new InstantCommand(() -> m_LEDSubsystem.changeLEDState(LEDState.BLINK)) 


    /** 
    * MAIN DRIVER BUTTONS:
    */


    //left triggejir: brake mode
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > IOConstants.kTriggerThreshold)
      .whileTrue(
        new BrakeDrive(m_chassisSubsystem,
        () -> -m_driverController.getLeftY(),
        () -> m_driverController.getRightX())
      );

    //Y button: auto aim (high pole) (i set it to be on a button press, not held)
    new JoystickButton(m_coDriverController, Button.kB.value)
    .toggleOnTrue(
      new AutoAlignTop(m_visionSubsystem, m_chassisSubsystem, m_LEDSubsystem)
    );

    //A button: auto aim (mide pole)
    new JoystickButton(m_driverController, Button.kA.value)
    .whileTrue(
      new AutoAlignBottom(m_visionSubsystem, m_chassisSubsystem, m_LEDSubsystem)
    );

    //Dpad up: reset encoders (for testing purposes)
    new POVButton(m_driverController, 0)
        .onTrue(
          //not setting requirements should prevent "Differential drive not updated enough"
          new InstantCommand(m_chassisSubsystem::resetEncoders)
        );
    




    /**
     * CO-DRIVER BUTTONS 
     */


    //Dpad left: Set arm to bottom
    new POVButton(m_coDriverController, 270)
      .onTrue(
        new ArmGotoAngle(ArmConstants.kBottomPosition, ArmConstants.kBottomSpeed, m_armSubsystem)
      );
    
    
    //Dpad up: Set arm to middle goal
    new POVButton(m_coDriverController, 0)
      .onTrue(
        new ArmGotoAngle(ArmConstants.kMiddlePosition, ArmConstants.kMiddleSpeed, m_armSubsystem)
      );

    
    //Dpad right: Set arm to top goal
    new POVButton(m_coDriverController, 90)
      .onTrue(
        new ArmGotoAngle(ArmConstants.kTopPositionCone, ArmConstants.kTopSpeed, m_armSubsystem)
      );

      //Right bumper: Raise the arm up manually
      // new JoystickButton(m_coDriverController, Button.kRightBumper.value)
      //   .onTrue(
      //   new InstantCommand(m_armSubsystem::up, m_armSubsystem)
      //   )
      //   .onFalse(
      //   new InstantCommand(m_armSubsystem::stop, m_armSubsystem)
      //   );

  


  /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
  public Command getAutonomousCommand() {
    //TODO: add a shuffleboard selector that doesn't break the robot
    return m_autonChooser.getSelected();
    //return new AutonOnePieceMiddleNoCommunity(m_chassisSubsystem, m_clawSubsystem, m_armSubsystem, ahrs, kInitialPitchOffset);
    //return new AutonOnePieceSide(m_chassisSubsystem, m_clawSubsystem, m_armSubsystem);
  }
}
