package frc.robot.subsystems;

//Motor libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

//Constants
import frc.robot.Constants.ChassisConstants;
import frc.robot.commands.chassis.DefaultDrive;
//Drive train object
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
//NAVX libraries
import com.kauailabs.navx.frc.AHRS;

public class ChassisSubsystem extends SubsystemBase {
    
    private final TalonFX test = new TalonFX(0)
    TalonFX grou
    private final TalonFX m_leftFront1 = new TalonFX(ChassisConstants.kLeftFrontPort);
    private final TalonFX m_leftFront2 = new TalonFX(ChassisConstants.kLeftFrontPort1);
    private final TalonFX m_leftRear1 = new TalonFX(ChassisConstants.kLeftRearPort);
    private final TalonFX m_leftRear2 = new TalonFX(ChassisConstants.kLeftRearPort1);
    MotorControllerGroup m_leftTop = new MotorControllerGroup(m_leftFront1, m_leftFront2);
    MotorControllerGroup m_leftBottom = new MotorControllerGroup(m_leftRear1, m_leftRear2);

    private final TalonFX m_rightFront1 = new TalonFX(ChassisConstants.kRightFrontPort);
    private final TalonFX m_rightFront2 = new TalonFX(ChassisConstants.kRightFrontPort1);
    private final TalonFX m_rightRear1 = new TalonFX(ChassisConstants.kRightRearPort);
    private final TalonFX m_rightRear2 = new TalonFX(ChassisConstants.kRightRearPort1);
    MotorControllerGroup m_rightTop = new MotorControllerGroup(m_rightFront1, m_rightFront2);
    MotorControllerGroup m_rightBottom = new MotorControllerGroup(m_rightRear1, m_rightRear2);

    //Get Encoders
    private RelativeEncoder m_leftFrontEncoder = m_leftFront1.getEncoder();
    private RelativeEncoder m_rightFrontEncoder = m_rightFront2.getEncoder();
    private RelativeEncoder m_rightRearEncoder1 = m_rightRear1.getEncoder();
    private RelativeEncoder m_rightRearEncoder2 = m_rightRear2.getEncoder();
    private RelativeEncoder m_leftRearEncoder1 = m_leftRear1.getEncoder();
    private RelativeEncoder m_leftRearEncoder2 = m_leftRear2.getEncoder();


    
    private final MecanumDrive m_drive = new MecanumDrive(m_leftFront1, m_leftRear1, m_rightFront1, m_rightRear1);
    private final MecanumDrive m_drive1 = new MecanumDrive(m_leftFront2, m_leftRear2, m_rightFront2, m_rightRear2);
    private AHRS ahrs; 


    public ChassisSubsystem(AHRS navx2){
        ahrs = navx2;
        m_leftFront1.restoreFactoryDefaults();
        m_rightFront2.restoreFactoryDefaults();
        m_rightRear1.restoreFactoryDefaults();
        m_rightRear2.restoreFactoryDefaults();
        m_leftRear1.restoreFactoryDefaults();
        m_leftRear2.restoreFactoryDefaults();

        m_leftFront1.setOpenLoopRampRate(0.25);
        m_rightFront2.setOpenLoopRampRate(0.25);
        m_rightRear1.setOpenLoopRampRate(0.25);
        m_rightRear2.setOpenLoopRampRate(0.25);
        m_leftRear1.setOpenLoopRampRate(0.25);
        m_leftRear2.setOpenLoopRampRate(0.25);

        m_leftFront1.setSmartCurrentLimit(ChassisConstants.kCurrentLimit);
        m_rightFront2.setSmartCurrentLimit(ChassisConstants.kCurrentLimit);
        m_rightRear1.setSmartCurrentLimit(ChassisConstants.kCurrentLimit);
        m_rightRear2.setSmartCurrentLimit(ChassisConstants.kCurrentLimit);
        m_leftRear1.setSmartCurrentLimit(ChassisConstants.kCurrentLimit);
        m_leftRear2.setSmartCurrentLimit(ChassisConstants.kCurrentLimit);
        
        m_leftFront1.setIdleMode(IdleMode.kCoast);
        m_rightFront2.setIdleMode(IdleMode.kCoast);
        m_rightRear1.setIdleMode(IdleMode.kCoast);
        m_rightRear2.setIdleMode(IdleMode.kCoast);
        m_leftRear1.setIdleMode(IdleMode.kCoast);
        m_leftRear2.setIdleMode(IdleMode.kCoast);
        
        m_leftFront1.setInverted(false);
        m_rightFront2.setInverted(true);
        m_leftRear1.setInverted(false);
        m_leftRear2.setInverted(false);
        m_rightRear1.setInverted(true);
        m_rightRear2.setInverted(true);

        m_leftFront1.burnFlash();
        m_rightFront2.burnFlash();
        m_leftRear1.burnFlash();
        m_leftRear2.burnFlash();
        m_rightRear1.burnFlash();
        m_rightRear2.burnFlash();

        resetEncoders();
    }

    public void setBrakeMode(){
        m_leftFront1.setIdleMode(IdleMode.kBrake);
        m_rightFront2.setIdleMode(IdleMode.kBrake);
        m_rightRear1.setIdleMode(IdleMode.kBrake);
        m_rightRear2.setIdleMode(IdleMode.kBrake);
        m_leftRear1.setIdleMode(IdleMode.kBrake);
        m_leftRear2.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode(){
        m_leftFront1.setIdleMode(IdleMode.kCoast);
        m_rightFront2.setIdleMode(IdleMode.kCoast);
        m_rightRear1.setIdleMode(IdleMode.kCoast);
        m_rightRear2.setIdleMode(IdleMode.kCoast);
        m_leftRear1.setIdleMode(IdleMode.kCoast);
        m_leftRear2.setIdleMode(IdleMode.kCoast);
    }

    //Called by autonomous commands
   // public void drive(double xSpeed, double zRotation, boolean b, double d){
        //Safety mode
        /*if(xSpeed>0.5){
            xSpeed = 0.5;
        }
        if(xSpeed<-0.5){
            xSpeed = -0.5;
        }
        if(zRotation>0.5){
            zRotation = 0.5;
        }
        if(zRotation<-0.5){
            zRotation = -0.5;
        }*/
        
        //Drift offset code
        // if(xSpeed>0){
        //     zRotation += SmartDashboard.getNumber("driftOffset", 0);
        // }
        // else if (xSpeed<0){
        //     zRotation -= SmartDashboard.getNumber("driftOffset", 0);
        // }


      //  m_drive.driveCartesian(xSpeed, xSpeed, zRotation);
   //}

    //Called by Default drive
    public void drive(double xSpeed, double ySpeed, boolean turbo, double twist){
        if(turbo){
            m_drive.driveCartesian(xSpeed, ySpeed, twist, ahrs.getRotation2d());
        }
        else{
            m_drive.driveCartesian(xSpeed*.8, ySpeed*.8, twist*.8, ahrs.getRotation2d());
        }
    }

    //Returns the average encoder rotation of the 6 encoders
    public double getAverageEncoderPosition(){
        return (m_leftFrontEncoder.getPosition() + m_leftRearEncoder1.getPosition() + m_leftRearEncoder2.getPosition() + 
        m_rightFrontEncoder.getPosition() + m_rightRearEncoder1.getPosition() + m_rightRearEncoder2.getPosition()) / 6;
    }

    //Returns the average encoder rotation of the 6 encoders converted to inches
    public double getAverageEncoderDistanceInches(){
        return getAverageEncoderPosition() / ChassisConstants.kInchesToRotationsConversionFactor;
    }

    //Returns the average rotation of the robot by subtracting the average of the right side from the average of the left side
    public double getAverageEncoderRotation(){
        return (
            ((m_leftFrontEncoder.getPosition() + m_leftRearEncoder1.getPosition() + m_leftRearEncoder2.getPosition())/3) - 
            ((m_rightFrontEncoder.getPosition() + m_rightRearEncoder1.getPosition() + m_rightRearEncoder2.getPosition())/3)
            );
    }

    //Might cause DifferentialDrive errors if called
    public void resetEncoders(){
        m_leftFrontEncoder.setPosition(0.0);
        m_rightFrontEncoder.setPosition(0.0);
        m_rightRearEncoder1.setPosition(0.0);
        m_leftRearEncoder1.setPosition(0.0);
        m_rightRearEncoder2.setPosition(0.0);
        m_leftRearEncoder2.setPosition(0.0);
    }

    //This is called every 20ms
    @Override
    public void periodic(){
        SmartDashboard.putNumber("LF_Enc",m_leftFrontEncoder.getPosition());
        SmartDashboard.putNumber("RF_Enc",m_rightFrontEncoder.getPosition());
        SmartDashboard.putNumber("RR1_Enc",m_rightRearEncoder1.getPosition());
        SmartDashboard.putNumber("RR2_Enc",m_rightRearEncoder2.getPosition());
        SmartDashboard.putNumber("LR1_Enc",m_leftRearEncoder1.getPosition());
        SmartDashboard.putNumber("LR2_Enc",m_leftRearEncoder2.getPosition());
        

        SmartDashboard.putNumber("LF_Speed",m_leftFront1.get());
        SmartDashboard.putNumber("RF_Speed",m_rightFront2.get());
        SmartDashboard.putNumber("RR1_Speed",m_rightRear1.get());
        SmartDashboard.putNumber("RR2_Speed",m_rightRear2.get());
        SmartDashboard.putNumber("LR1_Speed",m_leftRear1.get());
        SmartDashboard.putNumber("LR2_Speed",m_leftRear2.get());

        SmartDashboard.putNumber("AveragePosition",this.getAverageEncoderPosition());
        SmartDashboard.putNumber("AverageRotation",this.getAverageEncoderRotation());
        SmartDashboard.putNumber("AveragePosition(inch)",this.getAverageEncoderDistanceInches());
    }


}
