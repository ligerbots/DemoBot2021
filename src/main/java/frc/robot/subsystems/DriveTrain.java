/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    /*
        A Talon SRX is the motor controller we use on the WCD, and the WPI_TalonSRX class allows us
        to comminicate with it. The CAN ID is used to differentiate between the ones we have on the robot.
        (Each one has an individual id assigned to it, see Constants.java)
    */
    private WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(Constants.LEADER_LEFT_CAN_ID);
    private WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(Constants.FOLLOWER_LEFT_CAN_ID);
    private WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(Constants.LEADER_RIGHT_CAN_ID);
    private WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(Constants.FOLLOWER_RIGHT_CAN_ID);

    /*
        The purpose of the SpeedControllerGroup class is to group multiple SpeedControllers together.
        This allows us to set the speed of multiple motors with one function call.
        A WPI_TalonSRX is actually a SpeedController (since it implements the interface).
    */
    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftLeader, m_leftFollower);
    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightLeader, m_rightFollower);

    /*
        The DifferentialDrive class helps us do the math to calculate the motor speeds from how we
        want it to go (throttle & turn)
    */
    private DifferentialDrive m_differentialDrive;

    /*
        Encoders measure the speed a motor is going. Since we're not doing odometry (yet), they are not used.
    */
    private Encoder m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
    private Encoder m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

    /*
        The NAVX is a gyro, and an accelerometer combined together. It helps us figure out what
        direction the robot is pointing. Since we're not doing odometry, it's not used.
    */ 
    private AHRS m_navX;

    public DriveTrain() {

        // Note: since we are using the DifferentialDrive class, we don't need to invert any motors.
        
        m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors); 
        m_differentialDrive.setSafetyEnabled(false);

        m_navX = new AHRS(Port.kMXP, (byte) 200);

        
        m_leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
        m_rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
    }
 
    /*
        This method sets the voltages of the motors.
    */
    public void tankDriveVolts (double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(-leftVolts);
        m_rightMotors.setVoltage(rightVolts); // make sure right is negative becuase sides are opposite
        m_differentialDrive.feed();
    }
    

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}

    public void arcadeDrive(double throttle, double rotate, boolean squaredInputs) {
        m_differentialDrive.arcadeDrive(throttle, -rotate, squaredInputs);
    }
}
