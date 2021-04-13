/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {

    private WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(Constants.LEADER_LEFT_CAN_ID);
    private WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(Constants.FOLLOWER_LEFT_CAN_ID);
    private WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(Constants.LEADER_RIGHT_CAN_ID);
    private WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(Constants.FOLLOWER_RIGHT_CAN_ID);

    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftLeader, m_leftFollower);

    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightLeader, m_rightFollower);

    private DifferentialDrive m_differentialDrive;
    private DifferentialDriveOdometry m_odometry;

    private Encoder m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
    private Encoder m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

    private AHRS m_navX;

    // Simulation classes
    // private DifferentialDrivetrainSim drivetrainSimulator;
    // private EncoderSim leftEncoderSim;
    // private EncoderSim rightEncoderSim;
    // The Field2d class simulates the field in the sim GUI. Note that we can have only one instance!
    // private Field2d fieldSim;
    // private SimDouble gyroAngleSim;

    public DriveTrain() {
        // Note: since we are using the DifferentialDrive class, we don't need to invert any motors.

        m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_differentialDrive.setSafetyEnabled(false);

        m_navX = new AHRS(Port.kMXP, (byte) 200);

        // Set current limiting on drive train to prevent brown outs
        // Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
        //     .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // // Set motors to brake when idle. We don't want the drive train to coast.
        // Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
        //      .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        ////////////////////////////ODOMETRY SET UP//////////////////////////////////

        m_leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
        m_rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
        
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

        // if (RobotBase.isSimulation()) {
        //     // If our robot is simulated
        //     // This class simulates our drivetrain's motion around the field.
        //     drivetrainSimulator = new DifferentialDrivetrainSim(
        //           Constants.kDrivetrainPlant,
        //           Constants.kDriveGearbox,
        //           Constants.kDriveGearing,
        //           Constants.kTrackwidth,
        //           Constants.kWheelDiameterMeters / 2.0,
        //           null);
      
        //     // The encoder and gyro angle sims let us set simulated sensor readings
        //     leftEncoderSim = new EncoderSim(leftEncoder);
        //     rightEncoderSim = new EncoderSim(rightEncoder);
            
        //     // get the angle simulation variable
        //     // SimDevice is found by name and index, like "name[index]"
        //     gyroAngleSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");
      
        //     // the Field2d class lets us visualize our robot in the simulation GUI.
        //     fieldSim = new Field2d();
        //     SmartDashboard.putData("Field", fieldSim);
            
        //     SmartDashboard.putNumber("moveAroundField/startPos", prevStartLocation);
        //     SmartDashboard.putNumber("moveAroundField/ballPos", prevBallLocation);
        // }
    }

    // public Field2d getField2d() {
    //     return fieldSim;
    // }
    
    public Pose2d getPose () {
        return m_odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        // if (RobotBase.isSimulation()) {
        //     // This is a bit hokey, but if the Robot jumps on the field, we need
        //     //   to reset the internal state of the DriveTrainSimulator.
        //     //   No method to do it, but we can reset the state variables.
        //     //   NOTE: this assumes the robot is not moving, since we are not resetting
        //     //   the rate variables.
        //     drivetrainSimulator.setState(new Matrix<>(Nat.N7(), Nat.N1()));

        //     // reset the GyroSim to match the driveTrainSim
        //     // do it early so that "real" odometry matches this value
        //     gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
        //     fieldSim.setRobotPose(pose);
        // }

        // The left and right encoders MUST be reset when odometry is reset
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
    }
      
    public void tankDriveVolts (double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(-leftVolts);
        m_rightMotors.setVoltage(rightVolts);// make sure right is negative becuase sides are opposite
        m_differentialDrive.feed();
    }

    public double getRightSpeed() {
        return -m_rightMotors.get();
    }

    public double getLeftSpeed() {
        return m_leftMotors.get();
    }
    
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    }
    
    public double getLeftEncoderDistance() {
        return m_leftEncoder.getDistance();
    }
    
    public double getRightEncoderDistance() {
        return m_rightEncoder.getDistance();
    }
    
    public double getHeading() {
        return m_odometry.getPoseMeters().getRotation().getDegrees();
      }
    
    private double getGyroAngle() {
        return Math.IEEEremainder(m_navX.getAngle(), 360) * -1; // -1 here for unknown reason look in documatation
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getGyroAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

        SmartDashboard.putNumber("driveTrain/heading", getHeading());
        SmartDashboard.putNumber("driveTrain/NavX gyro", getGyroAngle());
        SmartDashboard.putNumber("driveTrain/x position", getPose().getX());
        SmartDashboard.putNumber("driveTrain/y position", getPose().getY());

        SmartDashboard.putNumber("driveTrain/left encoder", getLeftEncoderTicks());
        SmartDashboard.putNumber("driveTrain/right encoder", getRightEncoderTicks());
    }

    //@Override
    // public void simulationPeriodic() {
    //   // To update our simulation, we set motor voltage inputs, update the simulation,
    //   // and write the simulated positions and velocities to our simulated encoder and gyro.
    //   // We negate the right side so that positive voltages make the right side
    //   // move forward.
    //   drivetrainSimulator.setInputs(-m_leftMotors.get() * RobotController.getBatteryVoltage(),
    //     m_rightMotors.get() * RobotController.getBatteryVoltage());
    //   drivetrainSimulator.update(0.020);

    //   leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
    //   leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
  
    //   rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
    //   rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());
  
    //   gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
  
    //   fieldSim.setRobotPose(getPose());
    // }

    // public void allDrive(double throttle, double rotate, boolean squaredInputs, boolean driveSwitch) {
    //     if (squaredInputs) {
    //         if (Math.abs(throttle) < 0.05) {
    //             throttle = 0;
    //         } 
    //         else {
    //             throttle = Math.signum(throttle) * throttle * throttle;
    //         }
    //         if (Math.abs(rotate) < 0.05) {
    //             rotate = 0;
    //         } else {
    //             rotate = Math.signum(rotate) * rotate * rotate;
    //         }
    //     }
    //     // SmartDashboard.putBoolean("DriveSwitch", driveSwitch);
    //     // if (driveSwitch) {  
    //     //     differentialDrive.arcadeDrive(throttle, -rotate, squaredInputs);
    //     // } else {
    //     m_differentialDrive.curvatureDrive(throttle, -rotate, driveSwitch);
    // }

    // Raw access to arcade drive (use only for auto routines)
    public void arcadeDrive(double throttle, double rotate, boolean squaredInputs) {
        m_differentialDrive.arcadeDrive(throttle, -rotate, squaredInputs);
    }

    public int getLeftEncoderTicks () {
        return m_leftEncoder.get();
    }

    public int getRightEncoderTicks () {
        return m_rightEncoder.get();
    }

    public double getPitch() {
        return m_navX.getPitch();
    }

    public void setIdleMode(NeutralMode neutralMode) {
        if (Robot.isReal()) {
            Arrays.asList(m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower)
            .forEach((WPI_TalonSRX talon) -> talon.setNeutralMode(neutralMode));
        }
    }

    public void setRobotFromFieldPose() {
        // only applies for simulation
        // if (RobotBase.isSimulation())
        //     setPose(fieldSim.getRobotPose());
    }
}
