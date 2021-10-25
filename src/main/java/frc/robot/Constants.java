// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drive Train SPARK MAX CAN ids
    public static final int LEADER_LEFT_CAN_ID = 1; 
    public static final int LEADER_RIGHT_CAN_ID = 4;
    public static final int FOLLOWER_LEFT_CAN_ID = 2;
    public static final int FOLLOWER_RIGHT_CAN_ID = 3;
    
    // Drive Train Encoder CAN ids
    public static final int[] LEFT_ENCODER_PORTS = new int[]{0, 1};//DriveTrain Encoder
    public static final int[] RIGHT_ENCODER_PORTS = new int[]{2, 3};//DriveTrain Encoder

    // The distance the robot has moved after one pulse of the encoder
    public static final double DISTANCE_PER_PULSE = Units.inchesToMeters(Math.PI * 6.0 / 1024);


    public static final double kvVoltSecondsPerMeter = 2.64; 
    public static final double kaVoltSecondsSquaredPerMeter = 0.324; 
    public static final double kvVoltSecondsPerRadian = 3.0;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;


    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
    LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

    public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);
    public static final double kDriveGearing = 8;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kTrackwidth = 0.6604;

}
