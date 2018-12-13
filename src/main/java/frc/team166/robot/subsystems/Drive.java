package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.RobotMap.PreferenceStrings;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.sensors.Lidar;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {

    WPI_VictorSPX m_left = new WPI_VictorSPX(RobotMap.CAN.LEFT);
    WPI_VictorSPX m_right = new WPI_VictorSPX(RobotMap.CAN.RIGHT);

    /**
     * defines the left and right motors defined above into a differential drive
     * that can be used for arcade and tank drive, amung other things
     */
    DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

    public Drive() {

        // SmartDashboard.putData("XBox", XboxArcade());
        // SmartDashboard.putData("Turn -45", TurnByDegrees(-45));
        // SmartDashboard.putData("Turn 45", TurnByDegrees(45));
        // SmartDashboard.putData("Drive 2s", DriveTime(2, .6));
        // SmartDashboard.putData("Drive Box", DriveBox());

        addChild(m_drive);

        // Encoder Position is the current, cumulative position of your encoder. If
        // you're using an SRX, this will be the
        // 'getEncPosition' function.

    }

    // the default command for this code is supposed to rotate the robot so that
    // it's gyro value is 0
    public void initDefaultCommand() {
        setDefaultCommand(XboxArcade());

    }

    public void reset() {
        m_drive.stopMotor();
    }

    public Command XboxArcade() {
        return new SubsystemCommand("XBoxArcade", this) {
            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(
                        (Robot.m_oi.xBoxDrive.getTriggerAxis(Hand.kRight)
                                - Robot.m_oi.xBoxDrive.getTriggerAxis(Hand.kLeft)),
                        Robot.m_oi.xBoxDrive.getX(Hand.kLeft));

            }

        };
    }

};
