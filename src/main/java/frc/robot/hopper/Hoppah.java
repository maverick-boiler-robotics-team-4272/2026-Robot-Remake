package frc.robot.hopper;


import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SubsystemConstants;
import frc.robot.drive.SwerveIO;
import frc.robot.drive.VisionIO;
import frc.robot.util.navgrid.Navgrid;

import static frc.robot.constants.SubsystemConstants.*;
import static frc.robot.constants.SubsystemConstants.HoppahConstants.HOPPAH_VOLTAGE;

public class Hoppah extends SubsystemBase {
    HoppahIO io;
    HoppahIOInputsAutoLogged hoppahInputs = new HoppahIOInputsAutoLogged(); 
    Pose2d[] nodes;

    public Hoppah(HoppahIO io) {
            this.io = io;
            nodes = Navgrid.getInstance().nodes;
        }
    
    @Override
    public void periodic() {
        io.updateInputs(hoppahInputs);
    }
    public Command hopRun() {
        return runEnd(
            () -> {
                io.setHoppahState(HOPPAH_VOLTAGE, HOPPAH_VOLTAGE);
            }, 
            () -> {
                io.setHoppahState(0, 0);
            }
        );
    }
}