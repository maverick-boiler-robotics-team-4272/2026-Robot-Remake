package frc.robot.drive;

import static frc.robot.drive.VisionConstants.*;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.navgrid.Navgrid;

import static frc.robot.constants.SubsystemConstants.DriveConstants.*;

public class Drive extends SubsystemBase {
    SwerveIO io;
    SwerveIOInputsAutoLogged swerveInputs = new SwerveIOInputsAutoLogged();
    VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    ModuleIOInputsAutoLogged FLInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged FRInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged BLInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged BRInputs = new ModuleIOInputsAutoLogged();

    ModuleIOInputsAutoLogged[] moduleInputs = {FLInputs, FRInputs, BLInputs, BRInputs};
    VisionIO[] cameras;
    VisionIOInputsAutoLogged[] visionInputsList = {visionInputs};
    
    Pose2d[] nodes;

    public Drive(SwerveIO io, VisionIO... visionIOs) {
        this.io = io;
        this.cameras = visionIOs;
        nodes = Navgrid.getInstance().nodes;
    }

    @Override
    public void periodic() {
        io.updateSwerveInputs(swerveInputs);
        io.updateModuleInputs(moduleInputs);
        io.logModules(swerveInputs);

        for(int i = 0; i < cameras.length; ++i) {
            cameras[i].updateInputs(visionInputsList[i]);
        }

        Logger.processInputs("Drive/Swerve Inputs", swerveInputs);
        for(int i = 0; i < moduleInputs.length; i++) {
            Logger.processInputs("Drive/Module " + i + "/Inputs", moduleInputs[i]);
        }

        nodes = Navgrid.getInstance().getSpeedAdjustedNodes(ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, getState().Pose.getRotation()));

        Logger.recordOutput("Drive/Assist Nodes", nodes);

        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(visionInputsList[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), visionInputsList[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++) {
            // TODO: Update disconnected alert
            // disconnectedAlerts[cameraIndex].set(!visionInputsList[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : visionInputsList[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : visionInputsList[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                    observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                            && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ())
                            > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor =
                    Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;

                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                io.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp(),  VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

                // Log camera metadata
                Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[0]));
                Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[0]));
                Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[0]));
                Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[0]));
                allTagPoses.addAll(tagPoses);
                allRobotPoses.addAll(robotPoses);
                allRobotPosesAccepted.addAll(robotPosesAccepted);
                allRobotPosesRejected.addAll(robotPosesRejected);
            }

            // Log summary data
            Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
        }
    }

    public SwerveDriveState getState() {
        return swerveInputs;
    }

    public Command joystickDrive(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickThetaX) {
        FieldCentric request = new SwerveRequest.FieldCentric()
        .withDeadband(MAX_DRIVE_SPEED * 0.05)
        .withRotationalDeadband(MAX_ROTATIONAL_RATE * 0.05)
        .withDriveRequestType(DriveRequestType.Velocity);
        return run(() -> {
            io.setControl(
                request
                .withVelocityX(-joystickY.getAsDouble() * MAX_DRIVE_SPEED)
                    .withVelocityY(-joystickX.getAsDouble() * MAX_DRIVE_SPEED)
                    .withRotationalRate(-joystickThetaX.getAsDouble() * MAX_ROTATIONAL_RATE)
            );
        });
    }

    public Command joystickDriveWithAssist(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickTheta) {
        FieldCentric request = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_DRIVE_SPEED * 0.05)
            .withRotationalDeadband(MAX_ROTATIONAL_RATE * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity);
        return run(()-> {

            Pose2d[] relevent = Navgrid.getInstance().getRelevantNodes(getState().Pose, DRIVE_BASE_HYPOTONUSE * 1.5, nodes);
            double xOffset = Navgrid.getInstance().xOffsetFromRelevantNodes(relevent) * Navgrid.getInstance().getCurrentMagnitude(joystickX.getAsDouble(), joystickY.getAsDouble());
            double yOffset = Navgrid.getInstance().yOffsetFromRelevantNodes(relevent) * Navgrid.getInstance().getCurrentMagnitude(joystickX.getAsDouble(), joystickY.getAsDouble());

            Logger.recordOutput("XOFF", xOffset);
            Logger.recordOutput("yOFF", yOffset);

            io.setControl(
                request
                    .withVelocityX(-joystickY.getAsDouble() * MAX_DRIVE_SPEED + xOffset)
                    .withVelocityY(joystickX.getAsDouble() * MAX_DRIVE_SPEED + yOffset)
                    .withRotationalRate(-joystickTheta.getAsDouble() * MAX_ROTATIONAL_RATE));
        });
    }

    public Command rotateOnMove(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickTheta) {
        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MAX_DRIVE_SPEED * 0.05)
            .withRotationalDeadband(MAX_ROTATIONAL_RATE * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(1.5, 0, 0);
        return run(() -> {
            ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, getState().Pose.getRotation());
            io.setControl(
                request
                    .withVelocityX(-joystickY.getAsDouble() * MAX_DRIVE_SPEED)
                    .withVelocityY(-joystickX.getAsDouble() * MAX_DRIVE_SPEED)
                    .withTargetDirection(new Rotation2d(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond)));
        });
    }
    
}
