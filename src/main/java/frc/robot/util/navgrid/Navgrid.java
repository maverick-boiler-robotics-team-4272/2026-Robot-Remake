package frc.robot.util.navgrid;

import static frc.robot.constants.FieldConstants.*;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;

public class Navgrid {
    private static Navgrid navgrid = new Navgrid();

    private static final double kMaxRotationalDifferentce = 60.0; // degrees

    private final double originalVectorMagnitude = 2; // mps

    private boolean[][] grid;
    public Pose2d[] nodes;
    public double nodeSize;

    private Navgrid() {
        try {
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json"));

            grid = mapper.convertValue(root.get("grid"), boolean[][].class);
            nodeSize = mapper.convertValue(root.get("nodeSizeMeters"), Double.class);
            nodes = getNodePoseList();
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to load navgrid", e);
        }
    }

    public static Navgrid getInstance() {
        return navgrid;
    }

    private Pose2d[] getNodePoseList() {
        int markedNum = 0;
        int unMarkedNum = 0;
        for(boolean[] row : grid) {
            for(boolean node : row) {
                if(node) {
                    markedNum++;
                    continue;
                }
                unMarkedNum++;
            }
        }

        Translation2d[] nodes = new Translation2d[markedNum];
        Translation2d[] empty = new Translation2d[unMarkedNum];
        int nodeNum = 0;
        int emptyNum = 0;
        double x = 0;
        double y = 0.15;
        for(boolean[] row : grid) {
            for(int i = 0; i < grid[0].length; i++) {
                if(row[i]) {
                    nodes[nodeNum] = new Translation2d(x, y);
                    nodeNum++;
                } else {
                    empty[emptyNum] = new Translation2d(x, y);
                    emptyNum++;
                }
                x += nodeSize;
            }
            y += nodeSize;
            x = 0;
        }

        Pose2d[] victors = getNodeRotationsList(nodes, empty); //like a vector, but without a magnitude

        return victors;
    }

    //stupid rotation logic is no fun
    private Pose2d[] getNodeRotationsList(Translation2d[] nodes, Translation2d[] empty) {
        final Rotation2d blueForward = Rotation2d.kZero;
        final Rotation2d blueLeft = Rotation2d.kCCW_90deg;
        final Rotation2d blueRight = Rotation2d.kCW_90deg;
        final Rotation2d blueBackwards = Rotation2d.k180deg;
        List<Pose2d> empties = new ArrayList<>();
        boolean[] resolved = new boolean[nodes.length];

        Pose2d[] victors = new Pose2d[nodes.length];

        for(Translation2d item : empty) {
            empties.add(new Pose2d(item, Rotation2d.kZero));
        }

        for(int i = 0; i < nodes.length; i++) {
            /*
             * You can aquire a Rotation2d object from given x and y components
             * if we add all of the static components defined as the opposite of each wall
             * we can get an average rotation (this is specifically helpful with corners)
             * We start with wall logic
             */
            double x = 0.0;
            double y = 0.0;
            boolean wallAdjacent = false;

            if(nodes[i].getX() - nodeSize < 0) { //next to blue alliance wall
                x += blueForward.getCos();
                y += blueForward.getSin();
                wallAdjacent = true;
            }

            if(nodes[i].getX() + nodeSize > FIELD_LENGTH_M) { //next to red Alliance Wall
                x += blueBackwards.getCos();
                y += blueBackwards.getSin();
                wallAdjacent = true;
            }

            if(nodes[i].getY() - nodeSize < 0) { //next to blue right wall
                x += blueLeft.getCos();
                y += blueLeft.getSin();
                wallAdjacent = true;    
            }

            if(nodes[i].getY() + nodeSize > FIELD_WIDTH_M) { //next to blue left wall
                x += blueRight.getCos();
                y += blueRight.getSin();
                wallAdjacent = true;
            }

            // get the rotation from the x and y component
            if(wallAdjacent) {
                // opposite walls (e.g. a corridor) can cancel out to a zero vector, which
                // Rotation2d(x, y) can't convert to an angle, so fall back to a default rotation
                Rotation2d rotation = (x == 0.0 && y == 0.0) ? blueForward : new Rotation2d(x, y);
                victors[i] = new Pose2d(nodes[i], rotation);
                resolved[i] = true;
                continue;
            }

            //default
            victors[i] = new Pose2d(nodes[i], blueForward);
        }

            /*
            * Naturally... hardest for last...
            * With all wall adjacents complete, I have thought up an algorithm to determine the rotations of the remaining nodes
            * If we make a list of all of the empty square adjacents, then do the same thing we did for the empty square adjacents except:
            * instead of checking for empty squares, we check to see if it is in the list of empty square adjacents. Then we can do the same thing we did for the empty square adjacents
            * every time we do this, we make a new list of 'adjacentToEmptySquares' and run the algorithm recursively until 'adjacentToEmptySquares' length is zero.
            */
            List<Pose2d> currentEmpty = new ArrayList<>(empties);
            while(true) {
                List<Pose2d> newEmpty = new ArrayList<>();
                boolean anyResolutions = false;
                for(int i = 0; i < nodes.length; i++) {
                    if(resolved[i]) continue;
                    /*
                    * All nodes that have at least one open square next to it can have rotations defined as such
                    * for each adjacent open square: add x and y components (cos and sin) of the defined directio (see below)
                    * 
                    * if there is an open square in y up/down and x up/down, add sin/cos to y/x respectively
                    * y += nodeSize : right (yUp)
                    * y -= nodeSize : left (yDown)
                    * x += nodeSize : backward (xUp)
                    * x -= nodeSize : forward (xDown)
                    * 
                    * if there is an open square in the diagonals, add both x and y components as defined directly above
                    * y += nodeSize && x += nodeSize
                    * ... etc.
                    */

                    double x = 0.0;
                    double y = 0.0;
                    boolean emptySquare = false;

                    Translation2d xUp = new Translation2d(nodes[i].getX() + nodeSize, nodes[i].getY()); // backward
                    Translation2d xDown = new Translation2d(nodes[i].getX() - nodeSize, nodes[i].getY()); // forward
                    Translation2d yUp = new Translation2d(nodes[i].getX(), nodes[i].getY() + nodeSize); // right
                    Translation2d yDown = new Translation2d(nodes[i].getX(), nodes[i].getY() - nodeSize); // left
                    // diagonals are combinations of xy as defined directly above
                    Translation2d xUpYUpDiag = new Translation2d(nodes[i].getX() + nodeSize, nodes[i].getY() + nodeSize);
                    Translation2d xUpYDownDiag = new Translation2d(nodes[i].getX() + nodeSize, nodes[i].getY() - nodeSize);
                    Translation2d xDownYUpDiag = new Translation2d(nodes[i].getX() - nodeSize, nodes[i].getY() + nodeSize);
                    Translation2d xDownYDownDiag = new Translation2d(nodes[i].getX() - nodeSize, nodes[i].getY() - nodeSize);

                    if(contains(xDown, currentEmpty)) {
                        x += blueBackwards.getCos();
                        y += blueBackwards.getSin();
                        emptySquare = true;
                    }

                    if(contains(xUp, currentEmpty)) {
                        x += blueForward.getCos();
                        y += blueForward.getSin();
                        emptySquare = true;
                    }

                    if(contains(yDown, currentEmpty)) {
                        x += blueRight.getCos();
                        y += blueRight.getSin();
                        emptySquare = true;
                    }

                    if(contains(yUp, currentEmpty)) {
                        x += blueLeft.getCos();
                        y += blueLeft.getSin();
                        emptySquare = true;
                    }

                    if(contains(xUpYUpDiag, currentEmpty)) {
                        x += blueForward.getCos() + blueLeft.getCos(); 
                        y += blueForward.getSin() + blueLeft.getSin(); 
                        emptySquare = true;
                    }

                    if(contains(xUpYDownDiag, currentEmpty)) {
                        x += blueForward.getCos() + blueRight.getCos();
                        y += blueForward.getSin() + blueRight.getSin();
                        emptySquare = true;
                    }

                    if(contains(xDownYUpDiag, currentEmpty)) {
                        x += blueBackwards.getCos() + blueLeft.getCos();
                        y += blueBackwards.getSin() + blueLeft.getSin();
                        emptySquare = true;
                    }

                    if(contains(xDownYDownDiag, currentEmpty)) {
                        x += blueBackwards.getCos() + blueRight.getCos();
                        y += blueBackwards.getSin() + blueRight.getSin();
                        emptySquare = true;
                    }

                    // get the rotation from the x and y component
                    if(emptySquare) {
                        // opposite empty squares (e.g. a corridor) can cancel out to a zero vector,
                        // which Rotation2d(x, y) can't convert to an angle, so fall back to a default
                        Rotation2d rotation = (x == 0.0 && y == 0.0) ? Rotation2d.kZero : new Rotation2d(x, y);
                        victors[i] = new Pose2d(nodes[i], rotation);
                        newEmpty.add(victors[i]);
                        resolved[i] = true;
                        anyResolutions = true;
                        continue;
                    }
                }
                if(!anyResolutions) break;
                currentEmpty = newEmpty;
            }

        return victors;
    }

    private boolean contains(Translation2d item, List<Pose2d> list) {
        for(Pose2d thing : list) {
            if(item.equals(thing.getTranslation())) {
                return true;
            }
        }
        return false;
    }

    /**
     * This calculates the magnitude of all vectors
     * at 100% stick forward, the magnitude is {@link #originalVectorMagnitude 0.5} 
     * @param x is the joystick x value
     * @param y is the joystick y value
     */
    public double getCurrentMagnitude(double x, double y) {
        final double scaling = 1.5;
        double desiredMagnitude = Math.sqrt(x*x + y*y) * scaling;
        return originalVectorMagnitude * desiredMagnitude;
    }

    /**
     * @param robotSpeeds is FIELD RELATIVE
     * @return a list of adjusted node rotations
     */
    public Pose2d[] getSpeedAdjustedNodes(ChassisSpeeds robotSpeeds) {
        Pose2d[] adjusted = new Pose2d[nodes.length]; // a list of all adjusted nodes

        // if no speeds, save the calculation power
        if (Math.abs(robotSpeeds.vxMetersPerSecond) < 0.1 && Math.abs(robotSpeeds.vyMetersPerSecond) < 0.1) {
            System.arraycopy(nodes, 0, adjusted, 0, nodes.length);
            return adjusted;
        }

        // the target angle defined by the moving robot
        Rotation2d target = new Rotation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

        for (int i = 0; i < nodes.length; i++) {
            Rotation2d original = nodes[i].getRotation(); // node original rotation
            Rotation2d diff = target.minus(original); // the difference of rotation from the target and the original
            double diffDeg = diff.getDegrees();
            double absDiff = Math.abs(diffDeg);

            if (absDiff <= kMaxRotationalDifferentce) {
                double clampedDiffDeg = MathUtil.clamp(diffDeg, -kMaxRotationalDifferentce, kMaxRotationalDifferentce);
                adjusted[i] = new Pose2d(nodes[i].getTranslation(), original.plus(Rotation2d.fromDegrees(clampedDiffDeg)));

            } else {
                Rotation2d flipped = original.plus(Rotation2d.k180deg);
                Rotation2d diffFlipped = target.minus(flipped);
                double clampedDiffDeg = MathUtil.clamp(diffFlipped.getDegrees(), -kMaxRotationalDifferentce, kMaxRotationalDifferentce);
                adjusted[i] = new Pose2d(nodes[i].getTranslation(), original.plus(Rotation2d.fromDegrees(-clampedDiffDeg)));

            }
        }

        return adjusted;
    }
    /**
     * 
     * @param robotPose current pose of robot
     * @param distanceToRelatvantNode how close the nodes should be to be considered
     * @param adjustedNodes the speed adjusted nodes should be applied here so we don't have to recalculate
     * @return a list of nodes relavant to the current robot's position;
     */
    public Pose2d[] getRelevantNodes(Pose2d robotPose, double distanceToRelatvantNode, Pose2d[] adjustedNodes) {
        List<Pose2d> relativeNodes = new ArrayList<>();
        for(Pose2d node : adjustedNodes) {
            if(robotPose.getTranslation().getDistance(node.getTranslation()) <= distanceToRelatvantNode) {
                relativeNodes.add(node);
            }
        }

        return relativeNodes.toArray(new Pose2d[relativeNodes.size()]);
    }

    /**
     * 
     * @param relevantNodes precalculated nodes that should have effect on the robot
     * @return the average cosine of all the rotations from the pose
     * 
     * You can use this method and {@link #getCurrentMagnitude(double x, double y)} to get the x component of the assisted adjustment
     */
    public double xOffsetFromRelevantNodes(Pose2d[] relevantNodes) {
        if (relevantNodes.length == 0)
            return 0.0; // protect against no nodes
        double xSum = 0.0;
        for(Pose2d node : relevantNodes) {
            xSum += node.getRotation().getCos();
        }

        return xSum / relevantNodes.length;
    }

    /**
     * 
     * @param relevantNodes precalculated nodes that should have effect on the robot
     * @return the average sine of all the rotations from the pose
     * 
     * You can use this method and {@link #getCurrentMagnitude(double x, double y)} to get the y component of the assisted adjustment
     */
    public double yOffsetFromRelevantNodes(Pose2d[] relevantNodes) {
        if (relevantNodes.length == 0)
            return 0.0; // protect against no nodes
        double ySum = 0.0;
        for(Pose2d node : relevantNodes) {
            ySum += node.getRotation().getSin();
        }
        return ySum / relevantNodes.length;
    }
}
