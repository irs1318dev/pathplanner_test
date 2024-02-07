package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TrajectoryGenerator
{

    public static void generateTrajectory()
    {
        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
        points.add(new PathPoint(new Translation2d(0.0, 0.0)));
        points.add(new PathPoint(new Translation2d(3.0, 0.0)));
        points.add(new PathPoint(new Translation2d(-3.0, 0.0)));

        generateTrajectory(points);
    }

    public static void generateTrajectory(ArrayList<PathPoint> points)
    {
        PathConstraints constraints = new PathConstraints(
            3.0,
            3.0,
            Math.PI / 2.0,
            Math.PI / 2.0);

        PathPlannerPath path = PathPlannerPath.fromPathPoints(
            points,
            constraints,
            new GoalEndState(0.0, Rotation2d.fromDegrees(0.0)));
        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(), new Rotation2d());

        double totalTimeSeconds = trajectory.getTotalTimeSeconds();
        if (Double.isInfinite(totalTimeSeconds) || Double.isNaN(totalTimeSeconds) || totalTimeSeconds < 0.0)
        {
            System.out.println("TotalTimeSeconds is not a valid number: " + totalTimeSeconds);
        }
        else
        {
            System.out.println("x,y,theta,dx,dy,omega");
            for (double time = 0.0; time <= totalTimeSeconds; time += 0.02)
            {
                State state = (State)trajectory.sample(time);
                double xPosition = state.positionMeters.getX();
                double yPosition = state.positionMeters.getY();
                double orientation = state.targetHolonomicRotation.getDegrees();
                double xVelocity = state.heading.getCos() * state.velocityMps;
                double yVelocity = state.heading.getSin() * state.velocityMps;
                double turnVelocity = state.holonomicAngularVelocityRps.isPresent() ? state.holonomicAngularVelocityRps.get() : 0.0;
                System.out.println("" + xPosition + "," + yPosition + "," + orientation + "," + xVelocity + "," + yVelocity + "," + turnVelocity);
            }
        }
    }
}
