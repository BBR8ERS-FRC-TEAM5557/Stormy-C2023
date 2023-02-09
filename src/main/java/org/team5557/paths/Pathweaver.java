package org.team5557.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.team5557.RobotContainer;
import org.team5557.paths.pathfind.Edge;
import org.team5557.paths.pathfind.Node;
import org.team5557.paths.pathfind.Obstacle;
import org.team5557.paths.pathfind.VisGraph;
import org.team5557.subsystems.swerve.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pathweaver {
    List<Obstacle> obstacles = new ArrayList<>();
    List<Obstacle> obstaclesWithOffsets = new ArrayList<>();
    VisGraph navMesh = new VisGraph();
    double obstacleOffsetDistance;

    public Pathweaver(double obstacleOffsetDistance) {
        this.obstacleOffsetDistance = obstacleOffsetDistance;
    }

    public Pathweaver(double obstacleOffsetDistance, List<Obstacle> obstacles) {
        this(obstacleOffsetDistance);
        this.obstacles = obstacles;
        obstaclesWithOffsets = obstacles.stream().map(o -> o.offset(obstacleOffsetDistance)).toList();
        obstaclesWithOffsets.forEach(this::addObstacleNodes);
    }

    public void addObstacle(Obstacle obstacle) {
        this.obstacles.add(obstacle);
        this.obstaclesWithOffsets.add(obstacle.offset(obstacleOffsetDistance));
        addObstacleNodes(obstacle);
    }

    public void addObstacleNodes(Obstacle obstacle) {
        obstacle.addNodes(navMesh);
    }

    /**
     * Finds a path between the start point and end point.
     * End point should have a node created and edges added for it earlier on, probably
     * at robot startup, since we'll know all the points we have presets for
     *
     * @param startPoint Current robot position
     * @param endPoint Target position to find a path to
     * @return List of nodes to create a trajectory through, or null if no path is found
     */
    public List<Node> findPath(Node startPoint, Node endPoint) {
        // Add edges from current position to all other positions
        //addNode(startPoint);

        //return navMesh.findPath(startPoint, endPoint);
        List<Node> fullPath = new ArrayList<Node>();

        navMesh.addNode(startPoint);
        if (navMesh.addEdge(new Edge(startPoint, endPoint), obstacles)) {
          fullPath.add(0, startPoint);
          fullPath.add(1, endPoint);
        } else {
          for (int i = 0; i < navMesh.getNodeSize(); i++) {
            Node endNode = navMesh.getNode(i);
            navMesh.addEdge(new Edge(startPoint, endNode), obstacles);
          }
          fullPath = navMesh.findPath(startPoint, endPoint);
        }

        return fullPath;
    }

    public void addNode(Node node) {
        navMesh.addNode(node);
        for(int i = 0; i < navMesh.getNodeSize(); i++) {
            Node endNode = navMesh.getNode(i);
            navMesh.addEdge(new Edge(node, endNode), obstacles);
        }
    }

    /**
     * Add edges between all nodes. Do this after all obstacles are added to the field
     */
    public void generateNodeEdges() {
        for(int i = 0; i < navMesh.getNodeSize(); i++) {
            Node startNode = navMesh.getNode(i);
            for(int j = i + 1; j < navMesh.getNodeSize(); j++) {
                Node endNode = navMesh.getNode(j);
                navMesh.addEdge(new Edge(startNode, endNode), obstacles);
            }
        }
    }

    public VisGraph getMeshInstance() {
        return navMesh;
    }

    ////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\
    ///////////////A* PATH GENERATOR\\\\\\\\\\\\\\\\\
    ////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\

    public PathPlannerTrajectory generatePath(Node finalPosition) {
        Swerve swerve = RobotContainer.swerve;
        PathPlannerTrajectory trajectory;
        Node startPoint = new Node(swerve.getPose());
        Node endPoint = finalPosition;

        List<Node> fullPath = findPath(startPoint, endPoint);

        if (fullPath.size() <= 0) {
            return null;
        }
        
        double startingSpeed = Math.hypot(swerve.getFilteredVelocity().vxMetersPerSecond, swerve.getFilteredVelocity().vyMetersPerSecond);
        Rotation2d heading = new Rotation2d(fullPath.get(1).getX() - startPoint.getX(), fullPath.get(1).getY() - startPoint.getY());
        if(startingSpeed > 0.05){
            heading = new Rotation2d(swerve.getFilteredVelocity().vxMetersPerSecond, swerve.getFilteredVelocity().vyMetersPerSecond);
        }
      
          // Depending on if internal points are present, make a new array of the other
          // points in the path.
          PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];
       
          for (int i = 0; i < fullPath.size(); i++) {
            if (i == 0) {
              fullPathPoints[i] = new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()), heading,
                  swerve.getPose().getRotation(), startingSpeed);
            } else if (i + 1 == fullPath.size()) {
              fullPathPoints[i] = new PathPoint(new Translation2d(endPoint.getX(), endPoint.getY()),
                  new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
                  endPoint.getHolRot());
            } else {
              fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
              new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(), fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
              endPoint.getHolRot());
            }
          }

          trajectory = PathPlanner.generatePath(new PathConstraints(2, 2), Arrays.asList(fullPathPoints));
          return trajectory;
    }
}