// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.A_Star;
import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.Arrays;
import java.util.Collections;

public class A_Star 
{
    private static List<Node> grid;
    private static final Double[] object_size = Constants.Auton.robot_size;
    private static final Double[] grid_size = Constants.Auton.field_size;

    private static Node findPointOnGrid(Node node) {
        for (Node g : grid) {
            if (Arrays.equals(g.getLocation(), node.getLocation())) {
                return g;
            }
        }
        return null;
    }

    private static double round(double x) {
        return Math.round(1000.0*x)/1000.0;
    }

    private static List<Node> getNeighbors(Node point) {
        List<Double[]> neighbors = new ArrayList<Double[]>();
        List<Node> final_neighbors = new ArrayList<Node>();
        Double[] location = point.getLocation();
        // System.out.printf("Computing Neighbors for (%s, %s)\n", location[0], location[1]);
        neighbors.add(new Double[] {round(location[0]+object_size[0]), round(location[1])});
        neighbors.add(new Double[] {round(location[0]-object_size[0]), round(location[1])});
        neighbors.add(new Double[] {round(location[0]), round(location[1]+object_size[1])});
        neighbors.add(new Double[] {round(location[0]), round(location[1]-object_size[1])});
        neighbors.add(new Double[] {round(location[0]-object_size[0]), round(location[1]-object_size[1])});
        neighbors.add(new Double[] {round(location[0]+object_size[0]), round(location[1]+object_size[1])});
        neighbors.add(new Double[] {round(location[0]+object_size[0]), round(location[1]-object_size[1])}); // EXPERIMENTAL
        neighbors.add(new Double[] {round(location[0]-object_size[0]), round(location[1]+object_size[1])}); // EXPERIMENTAL
        for (Double[] n : neighbors) {
            Node g = findPointOnGrid(new Node(n[0], n[1], false));
            if (g != null && g.obstacle == false) {
                // System.out.printf("Found neighbor (%s %s)\n", g.getLocation()[0], g.getLocation()[1]);
                final_neighbors.add(g);
            }
        }
        return final_neighbors;
    }

    private static double getDistance(Node pointA, Node pointB) {
        Double[] coordA = pointA.getLocation();
        Double[] coordB = pointB.getLocation();
        double xDist = Math.abs(coordA[0] - coordB[0]);
        double yDist = Math.abs(coordA[1] - coordB[1]);
        return 14 * Math.min(xDist, yDist) + 10 * Math.abs(xDist - yDist);
    }

    public static List<Node> compute(Node startNode, Node endNode) throws RuntimeException {
        System.out.println(startNode.getLocation()[0]);
        if (grid == null) {
            grid = new ArrayList<Node>();
            for (double i=object_size[0]; i<grid_size[0]; i+=object_size[0]) {
                for (double j=object_size[1]; j<grid_size[1]; j+=object_size[1]) {
                    // System.out.printf("%s, %s\n", Math.round(1000.0*i)/1000.0, Math.round(1000.0*j)/1000.0);
                    grid.add(new Node(round(i), round(j), false));
                }
            }
        }
        for (Node g : grid) {
            g.set_g_cost(0);
            g.set_h_cost(0);
            g.parent = null;
        }
        startNode = findPointOnGrid(startNode.toNearestGrid(object_size));
        endNode = findPointOnGrid(endNode.toNearestGrid(object_size));
        System.out.println(Arrays.deepToString(startNode.getLocation()));
        System.out.println(Arrays.deepToString(endNode.getLocation()));
        if (startNode == null || startNode.obstacle) {
            throw new RuntimeException("Invalid Start Node!!!");
        }
        if (endNode.obstacle || endNode == null) {
            throw new RuntimeException("Invalid End Node!!!");
        }
        startNode.set_g_cost(0.0);
        List<Node> open = new ArrayList<Node>();
        open.add(startNode);
        List<Node> closed = new ArrayList<Node>();

        while (open.size() > 0) {
            Node currentNode = open.get(0);
            for (int i=1; i<open.size(); i++) {
                if (open.get(i).get_f_cost() < currentNode.get_f_cost() || (open.get(i).get_f_cost() == currentNode.get_f_cost() && open.get(i).get_h_cost() < currentNode.get_h_cost())) {
                    currentNode = open.get(i);
                }
            }
            open.remove(currentNode);
            closed.add(currentNode);

            // System.out.println(Arrays.deepToString(currentNode.getLocation()));

            if (currentNode == endNode) {
                Node lastNode = endNode;
                List<Node> chain = new ArrayList<Node>();
                while (lastNode.parent != null) {
                    chain.add(lastNode);
                    // System.out.println(Arrays.deepToString(lastNode.getLocation()));
                    lastNode = lastNode.parent;
                }
                chain.add(startNode);
                Collections.reverse(chain);
                return chain;
            }

            for (Node n : getNeighbors(currentNode)) {
                if (closed.contains(n)) {
                    continue;
                }
                double newMoveToNeighborCost = currentNode.get_g_cost() + getDistance(currentNode, n);
                if (newMoveToNeighborCost < n.get_g_cost() || !open.contains(n)) {
                    n.set_g_cost(newMoveToNeighborCost);
                    n.set_h_cost(getDistance(n, endNode));
                    n.set_parent(currentNode);
                    if (!open.contains(n)) {
                        open.add(n);
                    }
                }
            }
        }
        // return new ArrayList<Node>();
        throw new RuntimeException("No path found");
    }

    public static Pose2d[] nodeListToPoses(List<Node> path, Rotation2d rotation) {
        Pose2d[] finalList = new Pose2d[path.size()];
        for (int i=0; i<path.size(); i++) {
            finalList[i] = new Pose2d(path.get(i).getLocation()[0], path.get(i).getLocation()[1], rotation);
        }
        return finalList;
    }

    public static Pose2d[] nodeListToPosesWPI(List<Node> path, Rotation2d rotation) {
        Pose2d[] finalList = new Pose2d[path.size()];
        for (int i=0; i<path.size(); i++) {
            finalList[i] = RobotContainer.Custom_to_WPI(new Pose2d(path.get(i).getLocation()[0], path.get(i).getLocation()[1], rotation));
        }
        return finalList;
    }

    public static void rectangularObstacle(Double[] topLeft, Double[] bottomRight) {
        if (grid == null) {
            grid = new ArrayList<Node>();
            for (double i=object_size[0]; i<grid_size[0]; i+=object_size[0]) {
                for (double j=object_size[1]; j<grid_size[1]; j+=object_size[1]) {
                    // System.out.printf("%s, %s\n", Math.round(1000.0*i)/1000.0, Math.round(1000.0*j)/1000.0);
                    grid.add(new Node(round(i), round(j), false));
                }
            }
        }
        for (Node g : grid) {
            Double[] gCoord = g.getLocation();
            // System.out.printf("%s %s\n", gCoord[0], gCoord[1]);
            if (gCoord[0] >= topLeft[0] && gCoord[0] <= bottomRight[0]) {
                if (gCoord[1] >= bottomRight[1] && gCoord[1] <= topLeft[1]) {
                    g.obstacle = true;
                    System.out.printf("Obstacle at (%s, %s)\n", g.getLocation()[0], g.getLocation()[1]);
                }
            }
        }
    }

    public static Pose2d[] setRotation(Pose2d[] path, int nodeOnPath, Rotation2d rotation) {
        if (nodeOnPath < 0) {
            path[path.length-1] = new Pose2d(path[path.length-1].getX(), path[path.length-1].getY(), Rotation2d.fromDegrees(90));
        } else {
            path[nodeOnPath] = new Pose2d(path[nodeOnPath].getX(), path[nodeOnPath].getY(), Rotation2d.fromDegrees(90));
        }
        return path;
    }

    // public static Pose2d[] setAllRotation(Pose2d[] path, Rotation2d rotation) {
    //     for (int i=0; i<path.length; i++) {
    //         path[i] = new Pose2d(path[i].getX(), path[i].getY(), rotation);
    //     }
    //     return path;
    // }
}
