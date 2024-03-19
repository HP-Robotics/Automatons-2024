package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.poly2tri.Poly2Tri;
import org.poly2tri.triangulation.TriangulationPoint;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;
import org.poly2tri.triangulation.point.TPoint;
import org.poly2tri.triangulation.sets.PointSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TriangleInterpolator {
  private PointSet m_pointSet;
  private ArrayList<double[]> m_calibratedList;
  private int m_currentIndex = 0;
  private List<TriangulationPoint> m_pointList;
  public TriangleInterpolator() {
    addCalibratedPoint(0, 0, 50, 50, 36, 90);
    addCalibratedPoint(0, 5, 60, 60, 49, 91);
    addCalibratedPoint(2, 0, 65, 65, 50, 105);
    m_pointSet = new PointSet(m_pointList);
    Poly2Tri.triangulate(m_pointSet);
  }

  public void addCalibratedPoint(double x, double y, double shooterSpeedLeft, double shooterSpeedRight, double pivotAngle, double heading) {
    double[] data = {shooterSpeedLeft, shooterSpeedRight, pivotAngle, heading};
    m_calibratedList.add(m_currentIndex, data);
    m_pointList.add(m_currentIndex, new TPoint(x, y, m_currentIndex));
  }


  private static double getTriangleArea(TriangulationPoint p1, TriangulationPoint p2, TriangulationPoint p3) {
    return ((p2.getX() - p1.getX()) * (p3.getY() - p1.getY()) - (p3.getX() - p1.getX()) * (p2.getY() - p1.getY())) / 2;
  }

  private static boolean isInsideTriangle(double[] weights) {
    // double totalArea = getTriangleArea(triangle.points[0], triangle.points[1], triangle.points[2]);
    // double areaA = getTriangleArea(point, triangle.points[1], triangle.points[2]);
    // double areaB = getTriangleArea(triangle.points[0], point, triangle.points[2]);
    // double areaC = getTriangleArea(triangle.points[0], triangle.points[1], point);

    // if (Math.abs(totalArea - (areaA + areaB + areaC)) < 0.001) {
    //   //TODO
    // }

    if (weights[0]<0 || weights[1]<0 || weights[2]<0) {
      return false;
    }
    return true;
  }

  // public double interpolate(double x, double y, double z, String type) {
  //   List<DelaunayTriangle> triangles = m_pointSet.getTriangles();
  //   TPoint point = new TPoint(x, y, z);
  //   for (DelaunayTriangle triangle : triangles) {
  //     double[] weights = getWeights(new Pose2d(x, y, new Rotation2d()), triangle);
  //     if (isInsideTriangle(point, weights)) {
  //       System.out.println(triangle + " contains " + point);
        
  //       double output = getData(triangle.points[0], type) * weights[0] + getData(triangle.points[1], type) * weights[1] 
  //       + getData(triangle.points[2], type) * weights[2];
  //       return output;
  //     }
  //   }
  //   return 0.0;
  // }

  // public double getData(TriangulationPoint point, String type) {
  //   return point.getZ(); // TODO: Use this like an index
  // }

  
  public double[] getWeights(Pose2d point, DelaunayTriangle triangle) {
    TriangulationPoint v1 = triangle.points[0];
    TriangulationPoint v2 = triangle.points[1];
    TriangulationPoint v3 = triangle.points[2];
    
    //Formula from: https://codeplea.com/triangular-interpolation
    double weight1 = (((v2.getY() - v3.getY()) * (point.getX() - v3.getX())) + ((v3.getX() - v2.getX()) * (point.getY() - v3.getY())))
    / (((v2.getY() - v3.getY()) * (v1.getX() - v3.getX())) + ((v3.getX() - v2.getX()) * (v1.getY() - v3.getY())));

    double weight2 = (((v3.getY() - v1.getY()) * (point.getX() - v3.getX()) + ((v1.getX() - v3.getX()) * (point.getY() - v3.getY())))
    / (((v2.getY() - v3.getY()) * (v1.getX() - v3.getX())) + ((v3.getX() - v2.getX()) * (v1.getY() - v3.getY()))));

    double weight3 = (1 - weight1 - weight2);

    double[] output = {weight1, weight2, weight3};
    return output;

  }

  public double[] getTriangulatedOutput(Pose2d robotPose) {
    List<DelaunayTriangle> triangles = m_pointSet.getTriangles();
    for (DelaunayTriangle triangle : triangles) {
      double[] weights = getWeights(new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d()), triangle);
      if (isInsideTriangle(weights)) {
        // TODO: Get the z from each of the three points, use it to look up calibration points, 
        //then use the weights * z (for each point) for each of the four data categories
      }
    }
    return null;
  }
}
