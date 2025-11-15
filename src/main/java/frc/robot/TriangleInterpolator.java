package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;

import org.poly2tri.Poly2Tri;
import org.poly2tri.geometry.polygon.Polygon;
import org.poly2tri.geometry.polygon.PolygonPoint;
import org.poly2tri.triangulation.TriangulationPoint;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;
import org.poly2tri.triangulation.point.TPoint;
import org.poly2tri.triangulation.sets.PointSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.LimelightConstants;
import java.awt.Color;

public class TriangleInterpolator {
  private PointSet m_pointSet;
  private ArrayList<double[]> m_calibratedList = new ArrayList<double[]>();
  private int m_currentIndex = 0;
  private List<TriangulationPoint> m_pointList = new ArrayList<TriangulationPoint>();
  private int m_outputSize;
  private static DelaunayTriangle m_lastTriangle;
  private static TriangulationPoint m_lastPoint;

  public TriangleInterpolator(int outputSize) {
    // addCalibratedPoint(new double[]{0, 0, 50, 50, 36, 90);
    // addCalibratedPoint(new double[]{0, 5, 60, 60, 49, 91);
    // addCalibratedPoint(new double[]{2, 0, 65, 65, 50, 105);
    m_outputSize = outputSize;
  }

  public void addCalibratedPoint(double[] dataPoint) {
    double[] data = dataPoint;
    m_calibratedList.add(m_currentIndex, data);
    m_pointList.add(m_currentIndex, new TPoint(dataPoint[0], dataPoint[1], m_currentIndex));
    m_currentIndex += 1;
  }

  public void makeTriangles() {
    m_pointSet = new PointSet(m_pointList);
    Poly2Tri.triangulate(m_pointSet);
  }

  public Polygon asPolygon() {
    PolygonPoint[] points = new PolygonPoint[m_pointSet.getPoints().size()];
    int i = 0;
    for (TriangulationPoint point : m_pointSet.getPoints()) {
      PolygonPoint polygonPoint = new PolygonPoint(point.getX(), point.getY());
      points[i] = polygonPoint;
      i++;
    }
    Polygon polygon = new Polygon(points);
    return polygon;
  }

  

  private static double getTriangleArea(TriangulationPoint p1, TriangulationPoint p2, TriangulationPoint p3) {
    return ((p2.getX() - p1.getX()) * (p3.getY() - p1.getY()) - (p3.getX() - p1.getX()) * (p2.getY() - p1.getY())) / 2;
  }

  private static boolean isInsideTriangle(double[] weights) {
    // double totalArea = getTriangleArea(triangle.points[0], triangle.points[1],
    // triangle.points[2]);
    // double areaA = getTriangleArea(point, triangle.points[1],
    // triangle.points[2]);
    // double areaB = getTriangleArea(triangle.points[0], point,
    // triangle.points[2]);
    // double areaC = getTriangleArea(triangle.points[0], triangle.points[1],
    // point);

    // if (Math.abs(totalArea - (areaA + areaB + areaC)) < 0.001) {
    // //TODO
    // }

    if (weights[0] < 0 || weights[1] < 0 || weights[2] < 0) {
      return false;
    }
    return true;
  }

  public double[] getData(TriangulationPoint point) {
    double[] output = m_calibratedList.get((int) point.getZ());
    return output;
  }

  public double[] getWeights(Pose2d point, DelaunayTriangle triangle) {
    TriangulationPoint v1 = triangle.points[0];
    TriangulationPoint v2 = triangle.points[1];
    TriangulationPoint v3 = triangle.points[2];

    // Formula from: https://codeplea.com/triangular-interpolation
    double weight1 = (((v2.getY() - v3.getY()) * (point.getX() - v3.getX()))
        + ((v3.getX() - v2.getX()) * (point.getY() - v3.getY())))
        / (((v2.getY() - v3.getY()) * (v1.getX() - v3.getX())) + ((v3.getX() - v2.getX()) * (v1.getY() - v3.getY())));

    double weight2 = (((v3.getY() - v1.getY()) * (point.getX() - v3.getX())
        + ((v1.getX() - v3.getX()) * (point.getY() - v3.getY())))
        / (((v2.getY() - v3.getY()) * (v1.getX() - v3.getX())) + ((v3.getX() - v2.getX()) * (v1.getY() - v3.getY()))));

    double weight3 = (1 - weight1 - weight2);

    double[] output = { weight1, weight2, weight3 };
    return output;

  }

  public void draw(String fileName, int width, int height, double minX, double maxX, double maxY, double minY,
      int dataIndex, double dataMin, double dataMax) {
    BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
    double currentX = minX;
    double currentY = minY;
    // System.out.println((maxY-minY)/(double)height);
    for (int i = 0; i < width; i++) {
      currentY = minY;
      for (int j = 0; j < height; j++) {
        // System.out.println(currentX);
        // System.out.println(currentY);
        Optional<double[]> colorData = getTriangulatedOutput(new Pose2d(currentX, currentY, new Rotation2d()));
        int color = 0;
        if (colorData.isPresent()) {
          double input = colorData.get()[dataIndex];
          color = (int) (255 * ((input - dataMin) / (dataMax - dataMin)));
          if (color > 255) {
            color = 255;
          }
          if (color < 0) {
            color = 0;
          }
          image.setRGB(i, j, new Color(color, color, color).getRGB());
        }
        currentY += (maxY - minY) / (double) height;
      }
      currentX += (maxX - minX) / (double) width;
    }

    try {
      File outputFile = new File(fileName);
      System.out.println(outputFile.getAbsolutePath());
      ImageIO.write(image, "png", outputFile);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /**
   * 
   * @param robotPose
   * @return Optional double array where 0th element is left shooter speed, 1st is
   *         right shooter speed, 2nd is pivot angle, 3rd is heading
   * 
   */
  public Optional<double[]> getTriangulatedOutput(Pose2d robotPose) {
    List<DelaunayTriangle> triangles = m_pointSet.getTriangles();
    for (DelaunayTriangle triangle : triangles) {
      double[] weights = getWeights(new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d()), triangle);
      if (isInsideTriangle(weights)) {
        // TODO: Get the z from each of the three points, use it to look up calibration
        // points,
        // then use the weights * z (for each point) for each of the four data
        // categories

        double[] output = { 0, 0, 0, 0 };

        for (int i = 0; i < m_outputSize; i += 1) {
          for (int j = 0; j < 3; j += 1) {
            output[i] += getData(triangle.points[j])[i] * weights[j];
          }
        }
        return Optional.of(output);
      }
    }
    return Optional.empty();
  }

  public Optional<double[]> getWalkingTriangulatedOutput(Pose2d robotPose) {
    List<DelaunayTriangle> triangles = m_pointSet.getTriangles();
    DelaunayTriangle currentTriangle = m_lastTriangle;
    double xyDistanceToQuery = Math.sqrt(Math.pow(m_lastPoint.getX() - robotPose.getX(), 2) + Math.pow(m_lastPoint.getY() - robotPose.getY(),2));
    for (DelaunayTriangle neighbor : m_lastTriangle.neighbors) {

    }
    return Optional.empty();
    // double[] weights = getWeights(new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d()), );

  }

  public static void addv2Magic(TriangleInterpolator m_triangleInterpolator) {
    m_triangleInterpolator.addCalibratedPoint(new double[] {-0.04, 4.07, 40.0, 30.0, 0.4250, -1.5708});
m_triangleInterpolator.addCalibratedPoint(new double[] {0.22, 4.09, 40.0, 30.0, 0.4250, -1.3963});
m_triangleInterpolator.addCalibratedPoint(new double[] {0.47, 4.16, 40.0, 30.0, 0.4250, -1.2217});
m_triangleInterpolator.addCalibratedPoint(new double[] {0.70, 4.27, 40.0, 30.0, 0.4250, -1.0472});
m_triangleInterpolator.addCalibratedPoint(new double[] {0.91, 4.41, 40.0, 30.0, 0.4250, -0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.10, 4.60, 40.0, 30.0, 0.4250, -0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.24, 4.81, 40.0, 30.0, 0.4250, -0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.35, 5.04, 40.0, 30.0, 0.4250, -0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.42, 5.29, 40.0, 30.0, 0.4250, -0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.44, 5.55, 40.0, 30.0, 0.4250, 0.0000});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.42, 5.80, 40.0, 30.0, 0.4250, 0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.35, 6.05, 40.0, 30.0, 0.4250, 0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.24, 6.29, 40.0, 30.0, 0.4250, 0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.10, 6.50, 40.0, 30.0, 0.4250, 0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {0.91, 6.68, 40.0, 30.0, 0.4250, 0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {0.70, 6.83, 40.0, 30.0, 0.4250, 1.0472});
m_triangleInterpolator.addCalibratedPoint(new double[] {0.47, 6.94, 40.0, 30.0, 0.4250, 1.2217});
m_triangleInterpolator.addCalibratedPoint(new double[] {0.22, 7.01, 40.0, 30.0, 0.4250, 1.3963});
m_triangleInterpolator.addCalibratedPoint(new double[] {-0.04, 7.03, 40.0, 30.0, 0.4250, 1.5708});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.54, 3.66, 44.0, 33.0, 0.3950, -0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.24, 2.83, 63.0, 46.5, 0.3640, -0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.52, 2.50, 72.0, 54.0, 0.3550, -0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.94, 1.99, 80.0, 60.0, 0.3450, -0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.20, 1.69, 80.0, 60.0, 0.3410, -0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.91, 3.23, 55.3, 41.0, 0.3750, -0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.85, 3.97, 44.0, 33.0, 0.3950, -0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.68, 3.27, 63.0, 46.5, 0.3640, -0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.01, 2.99, 72.0, 54.0, 0.3550, -0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.52, 2.57, 80.0, 60.0, 0.3450, -0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.82, 2.31, 80.0, 60.0, 0.3410, -0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.28, 3.60, 55.3, 41.0, 0.3750, -0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.09, 4.32, 44.0, 33.0, 0.3950, -0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.04, 3.77, 63.0, 46.5, 0.3640, -0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.41, 3.56, 72.0, 54.0, 0.3550, -0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.98, 3.23, 80.0, 60.0, 0.3450, -0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.32, 3.03, 80.0, 60.0, 0.3410, -0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.59, 4.03, 55.3, 41.0, 0.3750, -0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.27, 4.71, 44.0, 33.0, 0.3950, -0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.30, 4.33, 63.0, 46.5, 0.3640, -0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.70, 4.19, 72.0, 54.0, 0.3550, -0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.32, 3.96, 80.0, 60.0, 0.3450, -0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.69, 3.83, 80.0, 60.0, 0.3410, -0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.81, 4.51, 55.3, 41.0, 0.3750, -0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.38, 5.12, 44.0, 33.0, 0.3950, -0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.46, 4.93, 63.0, 46.5, 0.3640, -0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.88, 4.86, 72.0, 54.0, 0.3550, -0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.53, 4.74, 80.0, 60.0, 0.3450, -0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.92, 4.67, 80.0, 60.0, 0.3410, -0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.95, 5.02, 55.3, 41.0, 0.3750, -0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.42, 5.55, 44.0, 33.0, 0.3950, 0.0000});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.51, 5.55, 63.0, 46.5, 0.3640, 0.0000});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.94, 5.55, 72.0, 54.0, 0.3550, 0.0000});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.60, 5.55, 80.0, 60.0, 0.3450, 0.0000});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.99, 5.55, 80.0, 60.0, 0.3410, 0.0000});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.99, 5.55, 55.3, 41.0, 0.3750, 0.0000});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.38, 5.98, 44.0, 33.0, 0.3950, 0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.46, 6.16, 63.0, 46.5, 0.3640, 0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.88, 6.24, 72.0, 54.0, 0.3550, 0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.53, 6.35, 80.0, 60.0, 0.3450, 0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.92, 6.42, 80.0, 60.0, 0.3410, 0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.95, 6.07, 55.3, 41.0, 0.3750, 0.1745});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.27, 6.39, 44.0, 33.0, 0.3950, 0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.30, 6.76, 63.0, 46.5, 0.3640, 0.3791});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.70, 6.91, 72.0, 54.0, 0.3550, 0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.32, 7.13, 80.0, 60.0, 0.3450, 0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.69, 7.27, 80.0, 60.0, 0.3410, 0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.81, 6.58, 55.3, 41.0, 0.3750, 0.3491});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.09, 6.78, 44.0, 33.0, 0.3950, 0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.04, 7.32, 63.0, 46.5, 0.3640, 0.5336});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.41, 7.54, 72.0, 54.0, 0.3550, 0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.98, 7.87, 80.0, 60.0, 0.3450, 0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {4.32, 8.06, 80.0, 60.0, 0.3410, 0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.59, 7.06, 55.3, 41.0, 0.3750, 0.5236});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.85, 7.13, 44.0, 33.0, 0.3950, 0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.68, 7.83, 63.0, 46.5, 0.3640, 0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.01, 8.11, 72.0, 54.0, 0.3550, 0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.52, 8.53, 80.0, 60.0, 0.3450, 0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.82, 8.78, 80.0, 60.0, 0.3410, 0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.28, 7.50, 55.3, 41.0, 0.3750, 0.6981});
m_triangleInterpolator.addCalibratedPoint(new double[] {1.54, 7.43, 44.0, 33.0, 0.3950, 0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.24, 8.27, 63.0, 46.5, 0.3640, 0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.52, 8.60, 72.0, 54.0, 0.3550, 0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {2.94, 9.10, 80.0, 60.0, 0.3450, 0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[] {3.20, 9.40, 80.0, 60.0, 0.3410, 0.8727});
m_triangleInterpolator.addCalibratedPoint(new double[]{1.91, 7.87, 55.3, 41.0, 0.3750, 0.8727});
    // m_triangleInterpolator.addCalibratedPoint(new double[] {2.75, 2.48, 50.0, 50.0, 0.3606,
    // -0.8191});
    m_triangleInterpolator.addCalibratedPoint(new double[] {1.35, 4.52, 40.0, 30.0, 0.4163, -0.7601});
    m_triangleInterpolator.addCalibratedPoint(new double[] {1.26, 3.36, 44.0, 33.0, 0.3936, -1.1441});
    m_triangleInterpolator.addCalibratedPoint(new double[] {1.75, 2.51, 60.0, 45.0, 0.3726, -1.0717});
    m_triangleInterpolator.addCalibratedPoint(new double[] {2.24, 1.64, 80.0, 60.0, 0.3560, -1.0940});
    Pose2d tempPose = reflectAcrossSpeaker(new Pose2d(1.35, 4.52, new Rotation2d(-0.7601)));
    m_triangleInterpolator.addCalibratedPoint(new double[] {tempPose.getX(), tempPose.getY(), 40.0, 30.0, 0.4163,
        tempPose.getRotation().getRadians()});
    tempPose = reflectAcrossSpeaker(new Pose2d(1.26, 3.36, new Rotation2d(-1.1441)));
    m_triangleInterpolator.addCalibratedPoint(new double[] {tempPose.getX(), tempPose.getY(), 44.0, 33.0, 0.3936,
        tempPose.getRotation().getRadians()});
    tempPose = reflectAcrossSpeaker(new Pose2d(1.75, 2.51, new Rotation2d(-1.0717)));
    m_triangleInterpolator.addCalibratedPoint(new double[] {tempPose.getX(), tempPose.getY(), 60.0, 45.0, 0.3726,
        tempPose.getRotation().getRadians()});
    tempPose = reflectAcrossSpeaker(new Pose2d(2.24, 1.64, new Rotation2d(-1.0940)));
    m_triangleInterpolator.addCalibratedPoint(new double[] {tempPose.getX(), tempPose.getY(), 80.0, 60.0, 0.3560,
        tempPose.getRotation().getRadians()});
  }
  public static void addFeederMagic(TriangleInterpolator m_triangleInterpolator) {
    m_triangleInterpolator.addCalibratedPoint(new double[] {16.54-6.62, 0.30, 72.0, 48.0, 0.3902, -Math.PI-(-2.5456)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-7.54, 0.30, 69.0, 46.0, 0.3893, -Math.PI-(-2.4540)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-8.94, 0.30, 63.0, 42.0, 0.3952, -Math.PI-(-2.3679)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-10.17, 0.26, 61.5, 41.0, 0.4052,-Math.PI-( -2.2385)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-6.75, 2.03, 69.0, 46.0, 0.3912, -Math.PI-(-2.6028)}); 
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-7.46, 3.14, 60.0, 40.0, 0.4016, -Math.PI-(-2.6246)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-8.88, 3.44, 54.5, 37.0, 0.4037, -Math.PI-(-2.5910)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-10.54, 3.71, 54.5, 37.0, 0.4235,-Math.PI-( -2.4971)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-10.66, 2.14, 63.0, 42.0, 0.4317,-Math.PI-( -2.3330)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-9.69, 1.88, 60.0, 40.0, 0.4099, -Math.PI-(-2.3735)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-8.51, 2.02, 63.0, 42.0, 0.4098, -Math.PI-(-2.4483)});
    m_triangleInterpolator.addCalibratedPoint(new double[]{16.54-7.00, 3.50, 64.5, 43.0, 0.4014, -Math.PI-(-2.6425)});
  }


  public static Pose2d reflectAcrossSpeaker(Pose2d pose) {
    return new Pose2d(pose.getX(),
        2 * LimelightConstants.aprilTagList[7].getY() - pose.getY(),
        new Rotation2d(Math.abs(pose.getRotation().getRadians())));
  }

  public static void addDuluthMagic(TriangleInterpolator m_triangleInterpolator) {
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.41314058, 5.277596247, 50.0, 50.0, 0.4299444444, 0.018391});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.63832003, 4.041083114, 50.0, 50.0, 0.3752, -0.5839633072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.39589337, 5.103818848, 50.0, 50.0, 0.3913369795, -0.1972823072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.60848282, 5.33537256, 50.0, 50.0, 0.3873613361, -0.1375873072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.993256, 4.898600438, 50.0, 50.0, 0.3706087902, -0.2208183072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.15031274, 5.438757876, 50.0, 50.0, 0.397555186, -0.1114233072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.73930134, 4.462402822, 50.0, 50.0, 0.3801812462, -0.3653813072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.84648549, 4.602512007, 50.0, 50.0, 0.3790828376, -0.3130483072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.22354068, 4.812005882, 50.0, 50.0, 0.3935331628, -0.3240553072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.68715797, 5.127974466, 50.0, 50.0, 0.384781491, -0.1861133072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.433177119, 5.84757599, 50.0, 50.0, 0.3910294829, 0.162825});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.667053493, 6.422996478, 50.0, 50.0, 0.4036139448, 0.503259});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.110370235, 3.199645395, 50.0, 50.0, 0.3757948733, -0.767135});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.821643336, 6.529457233, 50.0, 50.0, 0.3996894324, 0.446951});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.974389758, 6.779834339, 50.0, 50.0, 0.3943659786, 0.44475});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.42353368, 5.249270573, 50.0, 50.0, 0.4299444444, 0.023444});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.6288601, 4.128261768, 50.0, 50.0, 0.3976305541, -0.7003673072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.17282885, 5.359810205, 50.0, 50.0, 0.3972661597, -0.1876743072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{3.09188213, 5.418303475, 50.0, 50.0, 0.3768686914, -0.06866430718});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.06881572, 3.553076124, 50.0, 50.0, 0.3819566015, -0.7638183072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.21142255, 3.969846769, 50.0, 50.0, 0.3851701862, -0.6058903072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.20776304, 4.333930512, 50.0, 50.0, 0.3895976252, -0.5137203072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.989285177, 5.90009964, 50.0, 50.0, 0.3787603253, 0.121146});
    m_triangleInterpolator.addCalibratedPoint(new double[]{3.231397389, 5.11207446, 50.0, 50.0, 0.3735021617, -0.143537});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.428012932, 5.518578913, 50.0, 50.0, 0.4299444444, 0.023172});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.620511598, 7.00530109, 50.0, 50.0, 0.3722, 0.654504});
    m_triangleInterpolator.addCalibratedPoint(new double[]{3.2347096, 5.481164722, 50.0, 50.0, 0.3738251226, -0.029818});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.15372857, 5.273657296, 50.0, 50.0, 0.3974545076, -0.175868});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.625809125, 5.58844815, 50.0, 50.0, 0.3871621243, 0.027729});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.42907256, 5.282004367, 50.0, 50.0, 0.4299444444, 0.047495});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.72226364, 5.389945935, 50.0, 50.0, 0.3866666667, -0.1016043072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.66843967, 4.050820611, 50.0, 50.0, 0.3752, -0.5548233072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.7672212, 4.852017983, 50.0, 50.0, 0.3821936829, -0.2448603072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.08542492, 4.403018504, 50.0, 50.0, 0.3926952363, -0.4770853072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.62826296, 4.211461271, 50.0, 50.0, 0.3801701784, -0.4704423072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.25477345, 4.802291869, 50.0, 50.0, 0.3927246721, -0.3276193072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.27049919, 3.56480995, 50.0, 50.0, 0.3788369363, -0.6984413072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.83415848, 6.754153281, 50.0, 50.0, 0.3771872945, 0.347943});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.32350358, 3.568570111, 50.0, 50.0, 0.3781536345, -0.6601933072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{0.98147648, 6.331711632, 50.0, 50.0, 0.4299444444, 0.915934});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.91771206, 4.64138053, 50.0, 50.0, 0.3982867717, -0.4211693072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.9385205, 6.046934573, 50.0, 50.0, 0.3794125407, 0.148292});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.474402511, 6.104722355, 50.0, 50.0, 0.3892631628, 0.18986});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.83351067, 5.361053277, 50.0, 50.0, 0.3825180341, -0.052574});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.332014535, 5.37853126, 50.0, 50.0, 0.3935172366, -0.07829});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.951643341, 5.370308324, 50.0, 50.0, 0.3799203072, -0.047325});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.617173736, 4.000621755, 50.0, 50.0, 0.378202804, -0.523976});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.787018731, 5.996831065, 50.0, 50.0, 0.3828741191, 0.14859});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.261165126, 4.218904245, 50.0, 50.0, 0.3873752575, -0.52738});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.990060638, 5.650284178, 50.0, 50.0, 0.4011063018, 0.015316});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.43384361, 5.306691403, 50.0, 50.0, 0.4299444444, 0.049197});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.83805863, 5.427026165, 50.0, 50.0, 0.3866666667, -0.06649230718});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.74222365, 4.032507656, 50.0, 50.0, 0.3751530014, -0.5369113072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.67425036, 3.830670437, 50.0, 50.0, 0.3751685375, -0.5275443072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{0.97538936, 6.476139411, 50.0, 50.0, 0.4299444444, 1.005968});
    m_triangleInterpolator.addCalibratedPoint(new double[]{1.9721541, 4.115437352, 50.0, 50.0, 0.3914683976, -0.6382893072});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.542504105, 5.472450674, 50.0, 50.0, 0.3890010817, -0.096369});
    m_triangleInterpolator.addCalibratedPoint(new double[]{2.72985008, 5.922860181, 50.0, 50.0, 0.3843019253, 0.148872});
  }
}
