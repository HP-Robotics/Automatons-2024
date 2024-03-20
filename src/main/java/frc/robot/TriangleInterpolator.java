package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;

import org.poly2tri.Poly2Tri;
import org.poly2tri.triangulation.TriangulationPoint;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;
import org.poly2tri.triangulation.point.TPoint;
import org.poly2tri.triangulation.sets.PointSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.awt.Color;

public class TriangleInterpolator {
  private PointSet m_pointSet;
  private ArrayList<double[]> m_calibratedList = new ArrayList<double[]>();
  private int m_currentIndex = 0;
  private List<TriangulationPoint> m_pointList = new ArrayList<TriangulationPoint>();
  private int m_outputSize;

  public TriangleInterpolator(int outputSize) {
    // addCalibratedPoint(0, 0, 50, 50, 36, 90);
    // addCalibratedPoint(0, 5, 60, 60, 49, 91);
    // addCalibratedPoint(2, 0, 65, 65, 50, 105);
    m_outputSize = outputSize;
  }

  public void addCalibratedPoint(double x, double y, double shooterSpeedLeft, double shooterSpeedRight,
      double pivotAngle, double heading) {
    double[] data = { shooterSpeedLeft, shooterSpeedRight, pivotAngle, heading };
    m_calibratedList.add(m_currentIndex, data);
    m_pointList.add(m_currentIndex, new TPoint(x, y, m_currentIndex));
    m_currentIndex += 1;
  }

  public void makeTriangles() {
    m_pointSet = new PointSet(m_pointList);
    Poly2Tri.triangulate(m_pointSet);
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
        }
        image.setRGB(i, j, new Color(color, color, color).getRGB());
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

  public static void addDuluthMagic(TriangleInterpolator m_triangleInterpolator) {
    m_triangleInterpolator.addCalibratedPoint(1.41314058, 5.277596247, 50.0, 50.0, 0.4299444444, 3.018391);
    m_triangleInterpolator.addCalibratedPoint(2.63832003, 4.041083114, 50.0, 50.0, 0.3752, 0.580778);
    m_triangleInterpolator.addCalibratedPoint(2.39589337, 5.103818848, 50.0, 50.0, 0.3913369795, 0.194097);
    m_triangleInterpolator.addCalibratedPoint(2.60848282, 5.33537256, 50.0, 50.0, 0.3873613361, 0.134402);
    m_triangleInterpolator.addCalibratedPoint(2.993256, 4.898600438, 50.0, 50.0, 0.3706087902, 0.217633);
    m_triangleInterpolator.addCalibratedPoint(2.15031274, 5.438757876, 50.0, 50.0, 0.397555186, 0.108238);
    m_triangleInterpolator.addCalibratedPoint(2.73930134, 4.462402822, 50.0, 50.0, 0.3801812462, 0.362196);
    m_triangleInterpolator.addCalibratedPoint(2.84648549, 4.602512007, 50.0, 50.0, 0.3790828376, 0.309863);
    m_triangleInterpolator.addCalibratedPoint(2.22354068, 4.812005882, 50.0, 50.0, 0.3935331628, 0.32087);
    m_triangleInterpolator.addCalibratedPoint(2.68715797, 5.127974466, 50.0, 50.0, 0.384781491, 0.182928);
    m_triangleInterpolator.addCalibratedPoint(2.433177119, 5.84757599, 50.0, 50.0, 0.3910294829, 0.162825);
    m_triangleInterpolator.addCalibratedPoint(1.667053493, 6.422996478, 50.0, 50.0, 0.4036139448, 0.503259);
    m_triangleInterpolator.addCalibratedPoint(2.110370235, 3.199645395, 50.0, 50.0, 0.3757948733, 0.767135);
    m_triangleInterpolator.addCalibratedPoint(1.821643336, 6.529457233, 50.0, 50.0, 0.3996894324, 0.446951);
    m_triangleInterpolator.addCalibratedPoint(1.974389758, 6.779834339, 50.0, 50.0, 0.3943659786, 0.44475);
    m_triangleInterpolator.addCalibratedPoint(1.42353368, 5.249270573, 50.0, 50.0, 0.4299444444, 3.023444);
    m_triangleInterpolator.addCalibratedPoint(1.6288601, 4.128261768, 50.0, 50.0, 0.3976305541, 0.697182);
    m_triangleInterpolator.addCalibratedPoint(2.17282885, 5.359810205, 50.0, 50.0, 0.3972661597, 0.184489);
    m_triangleInterpolator.addCalibratedPoint(3.09188213, 5.418303475, 50.0, 50.0, 0.3768686914, 0.065479);
    m_triangleInterpolator.addCalibratedPoint(2.06881572, 3.553076124, 50.0, 50.0, 0.3819566015, 0.760633);
    m_triangleInterpolator.addCalibratedPoint(2.21142255, 3.969846769, 50.0, 50.0, 0.3851701862, 0.602705);
    m_triangleInterpolator.addCalibratedPoint(2.20776304, 4.333930512, 50.0, 50.0, 0.3895976252, 0.510535);
    m_triangleInterpolator.addCalibratedPoint(2.989285177, 5.90009964, 50.0, 50.0, 0.3787603253, 0.121146);
    m_triangleInterpolator.addCalibratedPoint(3.231397389, 5.11207446, 50.0, 50.0, 0.3735021617, 0.143537);
    m_triangleInterpolator.addCalibratedPoint(1.428012932, 5.518578913, 50.0, 50.0, 0.4299444444, 0.023172);
    m_triangleInterpolator.addCalibratedPoint(2.620511598, 7.00530109, 50.0, 50.0, 0.3722, 0.654504);
    m_triangleInterpolator.addCalibratedPoint(3.2347096, 5.481164722, 50.0, 50.0, 0.3738251226, 0.029818);
    m_triangleInterpolator.addCalibratedPoint(2.15372857, 5.273657296, 50.0, 50.0, 0.3974545076, 0.175868);
    m_triangleInterpolator.addCalibratedPoint(2.625809125, 5.58844815, 50.0, 50.0, 0.3871621243, 0.027729);
    m_triangleInterpolator.addCalibratedPoint(1.42907256, 5.282004367, 50.0, 50.0, 0.4299444444, 3.047495);
    m_triangleInterpolator.addCalibratedPoint(2.72226364, 5.389945935, 50.0, 50.0, 0.3866666667, 0.098419);
    m_triangleInterpolator.addCalibratedPoint(2.66843967, 4.050820611, 50.0, 50.0, 0.3752, 0.551638);
    m_triangleInterpolator.addCalibratedPoint(2.7672212, 4.852017983, 50.0, 50.0, 0.3821936829, 0.241675);
    m_triangleInterpolator.addCalibratedPoint(2.08542492, 4.403018504, 50.0, 50.0, 0.3926952363, 0.4739);
    m_triangleInterpolator.addCalibratedPoint(2.62826296, 4.211461271, 50.0, 50.0, 0.3801701784, 0.467257);
    m_triangleInterpolator.addCalibratedPoint(2.25477345, 4.802291869, 50.0, 50.0, 0.3927246721, 0.324434);
    m_triangleInterpolator.addCalibratedPoint(2.27049919, 3.56480995, 50.0, 50.0, 0.3788369363, 0.695256);
    m_triangleInterpolator.addCalibratedPoint(2.83415848, 6.754153281, 50.0, 50.0, 0.3771872945, 2.347943);
    m_triangleInterpolator.addCalibratedPoint(2.32350358, 3.568570111, 50.0, 50.0, 0.3781536345, 0.657008);
    m_triangleInterpolator.addCalibratedPoint(0.98147648, 6.331711632, 50.0, 50.0, 0.4299444444, 2.915934);
    m_triangleInterpolator.addCalibratedPoint(1.91771206, 4.64138053, 50.0, 50.0, 0.3982867717, 0.417984);
    m_triangleInterpolator.addCalibratedPoint(2.9385205, 6.046934573, 50.0, 50.0, 0.3794125407, 2.148292);
    m_triangleInterpolator.addCalibratedPoint(2.474402511, 6.104722355, 50.0, 50.0, 0.3892631628, 0.18986);
    m_triangleInterpolator.addCalibratedPoint(2.83351067, 5.361053277, 50.0, 50.0, 0.3825180341, 0.052574);
    m_triangleInterpolator.addCalibratedPoint(2.332014535, 5.37853126, 50.0, 50.0, 0.3935172366, 0.07829);
    m_triangleInterpolator.addCalibratedPoint(2.951643341, 5.370308324, 50.0, 50.0, 0.3799203072, 0.047325);
    m_triangleInterpolator.addCalibratedPoint(2.617173736, 4.000621755, 50.0, 50.0, 0.378202804, 0.523976);
    m_triangleInterpolator.addCalibratedPoint(2.787018731, 5.996831065, 50.0, 50.0, 0.3828741191, 0.14859);
    m_triangleInterpolator.addCalibratedPoint(2.261165126, 4.218904245, 50.0, 50.0, 0.3873752575, 0.52738);
    m_triangleInterpolator.addCalibratedPoint(1.990060638, 5.650284178, 50.0, 50.0, 0.4011063018, 0.015316);
    m_triangleInterpolator.addCalibratedPoint(1.43384361, 5.306691403, 50.0, 50.0, 0.4299444444, 3.049197);
    m_triangleInterpolator.addCalibratedPoint(2.83805863, 5.427026165, 50.0, 50.0, 0.3866666667, 0.063307);
    m_triangleInterpolator.addCalibratedPoint(2.74222365, 4.032507656, 50.0, 50.0, 0.3751530014, 0.533726);
    m_triangleInterpolator.addCalibratedPoint(2.67425036, 3.830670437, 50.0, 50.0, 0.3751685375, 0.524359);
    m_triangleInterpolator.addCalibratedPoint(0.97538936, 6.476139411, 50.0, 50.0, 0.4299444444, 3.005968);
    m_triangleInterpolator.addCalibratedPoint(1.9721541, 4.115437352, 50.0, 50.0, 0.3914683976, 0.635104);
    m_triangleInterpolator.addCalibratedPoint(2.542504105, 5.472450674, 50.0, 50.0, 0.3890010817, 0.096369);
    m_triangleInterpolator.addCalibratedPoint(2.72985008, 5.922860181, 50.0, 50.0, 0.3843019253, 0.148872);
  }
}
