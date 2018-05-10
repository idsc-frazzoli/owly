// code by ynager
package ch.ethz.idsc.owl.mapping;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.awt.image.WritableRaster;
import java.util.Arrays;

import ch.ethz.idsc.owl.gui.AffineTransforms;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.math.region.BoundedBoxRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Sign;

/** all pixels have the same amount of weight or clearance radius attached */
public class BayesianOccupancyGrid implements RenderInterface {
  private static final int OCCUPIED_COLOR = 0xFFFF << 16;
  private static final int EMPTY_COLOR = 0x4000FF << 8;
  private static final int UNKNOWN_COLOR = 0x7F909090;
  // ---
  private final Tensor lbounds;
  private final Scalar gridRes; // [m] per cell
  private final Scalar gridInv; // cells per [m]
  private final Tensor gridSize; // grid size in pixels
  private final Region<Tensor> gridRegion; // grid region in global coordinates
  private final Region<Tensor> obsRegion; // observation region in global coordinates
  // ---
  private double[] logOdds; // array of current log odd of each cell
  private final BufferedImage bufferedImage; // maximum likelihood obstacle map
  private int[] imagePixels;
  private final Graphics2D imageGraphics;
  // ---
  private final static double P_M = 0.5f; // prior
  private final static double P_M_HIT = 0.9f; // inv sensor model p(m|z)
  private final static double probThreshold = 0.5; // cells with p(m|z_1:t) > probThreshold are considered occupied
  private final double lThreshold;
  // ---
  private final static int TYPE_HIT_FLOOR = 0; // TODO define somewhere in lidar module
  private final static int TYPE_HIT_OBSTACLE = 1;
  // ---
  private Scalar obsDilationRadius;

  /** @param lbounds vector of length 2
   * @param ubounds vector of length 2
   * @param gridRes */
  public BayesianOccupancyGrid(Tensor lbounds, Tensor ubounds, Scalar gridRes) {
    this.lbounds = lbounds;
    this.gridRes = Sign.requirePositive(gridRes);
    gridInv = gridRes.reciprocal();
    gridSize = Ceiling.of(ubounds.subtract(lbounds).multiply(gridInv));
    bufferedImage = new BufferedImage( //
        gridSize.Get(0).number().intValue(), //
        gridSize.Get(1).number().intValue(), //
        BufferedImage.TYPE_INT_ARGB);
    WritableRaster writableRaster = bufferedImage.getRaster();
    DataBufferInt dataBufferByte = (DataBufferInt) writableRaster.getDataBuffer();
    imagePixels = dataBufferByte.getData();
    // ---
    imageGraphics = (Graphics2D) bufferedImage.getGraphics();
    Tensor center = Mean.of(Tensors.of(lbounds, ubounds));
    gridRegion = new BoundedBoxRegion(center, ubounds.subtract(center));
    obsRegion = gridRegion;
    obsDilationRadius = gridRes.divide(RealScalar.of(2));
    // ---
    int arrayLength = gridSize.Get(0).multiply(gridSize.Get(1)).number().intValue();
    logOdds = new double[arrayLength];
    Arrays.fill(logOdds, pToLogOdd(P_M)); // fill with prior P_M
    lThreshold = pToLogOdd(probThreshold); // convert prob threshold to logOdd threshold
    // ---
    Graphics graphics = bufferedImage.getGraphics();
    graphics.setColor(new Color(UNKNOWN_COLOR, true));
    graphics.fillRect(0, 0, bufferedImage.getWidth(), bufferedImage.getHeight());
  }

  /** process a new lidar observation and update the occupancy map
   * @param pos 2D position of new lidar observation in global coordinates
   * @param type of observation */
  public void processObservation(Tensor pos, int type) {
    if (obsRegion.isMember(pos)) {
      double p_m_z = P_M; // default p(m|z) to prior
      switch (type) {
      case TYPE_HIT_OBSTACLE:
        p_m_z = P_M_HIT; // prob cell is occupied given lidar hit obstacle
        break;
      case TYPE_HIT_FLOOR:
        p_m_z = 1 - P_M_HIT; // prob cell is occupied given lidar hit floor
        break;
      }
      Tensor cell = toCell(pos);
      int idx = cellToIdx(cell);
      updateCellLogOdd(idx, p_m_z);
      // ---
      // Maximum likelihood estimation
      double logOdd = logOdds[idx];
      if (logOdd > lThreshold)
        drawSphere(pos, obsDilationRadius, OCCUPIED_COLOR);
      else if (logOdd < lThreshold)
        drawCell(cell, EMPTY_COLOR);
      else {
        drawCell(cell, UNKNOWN_COLOR);
      }
    }
  }

  /** cells within this radius of an occupied cell will also be labeled as occupied.
   * If not set or below gridRes, only the occupied cell is labeled as an obstacle
   * @param radius */
  public void setObstacleRadius(Scalar radius) {
    obsDilationRadius = radius;
  }

  /** Update the log odds of a cell using the probability of occupation given a new observation
   * @param idx of cell to be updated
   * @param p_m_z probability in [0,1] that Cell is occupied given the current observation z */
  private void updateCellLogOdd(int idx, double p_m_z) {
    logOdds[idx] = logOdds[idx] + Math.log(p_m_z / (1 - p_m_z)) + Math.log((1 - P_M) / P_M);
  }

  private double pToLogOdd(double p) {
    return Math.log(p / (1 - p));
  }

  private Tensor toCell(Tensor pos) {
    return Floor.of(pos.extract(0, 2).subtract(lbounds).multiply(gridInv));
  }

  private int cellToIdx(Tensor cell) {
    return cell.Get(1).multiply(gridSize.Get(1)).add(cell.Get(0)).number().intValue();
  }

  private void drawCell(Tensor cell, int color) {
    imagePixels[cellToIdx(cell)] = color;
  }

  private void drawSphere(Tensor pos, Scalar radius, int color) {
    if (Scalars.lessEquals(obsDilationRadius, gridRes)) {
      drawCell(toCell(pos), color);
      return;
    }
    Scalar radiusScaled = radius.multiply(gridInv);
    double dim = radiusScaled.number().doubleValue();
    Ellipse2D sphere = new Ellipse2D.Double( //
        pos.Get(0).multiply(gridInv).subtract(radiusScaled).number().doubleValue(), //
        pos.Get(1).multiply(gridInv).subtract(radiusScaled).number().doubleValue(), //
        2 * dim, 2 * dim);
    imageGraphics.setColor(new Color(OCCUPIED_COLOR, true));
    imageGraphics.fill(sphere);
  }

  @Override // from Renderinterface
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    final Tensor matrix = geometricLayer.getMatrix();
    AffineTransform affineTransform = AffineTransforms.toAffineTransform(matrix);
    final double scale = gridRes.number().doubleValue();
    affineTransform.scale(scale, scale);
    affineTransform.translate( //
        lbounds.Get(0).multiply(gridInv).number().doubleValue(), //
        lbounds.Get(1).multiply(gridInv).number().doubleValue());
    graphics.drawImage(bufferedImage, affineTransform, null);
  }
}
