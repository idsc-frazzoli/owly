// code by ynager
package ch.ethz.idsc.owl.mapping;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;
import java.util.Arrays;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.gui.AffineTransforms;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.math.region.BoundedExclusiveBoxRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Log;
import ch.ethz.idsc.tensor.sca.Sign;

/** all pixels have the same amount of weight or clearance radius attached */
public class BayesianOccupancyGrid implements Region<Tensor>, RenderInterface {
  // private static final Set<Byte> VALUES = new HashSet<>();
  private static final short MASK_OCCUPIED = 0x00;
  private static final short MASK_EMPTY = 0xFF;
  private static final short MASK_UNKNOWN = 0xDD;
  private static final Color COLOR_OCCUPIED = new Color(MASK_OCCUPIED, MASK_OCCUPIED, MASK_OCCUPIED);
  // ---
  private Tensor lbounds;
  private final Scalar cellDim; // [m] per cell
  private final Scalar cellDimInv; // cells per [m]
  private final Tensor gridSize; // grid size in pixels
  private final Tensor gridRange; // grid size in coordinate space
  private Region<Tensor> gridRegion; // grid region in global coordinates
  // ---
  private double[] logOdds; // array of current log odd of each cell
  private BufferedImage bufferedImage; // maximum likelihood obstacle map
  private byte[] imagePixels;
  private final Graphics2D imageGraphics;
  // ---
  private static final Scalar P_M = DoubleScalar.of(0.5); // prior
  private static final Scalar P_M_HIT = DoubleScalar.of(0.9); // inv sensor model p(m|z)
  private static final Scalar priorInvLogOdd = pToLogOdd(RealScalar.ONE.subtract(P_M));
  private static final Scalar probThreshold = DoubleScalar.of(0.5); // cells with p(m|z_1:t) > probThreshold are considered occupied
  private final Scalar lThreshold;
  // ---
  public static final int TYPE_HIT_FLOOR = 0; // TODO define somewhere in lidar module
  public static final int TYPE_HIT_OBSTACLE = 1;
  // ---
  private Scalar obsDilationRadius;
  private final Tensor scaling;
  private final Scalar[] PREDEFINED_P;

  /** Returns an instance of BayesianOccupancyGrid whose grid dimensions are ceiled to
   * fit a whole number of cells per dimension
   * @param lbounds vector of length 2
   * @param range effective size of grid in coordinate space
   * @param cellDim dimension of cell in [m]
   * @return BayesianOccupancyGrid */
  public static BayesianOccupancyGrid of(Tensor lbounds, Tensor range, Scalar cellDim) {
    Tensor sizeCeil = Ceiling.of(range.divide(Sign.requirePositive(cellDim)));
    Tensor rangeCeil = sizeCeil.multiply(cellDim);
    return new BayesianOccupancyGrid(lbounds, rangeCeil, sizeCeil);
  }

  /** @param lbounds vector of length 2
   * @param range effective size of grid in coordinate space
   * @param size size of grid in cell space */
  public BayesianOccupancyGrid(Tensor lbounds, Tensor range, Tensor size) {
    GlobalAssert.that(VectorQ.ofLength(lbounds, 2));
    GlobalAssert.that(VectorQ.ofLength(range, 2));
    GlobalAssert.that(VectorQ.ofLength(size, 2));
    System.out.print("Grid range: " + range + "\n");
    System.out.print("Grid size: " + size + "\n");
    PREDEFINED_P = new Scalar[] { //
        RealScalar.ONE.subtract(P_M_HIT), P_M_HIT };
    this.lbounds = lbounds;
    this.gridSize = size;
    this.gridRange = range;
    this.cellDim = range.Get(0).divide(gridSize.Get(0)); // TODO just 1st dim is checked
    Tensor ubounds = lbounds.add(range);
    cellDimInv = cellDim.reciprocal();
    bufferedImage = new BufferedImage( //
        gridSize.Get(0).number().intValue(), //
        gridSize.Get(1).number().intValue(), //
        BufferedImage.TYPE_BYTE_GRAY);
    WritableRaster writableRaster = bufferedImage.getRaster();
    DataBufferByte dataBufferByte = (DataBufferByte) writableRaster.getDataBuffer();
    imagePixels = dataBufferByte.getData();
    // ---
    imageGraphics = bufferedImage.createGraphics();
    Tensor center = Mean.of(Tensors.of(lbounds, ubounds));
    gridRegion = new BoundedExclusiveBoxRegion(center, ubounds.subtract(center));
    obsDilationRadius = cellDim.divide(RealScalar.of(2));
    // ---
    int arrayLength = gridSize.Get(0).multiply(gridSize.Get(1)).number().intValue();
    logOdds = new double[arrayLength];
    Arrays.fill(logOdds, pToLogOdd(P_M).number().doubleValue()); // fill with prior P_M
    lThreshold = pToLogOdd(probThreshold); // convert prob threshold to logOdd threshold
    // ---
    Graphics graphics = bufferedImage.getGraphics();
    graphics.setColor(new Color(MASK_UNKNOWN, MASK_UNKNOWN, MASK_UNKNOWN));
    graphics.fillRect(0, 0, bufferedImage.getWidth(), bufferedImage.getHeight());
    // ---
    scaling = DiagonalMatrix.of( //
        cellDim.number().doubleValue(), //
        cellDim.number().doubleValue(), 1);
  }

  /** process a new lidar observation and update the occupancy map
   * @param pos 2D position of new lidar observation in global coordinates
   * @param type of observation */
  public void processObservation(Tensor pos, int type) {
    if (gridRegion.isMember(pos)) { // check if measurement is inside current grid region
      Scalar p_m_z = P_M; // default p(m|z) to prior
      p_m_z = PREDEFINED_P[type];
      // switch (type) {
      // case TYPE_HIT_FLOOR: // 0
      // p_m_z = RealScalar.ONE.subtract(P_M_HIT); // prob cell is occupied given lidar hit floor
      // break;
      // case TYPE_HIT_OBSTACLE: // 1
      // p_m_z = P_M_HIT; // prob cell is occupied given lidar hit obstacle
      // break;
      // }
      Tensor cell = toCell(pos);
      int idx = cellToIdx(cell);
      updateCellLogOdd(idx, p_m_z);
      // ---
      // Maximum likelihood estimation
      Scalar logOdd = DoubleScalar.of(logOdds[idx]);
      if (Scalars.lessThan(lThreshold, logOdd))
        drawSphere(pos, obsDilationRadius, MASK_OCCUPIED);
      else if (Scalars.lessThan(logOdd, lThreshold))
        drawCell(cell, MASK_EMPTY);
      else
        drawCell(cell, MASK_UNKNOWN);
    }
  }

  /** cells within this radius of an occupied cell will also be labeled as occupied.
   * If not set or below cellDim, only the occupied cell is labeled as an obstacle
   * @param radius */
  public void setObstacleRadius(Scalar radius) {
    obsDilationRadius = radius;
  }

  /** Updates the grid center. Grid range and size remain unchanged.
   * Overlapping segments remain unchanged.
   * 
   * @param state center of new grid */
  public void updateGridCenter(Tensor state) {
    // FIXME WIP
    Tensor radii = gridRange.divide(RealScalar.of(2));
    gridRegion = new BoundedExclusiveBoxRegion(state, radii);
    Tensor lboundsNew = state.subtract(radii);
    Tensor tVec = lboundsNew.subtract(lbounds);
    lbounds = lboundsNew;
    AffineTransform tx = new AffineTransform();
    tx.translate(tVec.Get(0).number().doubleValue(), tVec.Get(1).number().doubleValue());
    AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_NEAREST_NEIGHBOR);
    bufferedImage = op.filter(bufferedImage, null);
    // TODO translate elements of logOdds array
  }

  /** Update the log odds of a cell using the probability of occupation given a new observation.
   * l_t = l_{t-1} + log[ p(m|z_t) / (1 - p(m|z_t)) ] + log[ (1-p(m)) / p(m) ]
   * @param idx of cell to be updated
   * @param p_m_z probability in [0,1] that Cell is occupied given the current observation z */
  private void updateCellLogOdd(int idx, Scalar p_m_z) {
    Scalar logOddDelta = pToLogOdd(p_m_z).add(priorInvLogOdd);
    logOdds[idx] += logOddDelta.number().doubleValue();
    if (Double.isInfinite(logOdds[idx]))
      throw new ArithmeticException("Overflow");
  }

  private static Scalar pToLogOdd(Scalar p) {
    return Log.FUNCTION.apply(p.divide(RealScalar.ONE.subtract(p)));
  }

  private Tensor toCell(Tensor pos) {
    GlobalAssert.that(gridRegion.isMember(pos));
    return Floor.of(pos.extract(0, 2).subtract(lbounds).multiply(cellDimInv));
  }

  private int cellToIdx(Tensor cell) {
    return cell.Get(1).multiply(gridSize.Get(0)).add(cell.Get(0)).number().intValue();
  }

  private void drawCell(Tensor cell, short grayScale) {
    int idx = cellToIdx(cell);
    if (idx < imagePixels.length)
      imagePixels[idx] = (byte) (grayScale & 0xFF);
  }

  private void drawSphere(Tensor pos, Scalar radius, short grayScale) {
    if (Scalars.lessEquals(obsDilationRadius, cellDim)) {
      drawCell(toCell(pos), grayScale);
      return;
    }
    Scalar radiusScaled = radius.multiply(cellDimInv);
    double dim = radiusScaled.number().doubleValue();
    Ellipse2D sphere = new Ellipse2D.Double( //
        pos.Get(0).multiply(cellDimInv).subtract(radiusScaled).number().doubleValue(), //
        pos.Get(1).multiply(cellDimInv).subtract(radiusScaled).number().doubleValue(), //
        2 * dim, 2 * dim);
    imageGraphics.setColor(COLOR_OCCUPIED);
    imageGraphics.fill(sphere);
  }

  @Override // from Region<Tensor>
  public boolean isMember(Tensor state) {
    if (gridRegion.isMember(state)) {
      byte gs = imagePixels[cellToIdx(toCell(state))];
      return gs == MASK_OCCUPIED;
    }
    return true;
  }

  @Override // from Renderinterface
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Tensor model2pixel = geometricLayer.getMatrix();
    Tensor translate = IdentityMatrix.of(3);
    translate.set(lbounds.get(0).multiply(cellDimInv), 0, 2);
    translate.set(lbounds.get(1).multiply(cellDimInv), 1, 2);
    Tensor matrix = model2pixel.dot(scaling).dot(translate);
    graphics.drawImage(bufferedImage, AffineTransforms.toAffineTransform(matrix), null);
  }
}
