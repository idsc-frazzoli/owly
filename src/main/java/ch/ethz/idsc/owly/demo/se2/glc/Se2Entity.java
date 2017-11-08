// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.Collections;
import java.util.Objects;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeMinShiftExtraCostGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeMinShiftGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.StateTimeTensorFunction;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.se2.Se2Utils;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.sca.Sqrt;

/** several magic constants are hard-coded in the implementation.
 * that means, the functionality does not apply to all examples universally. */
class Se2Entity extends AbstractEntity {
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { //
          { .2, +.07, 1 }, //
          { .2, -.07, 1 }, //
          { -.1, -.07, 1 }, //
          { -.1, +.07, 1 } //
      }).unmodifiable();
  static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vector(1, 1, 2));
  static final Tensor PARTITIONSCALE = Tensors.vector(5, 5, 50 / Math.PI).unmodifiable(); // 50/pi == 15.9155
  // ---
  static {
    if (!PARTITIONSCALE.get(0).equals(PARTITIONSCALE.get(1)))
      throw TensorRuntimeException.of(PARTITIONSCALE);
  }

  public static Se2Entity createDefault(Tensor state) {
    return new Se2Entity(state);
  }

  // ---
  private final Collection<Flow> controls;
  private final Tensor goalRadius;
  public TrajectoryRegionQuery obstacleQuery = null;
  // private BufferedImage bufferedImage = null;

  Se2Entity(Tensor state) {
    super(new SimpleEpisodeIntegrator( //
        Se2StateSpaceModel.INSTANCE, //
        Se2CarIntegrator.INSTANCE, //
        new StateTime(state, RealScalar.ZERO))); // initial position
    controls = Se2Controls.createControlsForwardAndReverse(RotationUtils.DEGREE(45), 6);
    final Scalar goalRadius_xy = Sqrt.of(RealScalar.of(2)).divide(PARTITIONSCALE.Get(0));
    final Scalar goalRadius_theta = Sqrt.of(RealScalar.of(2)).divide(PARTITIONSCALE.Get(2));
    goalRadius = Tensors.of(goalRadius_xy, goalRadius_xy, goalRadius_theta);
    // try {
    // bufferedImage = ImageIO.read(UserHome.Pictures("car_green.png"));
    // } catch (Exception exception) {
    // exception.printStackTrace();
    // }
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return SE2WRAP.distance(x, y);
  }

  @Override
  protected Tensor fallbackControl() {
    return Array.zeros(3).unmodifiable(); // {vx, vy, rate}
  }

  @Override
  public Scalar delayHint() {
    return RealScalar.of(1.5);
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.STANDARD;
  }

  // TODO JAN design is despicable
  public CostFunction costFunction = null;

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    GlobalAssert.that(VectorQ.ofLength(goal, 3));
    this.obstacleQuery = obstacleQuery;
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(Se2CarIntegrator.INSTANCE, RationalScalar.of(1, 10), 4);
    GoalInterface goalInterface = Objects.isNull(costFunction) ? //
        Se2MinTimeMinShiftGoalManager.create(goal, goalRadius, controls) : //
        Se2MinTimeMinShiftExtraCostGoalManager.create(goal, goalRadius, controls, costFunction);
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta(), stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.represent = StateTimeTensorFunction.state(SE2WRAP::represent);
    return trajectoryPlanner;
  }

  protected Tensor eta() {
    return PARTITIONSCALE;
  }

  private boolean obstacleQuery_isDisjoint(StateTime stateTime) {
    if (Objects.nonNull(obstacleQuery))
      return obstacleQuery.isDisjoint(Collections.singletonList(stateTime));
    return true;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    { // indicate current position
      final StateTime stateTime = getStateTimeNow();
      Color color = new Color(64, 64, 64, 128);
      if (!obstacleQuery_isDisjoint(stateTime))
        color = new Color(255, 64, 64, 128);
      geometricLayer.pushMatrix(Se2Utils.toSE2Matrix(stateTime.state()));
      graphics.setColor(color);
      graphics.fill(geometricLayer.toPath2D(SHAPE));
      geometricLayer.popMatrix();
    }
    { // indicate position delay[s] into the future
      Tensor state = getEstimatedLocationAt(delayHint());
      Point2D point = geometricLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 64, 192));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
    {
      Color color = new Color(0, 128, 255, 192);
      StateTime stateTime = new StateTime(geometricLayer.getMouseSe2State(), getStateTimeNow().time());
      if (!obstacleQuery_isDisjoint(stateTime))
        color = new Color(255, 96, 96, 128);
      geometricLayer.pushMatrix(geometricLayer.getMouseSe2Matrix());
      graphics.setColor(color);
      graphics.fill(geometricLayer.toPath2D(SHAPE));
      geometricLayer.popMatrix();
    }
    // {
    // Tensor model2pixel = owlyLayer.model2pixel();
    // Tensor translate = IdentityMatrix.of(3);
    // translate.set(RealScalar.of(-30), 0, 2); // pixel of rear axle
    // translate.set(RealScalar.of(-32), 1, 2); // image width/2
    // Tensor image = DiagonalMatrix.of(.005, .005, 1);
    // Tensor m = model2pixel.dot(owlyLayer.getMouseSe2Matrix()).dot(image).dot(translate);
    // GraphicsUtil.setQualityHigh(graphics);
    // graphics.drawImage(bufferedImage, at, null);
    // GraphicsUtil.setQualityDefault(graphics);
    // }
  }
}
