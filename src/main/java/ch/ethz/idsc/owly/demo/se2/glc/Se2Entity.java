// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.Collections;
import java.util.Objects;

import javax.swing.JLabel;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2MinDistGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta4Integrator;
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

public class Se2Entity extends AbstractEntity {
  @SuppressWarnings("unused")
  private static final JLabel JLABEL = new JLabel();
  private static final Tensor FALLBACK_CONTROL = Array.zeros(2).unmodifiable(); // {angle=0, vel=0}
  private static final Scalar DELAY_HINT = RealScalar.ONE;
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { //
          { .2, +.07, 1 }, //
          { .2, -.07, 1 }, //
          { -.1, -.07, 1 }, //
          { -.1, +.07, 1 } //
      }).unmodifiable();
  private static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vector(1, 1, 2));
  private static final Tensor PARTITIONSCALE = Tensors.vector(5, 5, 50 / Math.PI); // 50/pi == 15.9155
  // ---
  static {
    if (!PARTITIONSCALE.get(0).equals(PARTITIONSCALE.get(1)))
      throw TensorRuntimeException.of(PARTITIONSCALE);
  }

  public static Se2Entity createDefault(Tensor state) {
    return new Se2Entity(state, RungeKutta4Integrator.INSTANCE);
  }

  // ---
  private final Integrator integrator;
  private final Collection<Flow> controls;
  private final Tensor goalRadius;
  private TrajectoryRegionQuery obstacleQuery = null;
  // private BufferedImage bufferedImage = null;

  private Se2Entity(Tensor state, Integrator integrator) {
    super(new SimpleEpisodeIntegrator( //
        Se2StateSpaceModel.INSTANCE, //
        integrator, //
        new StateTime(state, RealScalar.ZERO))); // initial position
    this.integrator = integrator;
    controls = Se2Controls.createControlsForwardAndReverse(RotationUtils.DEGREE(30), 6); // TODO magic const
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
    return FALLBACK_CONTROL;
  }

  @Override
  public Scalar delayHint() {
    return DELAY_HINT;
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.STANDARD;
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    GlobalAssert.that(VectorQ.ofLength(goal, 3));
    this.obstacleQuery = obstacleQuery;
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(integrator, RationalScalar.of(1, 10), 4); // TODO magic const
    Se2MinDistGoalManager se2MinDistGoalManager = new Se2MinDistGoalManager(goal, goalRadius);
    GoalInterface goalInterface = se2MinDistGoalManager.getGoalInterface();
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        PARTITIONSCALE, stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.represent = SE2WRAP::represent;
    return trajectoryPlanner;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      StateTime stateTime = getStateTimeNow();
      Color color = new Color(64, 64, 64, 128);
      if (Objects.nonNull(obstacleQuery))
        if (!obstacleQuery.isDisjoint(Collections.singletonList(stateTime)))
          color = new Color(255, 64, 64, 128);
      graphics.setColor(color);
      Tensor matrix = Se2Utils.toSE2Matrix(stateTime.state());
      Path2D path2d = owlyLayer.toPath2D(Tensor.of(SHAPE.flatten(0).map(matrix::dot)));
      graphics.fill(path2d);
    }
    { // indicate position delay[s] into the future
      Tensor state = getEstimatedLocationAt(DELAY_HINT);
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 64, 192));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
    {
      graphics.setColor(new Color(0, 128, 255, 192));
      Tensor matrix = owlyLayer.getMouseSe2Matrix();
      Path2D path2d = owlyLayer.toPath2D(Tensor.of(SHAPE.flatten(0).map(matrix::dot)));
      graphics.fill(path2d);
    }
    // {
    // Tensor model2pixel = owlyLayer.model2pixel();
    // Tensor translate = IdentityMatrix.of(3);
    // translate.set(RealScalar.of(-30), 0, 2); // pixel of rear axle
    // translate.set(RealScalar.of(-32), 1, 2); // image width/2
    // Tensor image = DiagonalMatrix.of(.005, .005, 1);
    // Tensor m = model2pixel.dot(owlyLayer.getMouseSe2Matrix()).dot(image).dot(translate);
    // AffineTransform at = new AffineTransform( //
    // m.Get(0, 0).number().doubleValue(), //
    // m.Get(1, 0).number().doubleValue(), //
    // m.Get(0, 1).number().doubleValue(), //
    // m.Get(1, 1).number().doubleValue(), //
    // m.Get(0, 2).number().doubleValue(), //
    // m.Get(1, 2).number().doubleValue());
    // GraphicsUtil.setQualityHigh(graphics);
    // graphics.drawImage(bufferedImage, at, JLABEL);
    // GraphicsUtil.setQualityDefault(graphics);
    // }
  }
}
