// code by jph
package ch.ethz.idsc.owly.demo.twd;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
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
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.red.ArgMin;
import ch.ethz.idsc.tensor.sca.Sqrt;

public class TwdEntity extends AbstractEntity {
  private static final Tensor FALLBACK_CONTROL = Tensors.vector(0, 0).unmodifiable();
  private static final Scalar DELAY_HINT = RealScalar.ONE;
  // triangle
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { { .3, 0, 1 }, { -.1, -.1, 1 }, { -.1, +.1, 1 } }).unmodifiable();
  private static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vector(1, 1, 2));
  private static final Tensor PARTITIONSCALE = Tensors.vector(4, 4, 50 / Math.PI); // 50/pi == 15.9155
  // ---
  static {
    if (!PARTITIONSCALE.get(0).equals(PARTITIONSCALE.get(1)))
      throw TensorRuntimeException.of(PARTITIONSCALE);
  }

  public static TwdEntity createDefault(Tensor state) {
    return new TwdEntity(TwdStateSpaceModel.createDefault(), MidpointIntegrator.INSTANCE, state);
  }

  // ---
  final Integrator integrator;
  final Collection<Flow> controls;
  final Scalar goalRadius_xy;
  final Scalar goalRadius_theta;
  TrajectoryRegionQuery obstacleQuery = null;

  public TwdEntity(TwdStateSpaceModel twdStateSpaceModel, Integrator integrator, Tensor state) {
    super(new SimpleEpisodeIntegrator( //
        twdStateSpaceModel, //
        integrator, //
        new StateTime(state, RealScalar.ZERO))); // initial position
    this.integrator = integrator;
    controls = TwdControls.createControls(twdStateSpaceModel, 8);
    goalRadius_xy = Sqrt.of(RealScalar.of(2)).divide(PARTITIONSCALE.Get(0));
    goalRadius_theta = Sqrt.of(RealScalar.of(2)).divide(PARTITIONSCALE.Get(2));
  }

  @Override
  public int indexOfClosestTrajectorySample(List<TrajectorySample> trajectory) {
    final Tensor x = episodeIntegrator.tail().state();
    return ArgMin.of(Tensor.of(trajectory.stream() //
        .map(TrajectorySample::stateTime) //
        .map(StateTime::state) //
        .map(state -> SE2WRAP.distance(state, x))));
  }

  @Override
  public Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  public Scalar delayHint() {
    return DELAY_HINT;
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    GlobalAssert.that(VectorQ.ofLength(goal, 3));
    // obstacleQuery = EmptyTrajectoryRegionQuery.INSTANCE; // <- for testing
    this.obstacleQuery = obstacleQuery;
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(integrator, RationalScalar.of(1, 10), 4);
    // TwdMinTimeGoalManager twdMinTimeGoalManager = //
    // new TwdMinTimeGoalManager(Tensors.of(goal.Get(0), goal.Get(1), RealScalar.ZERO), goalRadius_xy, RealScalar.of(Math.PI));
    TwdMinCurvatureGoalManager twdMinCurvatureGoalManager = //
        new TwdMinCurvatureGoalManager(goal, goalRadius_xy, goalRadius_theta);
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        PARTITIONSCALE, stateIntegrator, controls, obstacleQuery, twdMinCurvatureGoalManager.getGoalInterface());
    trajectoryPlanner.represent = SE2WRAP::represent;
    return trajectoryPlanner;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      StateTime stateTime = episodeIntegrator.tail();
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
  }
}
