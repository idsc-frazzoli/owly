// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;

import ch.ethz.idsc.owly.demo.twd.TwdControls;
import ch.ethz.idsc.owly.demo.twd.TwdMinTimeGoalManager;
import ch.ethz.idsc.owly.demo.twd.TwdStateSpaceModel;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.OwlyLayer;
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
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.ArgMin;

public class TwdEntity extends AbstractEntity {
  private static final Tensor FALLBACK_CONTROL = Tensors.vector(0, 0).unmodifiable();
  private static final Scalar DELAY_HINT = RealScalar.ONE;
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { { .3, 0, 1 }, { -.1, -.1, 1 }, { -.1, +.1, 1 } }).unmodifiable();

  public static TwdEntity create(TwdStateSpaceModel twdStateSpaceModel) {
    return new TwdEntity(twdStateSpaceModel, MidpointIntegrator.INSTANCE);
  }

  public static TwdEntity createDefault() {
    Scalar wheelRadius = RationalScalar.of(100, 100); // 1[m] -> results in speed 1[m/s]
    Scalar wheelDistance = RationalScalar.of(40, 10); // 40[cm]
    TwdStateSpaceModel twdStateSpaceModel = new TwdStateSpaceModel(wheelRadius, wheelDistance);
    return create(twdStateSpaceModel);
  }

  // ---
  final TwdStateSpaceModel twdStateSpaceModel;
  final Integrator integrator;
  final Collection<Flow> controls;

  public TwdEntity(TwdStateSpaceModel twdStateSpaceModel, Integrator integrator) {
    super(new SimpleEpisodeIntegrator( //
        twdStateSpaceModel, //
        integrator, //
        new StateTime(Tensors.vector(0, 0, 0), RealScalar.ZERO)));
    this.twdStateSpaceModel = twdStateSpaceModel;
    this.integrator = integrator;
    controls = TwdControls.createControls(twdStateSpaceModel, 4);
  }

  @Override
  int indexOfClosestTrajectorySample() {
    final Tensor x = episodeIntegrator.tail().state();
    return ArgMin.of(Tensor.of(trajectory.stream() //
        .map(TrajectorySample::stateTime) //
        .map(StateTime::state) //
        .map(state -> errorCombined(state, x))));
  }

  private static Scalar errorCombined(Tensor state1, Tensor state2) {
    Scalar ep = TwdStateSpaceModel.errorPosition(state1, state2);
    Scalar er = TwdStateSpaceModel.errorRotation(state1, state2);
    return ep.add(er);
  }

  @Override
  Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  Scalar delayHint() {
    return DELAY_HINT;
  }

  @Override
  TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    Tensor partitionScale = Tensors.vector(6, 6, 20);
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(integrator, RationalScalar.of(1, 10), 4);
    TwdMinTimeGoalManager twdMinTimeGoalManager = //
        new TwdMinTimeGoalManager(Tensors.of(goal.Get(0), goal.Get(1), RealScalar.ZERO), RealScalar.of(.5), RealScalar.of(Math.PI));
    // obstacleQuery = EmptyTrajectoryRegionQuery.INSTANCE; // TODO temporary
    return new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, twdMinTimeGoalManager.getGoalInterface());
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      Tensor state = episodeIntegrator.tail().state();
      Tensor matrix = Se2Utils.toSE2Matrix(state);
      Tensor polygon = Tensor.of(SHAPE.flatten(0).map(matrix::dot));
      {
        Path2D path2d = owlyLayer.toPath2D(polygon);
        graphics.setColor(new Color(64, 64, 64, 128));
        graphics.fill(path2d);
      }
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(128 - 64, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
    { // indicate position 1[s] into the future
      Tensor state = getEstimatedLocationAt(DELAY_HINT);
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
  }
}
