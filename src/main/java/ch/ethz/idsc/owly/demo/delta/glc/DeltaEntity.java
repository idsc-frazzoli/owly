// code by jph
package ch.ethz.idsc.owly.demo.delta.glc;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;

import ch.ethz.idsc.owly.demo.delta.DeltaControls;
import ch.ethz.idsc.owly.demo.delta.DeltaNoHeuristicGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
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
import ch.ethz.idsc.tensor.red.Norm2Squared;

/** class controls delta using {@link StandardTrajectoryPlanner} */
/* package */ class DeltaEntity extends AbstractEntity {
  private static final Tensor FALLBACK_CONTROL = Tensors.vector(0, 0).unmodifiable();
  /** preserve 1[s] of the former trajectory */
  private static final Scalar DELAY_HINT = RealScalar.of(2);
  private static final Scalar maxInput = RealScalar.of(.6);

  public static DeltaEntity createDefault(ImageGradient ipr, Tensor state) {
    EpisodeIntegrator episodeIntegrator = new SimpleEpisodeIntegrator( //
        new DeltaStateSpaceModel(ipr, maxInput), //
        EulerIntegrator.INSTANCE, //
        new StateTime(state, RealScalar.ZERO));
    return new DeltaEntity(episodeIntegrator, ipr, state);
  }

  private final ImageGradient ipr;

  public DeltaEntity(EpisodeIntegrator episodeIntegrator, ImageGradient ipr, Tensor state) {
    super(episodeIntegrator);
    this.ipr = ipr;
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return Norm2Squared.ofVector(x.subtract(y));
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
    Tensor eta = Tensors.vector(5, 5);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 10), 4);
    Collection<Flow> controls = DeltaControls.createControls( //
        new DeltaStateSpaceModel(ipr, maxInput), maxInput, 10);
    DeltaNoHeuristicGoalManager deltaGoalManager = new DeltaNoHeuristicGoalManager( //
        goal.extract(0, 2), Tensors.vector(.3, .3));
    return new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, deltaGoalManager);
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      Tensor state = getStateTimeNow().state();
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(64, 128, 64, 192));
      graphics.fill(new Ellipse2D.Double(point.getX() - 2, point.getY() - 2, 7, 7));
    }
    { // indicate position 1[s] into the future
      Tensor state = getEstimatedLocationAt(DELAY_HINT);
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
  }
}
