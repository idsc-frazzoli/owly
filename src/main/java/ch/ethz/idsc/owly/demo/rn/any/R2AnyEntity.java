// code by jl and jph
package ch.ethz.idsc.owly.demo.rn.any;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.ani.AbstractAnyEntity;
import ch.ethz.idsc.owl.gui.ani.PlannerType;
import ch.ethz.idsc.owl.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owl.math.flow.EulerIntegrator;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2Flows;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm2Squared;

/** omni-directional movement with constant speed */
/* package */ class R2AnyEntity extends AbstractAnyEntity {
  private static final Tensor FALLBACK_CONTROL = Tensors.vector(0, 0).unmodifiable();
  // ---
  private static final R2Flows r2Config = new R2Flows(RealScalar.ONE);
  protected final Collection<Flow> controls = r2Config.getFlows(parameters.getResolutionInt());
  private final Scalar goalRadius = RealScalar.of(0.2);

  /** @param state initial position of entity */
  public R2AnyEntity(StateTime state, int resolution) {
    super(new R2Parameters( //
        RealScalar.of(resolution), // resolution
        RealScalar.of(2), // TimeScale
        RealScalar.of(100), // DepthScale
        Tensors.vector(30, 30), // PartitionScale
        RationalScalar.of(1, 6), // dtMax
        2000, // maxIter
        RealScalar.ONE), // Lipschitz
        // --
        r2Config.getFlows(resolution), //
        // --
        new SimpleEpisodeIntegrator( //
            SingleIntegratorStateSpaceModel.INSTANCE, //
            EulerIntegrator.INSTANCE, //
            state), //
        RealScalar.of(0.5), // lead time
        RealScalar.of(0.3)); // expand Time
  }

  /** implementation of 2 dots following trajectory
   * 
   * @param geometricLayer
   * @param graphics */
  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    { // indicate current position
      Tensor state = getStateTimeNow().state();
      Point2D point = geometricLayer.toPoint2D(state);
      graphics.setColor(new Color(64, 128, 64, 192));
      graphics.fill(new Ellipse2D.Double(point.getX() - 2, point.getY() - 2, 7, 7));
    }
    { // indicate position 1[s] into the future
      Tensor state = getEstimatedLocationAt(delayHint());
      Point2D point = geometricLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.ANY;
  }

  @Override
  public Scalar distance(Tensor x, Tensor y) {
    return Norm2Squared.between(x, y); // non-negative
  }

  @Override
  protected Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  protected final GoalInterface createGoal(Tensor goal) {
    return RnMinDistSphericalGoalManager.create(goal.extract(0, 2), goalRadius);
  }

  @Override
  protected Region<Tensor> createGoalCheckHelp(Tensor goal) {
    Tensor r2Goal = goal.extract(0, 2);
    // (GoalRadius + maximalDistance/step) * Securityfactor
    Scalar goalSearchHelperRadius = (goalRadius.add(RealScalar.ONE)).multiply(RealScalar.of(1.5));
    return new EllipsoidRegion(r2Goal, Tensors.of(goalSearchHelperRadius, goalSearchHelperRadius));
  }

  @Override
  protected TrajectoryRegionQuery updateObstacle(Region<Tensor> environmentRegion, StateTime currentState) {
    return SimpleTrajectoryRegionQuery.timeInvariant( //
        EuclideanDistanceDiscoverRegion.of(environmentRegion, currentState.state(), RealScalar.of(4)));
  }

  @Override
  protected TrajectoryRegionQuery initializeObstacle(Region<Tensor> region, StateTime currentState) {
    return updateObstacle(region, currentState);
  }

  @Override
  protected StateIntegrator createIntegrator() {
    return FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
  }
}
