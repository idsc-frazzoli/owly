// code by jl and jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractAnyEntity;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;

/** omni-directional movement with constant speed */
public class R2AnyEntity extends AbstractAnyEntity {
  private static final Tensor FALLBACK_CONTROL = Tensors.vector(0, 0).unmodifiable();
  // ---
  protected final Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
  private final Scalar goalRadius = RealScalar.of(0.2);

  /** @param state initial position of entity */
  public R2AnyEntity(Tensor state, int resolution) {
    super(state, new R2Parameters( //
        (RationalScalar) RealScalar.of(resolution), // resolution
        RealScalar.of(2), // TimeScale
        RealScalar.of(100), // DepthScale
        Tensors.vector(30, 30), // PartitionScale
        RationalScalar.of(1, 6), // dtMax
        2000, // maxIter
        RealScalar.ONE), // Lipschitz
        R2Controls.createRadial(resolution), //
        new SimpleEpisodeIntegrator( //
            SingleIntegratorStateSpaceModel.INSTANCE, //
            EulerIntegrator.INSTANCE, //
            new StateTime(state, RealScalar.ZERO)));
  }

  /** implementation of 2 dots following trajectory
   * 
   * @param owlyLayer
   * @param graphics */
  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      Tensor state = getStateTimeNow().state();
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(64, 128, 64, 192));
      graphics.fill(new Ellipse2D.Double(point.getX() - 2, point.getY() - 2, 7, 7));
    }
    { // indicate position 1[s] into the future
      Tensor state = getEstimatedLocationAt(delayHint());
      Point2D point = owlyLayer.toPoint2D(state);
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
    return Norm._2SQUARED.of(x.subtract(y));
  }

  @Override
  public Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  protected final GoalInterface createGoal(Tensor goal) {
    return new RnSimpleCircleHeuristicGoalManager(goal.extract(0, 2), goalRadius);
  }

  @Override
  protected Region createGoalCheckHelp(Tensor goal) {
    Tensor r2Goal = goal.extract(0, 2);
    // (GoalRadius + maximalDistance/step) * Securityfactor
    Scalar goalSearchHelperRadius = (goalRadius.add(RealScalar.ONE)).multiply(RealScalar.of(1.5));
    return new EllipsoidRegion(r2Goal, Tensors.of(goalSearchHelperRadius, goalSearchHelperRadius));
  }

  @Override
  protected TrajectoryRegionQuery updateObstacle(Region environmentRegion, Tensor currentState) {
    return new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(//
        EuclideanDistanceDiscoverRegion.of(environmentRegion, currentState, RealScalar.of(4))));
  }

  @Override
  protected TrajectoryRegionQuery initializeObstacle(Region region, Tensor currentState) {
    return updateObstacle(region, currentState);
  }

  @Override
  protected StateIntegrator createIntegrator() {
    return FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
  }
}
