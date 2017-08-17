// code by jl and jph
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Collection;

import javax.swing.JLabel;

import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.RnSimpleCircleHeuristicGoalManager;
import ch.ethz.idsc.owly.demo.rn.Se2Parameters;
import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2MinDistGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2r.Se2rGoalManager;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.gui.ani.AbstractAnyEntity;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Sqrt;

/** omni-directional movement with constant speed */
public class SE2AnyEntity extends AbstractAnyEntity {
  private static final JLabel JLABEL = new JLabel();
  private static final Tensor FALLBACK_CONTROL = Array.zeros(2).unmodifiable(); // {angle=0, vel=0}
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { //
          { .2, +.07, 1 }, //
          { .2, -.07, 1 }, //
          { -.1, -.07, 1 }, //
          { -.1, +.07, 1 } //
      }).unmodifiable();
  private static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vector(1, 1, 2));
  // ---
  protected final Collection<Flow> controls = Se2Controls.createControlsForwardAndReverse(RealScalar.ONE, parameters.getResolutionInt());
  private final Scalar goalRadius_xy = Sqrt.of(RealScalar.of(2)).divide(parameters.getEta().Get(0));;
  private final Scalar goalRadius_theta =Sqrt.of(RealScalar.of(2)).divide(parameters.getEta().Get(2));  ;

  /** @param state initial position of entity */
  public SE2AnyEntity(Tensor state) {
    super(state, new Se2Parameters( //
        (RationalScalar) RealScalar.of(10), // resolution
        RealScalar.of(2), // TimeScale
        RealScalar.of(100), // DepthScale
        Tensors.vector(5, 5, 50/Math.PI), // PartitionScale
        RationalScalar.of(1, 6), // dtMax
        2000, // maxIter
        RealScalar.ONE), // Lipschitz
        R2Controls.createRadial(10)); // ;parameters.getResolutionInt())); //TODO: JAN possible to use parameters.?
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.ANY;
  }

  @Override
  public Scalar distance(Tensor x, Tensor y) {
    return SE2WRAP.distance(x, y);
  }

  @Override
  public Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  protected final GoalInterface createGoal(Tensor goal) {
    return new Se2MinDistGoalManager(goal, Tensors.of(goalRadius_xy, goalRadius_xy, goalRadius_theta)))
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
}
