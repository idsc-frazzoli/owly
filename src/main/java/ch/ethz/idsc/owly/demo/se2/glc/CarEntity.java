// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.util.Collection;
import java.util.Optional;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.glc.adapter.MultiConstraintAdapter;
import ch.ethz.idsc.owl.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.std.PlannerConstraint;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.std.TrajectoryObstacleConstraint;
import ch.ethz.idsc.owl.gui.ani.PlannerType;
import ch.ethz.idsc.owl.mapping.MappingInterface;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.StateTimeTensorFunction;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.se2.CarFlows;
import ch.ethz.idsc.owly.demo.se2.CarStandardFlows;
import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2ShiftCostFunction;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.sca.Sqrt;

/** several magic constants are hard-coded in the implementation.
 * that means, the functionality does not apply to all examples universally. */
public class CarEntity extends Se2Entity {
  private static final Tensor PARTITIONSCALE = Tensors.vector(5, 5, 50 / Math.PI).unmodifiable(); // 50/pi == 15.9155
  private static final Scalar SQRT2 = Sqrt.of(RealScalar.of(2));
  private static final Scalar SHIFT_PENALTY = RealScalar.of(0.4);
  /** the time difference between two successive nodes in the planner tree
   * is 4/10 */
  // TODO use 10/4 in last entry of partition scale
  public static final FixedStateIntegrator FIXEDSTATEINTEGRATOR = //
      FixedStateIntegrator.create(Se2CarIntegrator.INSTANCE, RationalScalar.of(1, 10), 4);
  // ---
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { //
          { .2, +.07 }, //
          { .25, +.0 }, //
          { .2, -.07 }, //
          { -.1, -.07 }, //
          { -.1, +.07 } //
      }).unmodifiable();
  // ---
  static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vector(1, 1, 1));

  public static CarEntity createDefault(StateTime stateTime) {
    return new CarEntity(stateTime);
  }

  // ---
  private final Collection<Flow> controls;
  private final Tensor goalRadius;
  // ---
  private Optional<MappingInterface> mapping = Optional.empty();

  /** extra cost functions, for instance
   * 1) to penalize switching gears
   * 2) to prevent cutting corners
   * 
   * @param stateTime initial position */
  CarEntity(StateTime stateTime) {
    super(new SimpleEpisodeIntegrator(Se2StateSpaceModel.INSTANCE, Se2CarIntegrator.INSTANCE, stateTime));
    CarFlows carFlows = new CarStandardFlows(RealScalar.ONE, Degree.of(45));
    controls = carFlows.getFlows(6);
    final Scalar goalRadius_xy = SQRT2.divide(PARTITIONSCALE.Get(0));
    final Scalar goalRadius_theta = SQRT2.divide(PARTITIONSCALE.Get(2));
    goalRadius = Tensors.of(goalRadius_xy, goalRadius_xy, goalRadius_theta);
    extraCosts.add(new Se2ShiftCostFunction(SHIFT_PENALTY));
  }

  CarEntity(StateTime stateTime, CarFlows carFlows) {
    super(new SimpleEpisodeIntegrator(Se2StateSpaceModel.INSTANCE, Se2CarIntegrator.INSTANCE, stateTime));
    controls = carFlows.getFlows(6);
    final Scalar goalRadius_xy = SQRT2.divide(PARTITIONSCALE.Get(0));
    final Scalar goalRadius_theta = SQRT2.divide(PARTITIONSCALE.Get(2));
    goalRadius = Tensors.of(goalRadius_xy, goalRadius_xy, goalRadius_theta);
    extraCosts.add(new Se2ShiftCostFunction(SHIFT_PENALTY));
  }

  @Override
  public Scalar distance(Tensor x, Tensor y) {
    return SE2WRAP.distance(x, y); // non-negative
  }

  @Override
  public Scalar delayHint() {
    return RealScalar.of(1.5);
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.STANDARD;
  }

  public void setMapping(MappingInterface mapping) {
    this.mapping = Optional.of(mapping);
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    GlobalAssert.that(VectorQ.ofLength(goal, 3));
    if (mapping.isPresent())
      mapping.get().prepareForQuery();
    this.obstacleQuery = obstacleQuery;
    GoalInterface goalInterface = MultiCostGoalAdapter.of( //
        Se2MinTimeGoalManager.create(goal, goalRadius, controls), extraCosts);
    extraConstraints.add(new TrajectoryObstacleConstraint(obstacleQuery));
    PlannerConstraint plannerConstraint = MultiConstraintAdapter.of(extraConstraints);
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta(), FIXEDSTATEINTEGRATOR, controls, plannerConstraint, goalInterface);
    trajectoryPlanner.represent = StateTimeTensorFunction.state(SE2WRAP::represent);
    StandardTrajectoryPlanner stp = (StandardTrajectoryPlanner) trajectoryPlanner;
    return trajectoryPlanner;
  }

  @Override
  protected Tensor eta() {
    return PARTITIONSCALE;
  }

  @Override
  protected Tensor shape() {
    return SHAPE;
  }
}
