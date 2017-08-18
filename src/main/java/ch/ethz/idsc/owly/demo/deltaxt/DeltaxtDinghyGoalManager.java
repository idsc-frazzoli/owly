// code by jl
package ch.ethz.idsc.owly.demo.deltaxt;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

public class DeltaxtDinghyGoalManager extends TrajectoryGoalManager implements GoalInterface {
  // TODO JONAS 2 variables not used:
  private final Scalar maxSpeed;
  private final DeltaxtStateSpaceModel stateSpaceModel;
  private final Scalar timeCostScalingFactor;

  public DeltaxtDinghyGoalManager(List<Region> goalRegions, DeltaxtStateSpaceModel stateSpaceModel) {
    this(goalRegions, RealScalar.ONE, stateSpaceModel);
  }

  public DeltaxtDinghyGoalManager(List<Region> goalRegions, Scalar timeCostScalingFactor, DeltaxtStateSpaceModel stateSpaceModel) {
    super(goalRegions);
    this.stateSpaceModel = stateSpaceModel;
    this.maxSpeed = stateSpaceModel.getMaxPossibleChange();
    this.timeCostScalingFactor = timeCostScalingFactor;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    // Costfunction: t
    // return StateTimeTrajectories.timeIncrement(from, trajectory);
    // alternative:
    Scalar sum = Norm._2.of(flow.getU()).add(timeCostScalingFactor);
    // Costfunction: integrate (u^2 +1, t)
    return sum.multiply(StateTimeTrajectories.timeIncrement(from, trajectory));
  }

  /** Ellipsoid with axis: a,b and vector from Center: v = (x,y)
   * specific radius at intersection:
   * r :
   * 
   * a*b * ||v||
   * ---------------
   * sqrt(a²y² + b²x²) */
  @Override
  public Scalar minCostToGoal(Tensor x) {
    // B. Paden: A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
    // p. 79 Eq: 6.4.14
    return RealScalar.ZERO;
  }

  @Override
  public boolean hasHeuristic() {
    return false;
  }
}
