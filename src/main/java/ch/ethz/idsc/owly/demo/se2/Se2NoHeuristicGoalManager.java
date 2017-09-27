// code by jph and jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** Se2 goal region is not elliptic, therefore we implement {@link Region}
 * This class exists to compare and show the effects of a Heuristic in the Se2Model
 * bapaden phd thesis: (6.4.10) */
public final class Se2NoHeuristicGoalManager extends Se2AbstractGoalManager {
  /** @param center
   * @param radiusVector with 3 entries the first 2 of which have to be identical */
  public Se2NoHeuristicGoalManager(Tensor center, Tensor radiusVector) {
    super(center, radiusVector);
  }

  @Override // from CostFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory); // integrate(1, t)
  }

  @Override // from CostFunction
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }
}