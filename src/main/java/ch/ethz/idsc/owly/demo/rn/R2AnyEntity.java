// code by jl and jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.Collection;

import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
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

  /** @param state initial position of entity */
  public R2AnyEntity(Tensor state) {
    super(state, new R2Parameters( //
        (RationalScalar) RealScalar.of(15), // resolution
        RealScalar.of(2), // TimeScale
        RealScalar.of(100), // DepthScale
        Tensors.vector(30, 30), // PartitionScale
        RationalScalar.of(1, 6), // dtMax
        2000, // maxIter
        RealScalar.ONE), // Lipschitz
        R2Controls.createRadial(15)); // ;parameters.getResolutionInt())); //TODO: JAN possible to use parameters.?
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.R2ANY;
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
    return new RnSimpleCircleHeuristicGoalManager(goal.extract(0, 2), DoubleScalar.of(.2));
  }
}
