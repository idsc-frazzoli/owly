// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.Transition;
import ch.ethz.idsc.owly.rrts.core.TransitionSpace;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

// TODO API not finalized, needs interfaces!
public class RnFlowTrajectory {
  public static Flow between(StateTime orig, StateTime dest) {
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE; // Rn
    Tensor direction = dest.state().subtract(orig.state());
    Scalar delta = dest.time().subtract(orig.time());
    return StateSpaceModels.createFlow(stateSpaceModel, direction.divide(delta));
  }

  public static List<TrajectorySample> createTrajectory( //
      TransitionSpace transitionSpace, List<RrtsNode> sequence, Scalar dt) {
    List<TrajectorySample> trajectory = new LinkedList<>();
    Scalar t0 = RealScalar.ZERO;
    Scalar ofs = RealScalar.ZERO; // TODO
    RrtsNode prev = sequence.get(0);
    trajectory.add(TrajectorySample.head(new StateTime(prev.state(), t0)));
    for (RrtsNode node : sequence.subList(1, sequence.size())) {
      System.out.println(node.state());
      Transition transition = transitionSpace.connect(prev.state(), node.state());
      List<StateTime> stateTimes = transition.sampled(t0, ofs, dt);
      // TODO update ofs based on remainder/length
      for (StateTime stateTime : stateTimes) {
        StateTime orig = trajectory.get(trajectory.size() - 1).stateTime();
        Flow flow = between(orig, stateTime);
        trajectory.add(new TrajectorySample(stateTime, flow));
      }
      prev = node;
    }
    return trajectory;
  }
}
