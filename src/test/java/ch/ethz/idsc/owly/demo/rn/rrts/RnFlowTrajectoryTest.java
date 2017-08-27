// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import java.util.Arrays;
import java.util.List;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.demo.rn.RnNodeCollection;
import ch.ethz.idsc.owly.demo.rn.RnTransitionSpace;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.rrts.adapter.EmptyTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.adapter.LengthCostFunction;
import ch.ethz.idsc.owly.rrts.core.DefaultRrts;
import ch.ethz.idsc.owly.rrts.core.Rrts;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RnFlowTrajectoryTest extends TestCase {
  public void testSimple() {
    RnTransitionSpace rnts = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(Tensors.vector(0, 0), Tensors.vector(10, 10));
    TransitionRegionQuery trq = EmptyTransitionRegionQuery.INSTANCE;
    Rrts rrts = new DefaultRrts(rnts, nc, trq, LengthCostFunction.IDENTITY);
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 0).get();
    assertEquals(root.children().size(), 0);
    RrtsNode n1 = rrts.insertAsNode(Tensors.vector(1.1, 0), 0).get();
    assertEquals(root.children().size(), 1);
    // ---
    List<RrtsNode> sequence = Arrays.asList(root, n1);
    Scalar t0 = RealScalar.ZERO;
    List<TrajectorySample> traj = RnFlowTrajectory.createTrajectory(rnts, sequence, t0, RealScalar.of(.2));
    // Trajectories.print(traj);
  }

  public void testDual() {
    RnTransitionSpace rnts = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(Tensors.vector(0, 0), Tensors.vector(10, 10));
    TransitionRegionQuery trq = EmptyTransitionRegionQuery.INSTANCE;
    Rrts rrts = new DefaultRrts(rnts, nc, trq, LengthCostFunction.IDENTITY);
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 0).get();
    assertEquals(root.children().size(), 0);
    RrtsNode n1 = rrts.insertAsNode(Tensors.vector(1.05, 0), 0).get();
    assertEquals(root.children().size(), 1);
    RrtsNode n2 = rrts.insertAsNode(Tensors.vector(2, 1), 0).get();
    // ---
    List<RrtsNode> sequence = Nodes.listFromRoot(n2);
    assertEquals(sequence, Arrays.asList(root, n1, n2));
    Scalar t0 = RealScalar.ZERO;
    List<TrajectorySample> traj = RnFlowTrajectory.createTrajectory(rnts, sequence, t0, RealScalar.of(.2));
    // Trajectories.print(traj);
  }
}
