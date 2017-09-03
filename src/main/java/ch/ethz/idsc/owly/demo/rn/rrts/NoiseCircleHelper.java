// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import java.util.List;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.demo.rn.RnNodeCollection;
import ch.ethz.idsc.owly.demo.rn.RnTransitionSpace;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.math.sample.CircleRandomSample;
import ch.ethz.idsc.owly.math.sample.RandomSample;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.rrts.adapter.LengthCostFunction;
import ch.ethz.idsc.owly.rrts.adapter.RrtsNodes;
import ch.ethz.idsc.owly.rrts.core.DefaultRrts;
import ch.ethz.idsc.owly.rrts.core.Rrts;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owly.rrts.core.RrtsPlanner;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.red.Norm;

class NoiseCircleHelper {
  ;
  private final StateTime tail;
  RrtsNode root;
  TransitionRegionQuery obstacleQuery;
  List<TrajectorySample> trajectory = null;
  RrtsPlanner rrtsPlanner;
  RnTransitionSpace rnts;

  public NoiseCircleHelper(TransitionRegionQuery obstacleQuery, StateTime tail, Tensor goal) {
    this.tail = tail;
    Tensor orig = tail.state();
    final Tensor diff = goal.subtract(orig);
    Scalar radius = Norm._2.ofVector(diff).multiply(RealScalar.of(.5)).add(RealScalar.ONE);
    final Tensor center = Mean.of(Tensors.of(orig, goal));
    Tensor min = center.map(s -> s.subtract(radius));
    Tensor max = center.map(s -> s.add(radius));
    rnts = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(min, max);
    // obstacleQuery = StaticHelper.noise1();
    this.obstacleQuery = obstacleQuery;
    // ---
    // int iters =
    Rrts rrts = new DefaultRrts(rnts, nc, obstacleQuery, LengthCostFunction.IDENTITY);
    root = rrts.insertAsNode(orig, 5).get();
    RandomSample spaceSampler = new CircleRandomSample(center, radius);
    RandomSample goalSampler = new CircleRandomSample(goal, RealScalar.of(.5));
    rrtsPlanner = new RrtsPlanner(rrts, spaceSampler, goalSampler);
  }

  public void plan(int steps) {
    Expand.steps(rrtsPlanner, steps);
    // System.out.println("found " + rrtsPlanner.getBest().isPresent());
    // System.out.println("iterations =" + iters);
    // System.out.println("rewireCount=" + rrts.rewireCount());
    RrtsNodes.costConsistency(root, rnts, LengthCostFunction.IDENTITY);
    if (rrtsPlanner.getBest().isPresent()) {
      System.out.println("Trajectory to goal region:");
      RrtsNode best = rrtsPlanner.getBest().get();
      List<RrtsNode> sequence = Nodes.listFromRoot(best);
      // TODO magic const
      trajectory = RnFlowTrajectory.createTrajectory(rnts, sequence, tail.time(), RealScalar.of(.1));
    }
  }
}
