// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import java.util.List;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.demo.rn.RnNodeCollection;
import ch.ethz.idsc.owly.demo.rn.RnTransitionSpace;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.rrts.adapter.LengthCostFunction;
import ch.ethz.idsc.owly.rrts.adapter.RrtsNodes;
import ch.ethz.idsc.owly.rrts.core.DefaultRrts;
import ch.ethz.idsc.owly.rrts.core.Explore;
import ch.ethz.idsc.owly.rrts.core.RandomSampleInterface;
import ch.ethz.idsc.owly.rrts.core.Rrts;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owly.rrts.core.RrtsPlanner;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2NoiseExploreDemo {
  ;
  public static void main(String[] args) {
    Tensor min = Tensors.vector(-1, -3);
    Tensor max = Tensors.vector(-1 + 6, -3 + 6);
    RnTransitionSpace rnts = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(min, max);
    TransitionRegionQuery obstacleQuery = StaticHelper.noise1();
    // ---
    Rrts rrts = new DefaultRrts(rnts, nc, obstacleQuery, LengthCostFunction.IDENTITY);
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 5).get();
    RandomSampleInterface spaceSampler = new CircleRandomSample(Tensors.vector(2, 0), RealScalar.of(3));
    RandomSampleInterface goalSampler = new CircleRandomSample(Tensors.vector(4.5, 0), RealScalar.of(.5));
    RrtsPlanner rrtsPlanner = new RrtsPlanner(rrts, spaceSampler, goalSampler);
    int iters = Explore.constantSteps(rrtsPlanner, 200);
    System.out.println("found " + rrtsPlanner.getBest().isPresent());
    System.out.println("iterations =" + iters);
    System.out.println("rewireCount=" + rrts.rewireCount());
    RrtsNodes.costConsistency(root, rnts, LengthCostFunction.IDENTITY);
    {
      if (rrtsPlanner.getBest().isPresent()) {
        System.out.println("Trajectory to goal region:");
        RrtsNode best = rrtsPlanner.getBest().get();
        List<RrtsNode> sequence = Nodes.listFromRoot(best);
        RnFlowTrajectory.createTrajectory(rnts, sequence, RealScalar.of(.2));
      }
    }
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(122, 226);
    owlyFrame.jFrame.setBounds(100, 100, 500, 500);
    owlyFrame.setRrts(root, obstacleQuery);
  }
}
