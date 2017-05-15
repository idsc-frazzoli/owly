// code by jph
package ch.ethz.idsc.owly.demo.rrts.rn;

import java.util.Random;

import ch.ethz.idsc.owly.demo.util.ImageRegions;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.rrts.adapter.DefaultRrts;
import ch.ethz.idsc.owly.rrts.adapter.LengthCostFunction;
import ch.ethz.idsc.owly.rrts.adapter.RrtsNodes;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.Rrts;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class R2ImageDemo {
  public static void main(String[] args) throws Exception {
    final int wid = 7;
    RnTransitionSpace rnss = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(Tensors.vector(0, 0), Tensors.vector(wid, wid));
    Region region = ImageRegions.loadFromRepository("/io/track0_100.png", Tensors.vector(wid, wid));
    TransitionRegionQuery trq = //
        new SampledTransitionRegionQuery(new SimpleTrajectoryRegionQuery( //
            new TimeInvariantRegion(region)), RealScalar.of(.1));
    // ---
    Rrts rrts = new DefaultRrts(rnss, nc, trq, LengthCostFunction.IDENTITY);
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 5);
    Random random = new Random();
    for (int c = 0; c < 1000; ++c) {
      Tensor pnt = Tensors.vector( //
          random.nextDouble() * wid, //
          random.nextDouble() * wid);
      rrts.insertAsNode(pnt, 15);
    }
    System.out.println(rrts.rewireCount());
    RrtsNodes.costConsistency(root, rnss, LengthCostFunction.IDENTITY);
    Gui.rrts(root, trq);
  }
}