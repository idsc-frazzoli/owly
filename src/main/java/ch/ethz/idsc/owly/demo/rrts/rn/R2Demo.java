// code by jph
package ch.ethz.idsc.owly.demo.rrts.rn;

import java.util.Random;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.region.PolygonRegion;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.rrts.adapter.DefaultRrts;
import ch.ethz.idsc.owly.rrts.adapter.LengthCostFunction;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.Rrts;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class R2Demo {
  public static void main(String[] args) {
    RnTransitionSpace rnss = new RnTransitionSpace();
    TransitionRegionQuery obstacleQuery = //
        new SampledTransitionRegionQuery(new SimpleTrajectoryRegionQuery( //
            new TimeInvariantRegion( //
                new PolygonRegion(Tensors.matrix(new Number[][] { //
                    { 3, 1 }, //
                    { 4, 1 }, //
                    { 4, 6 }, //
                    { 1, 6 }, //
                    { 1, 3 }, //
                    { 3, 3 }//
                })))), RealScalar.of(.1));
    // ---
    RrtsNodeCollection nc = new RnNodeCollection(Tensors.vector(0, 0), Tensors.vector(10, 10));
    Rrts rrts = new DefaultRrts(rnss, nc, obstacleQuery, new LengthCostFunction());
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 5);
    Random random = new Random();
    for (int c = 0; c < 1000; ++c) {
      Tensor pnt = Tensors.vector( //
          random.nextDouble() * 10, //
          random.nextDouble() * 10);
      rrts.insertAsNode(pnt, 5);
    }
    System.out.println(rrts.rewireCount());
    // System.out.println("trans cost " + RrtsNodes.isCostConsistent(root, rnss, new LengthCostFunction()));
    Gui.rrts(root, obstacleQuery);
  }
}
