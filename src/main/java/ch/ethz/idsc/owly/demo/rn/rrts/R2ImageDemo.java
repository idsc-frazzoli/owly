// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.demo.rn.RnNodeCollection;
import ch.ethz.idsc.owly.demo.rn.RnTransitionSpace;
import ch.ethz.idsc.owly.demo.util.ImageRegions;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
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
import ch.ethz.idsc.tensor.pdf.Distribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;
import ch.ethz.idsc.tensor.pdf.UniformDistribution;

enum R2ImageDemo {
  ;
  public static void main(String[] args) throws Exception {
    final int wid = 7;
    Tensor MIN = Tensors.vector(0, 0);
    Tensor MAX = Tensors.vector(wid, wid);
    RnTransitionSpace rnss = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(MIN, MAX);
    Region region = ImageRegions.loadFromRepository("/io/track0_100.png", Tensors.vector(wid, wid), false);
    TransitionRegionQuery trq = //
        new SampledTransitionRegionQuery(new SimpleTrajectoryRegionQuery( //
            new TimeInvariantRegion(region)), RealScalar.of(.1));
    // ---
    Rrts rrts = new DefaultRrts(rnss, nc, trq, LengthCostFunction.IDENTITY);
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 5);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(60, 477);
    owlyFrame.jFrame.setBounds(100, 100, 550, 550);
    Distribution distributionX = UniformDistribution.of(MIN.Get(0), MAX.Get(0));
    Distribution distributionY = UniformDistribution.of(MIN.Get(1), MAX.Get(1));
    int frame = 0;
    while (frame++ < 20 && owlyFrame.jFrame.isVisible()) {
      for (int c = 0; c < 50; ++c) {
        Tensor pnt = Tensors.of( //
            RandomVariate.of(distributionX), //
            RandomVariate.of(distributionY));
        rrts.insertAsNode(pnt, 15);
      }
      owlyFrame.setRrts(root, trq);
      Thread.sleep(200);
    }
    System.out.println(rrts.rewireCount());
    RrtsNodes.costConsistency(root, rnss, LengthCostFunction.IDENTITY);
  }
}
