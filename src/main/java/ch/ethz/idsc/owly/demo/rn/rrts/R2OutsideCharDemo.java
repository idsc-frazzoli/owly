// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.RnNodeCollection;
import ch.ethz.idsc.owly.demo.rn.RnTransitionSpace;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.sample.BoxRandomSample;
import ch.ethz.idsc.owly.rrts.adapter.LengthCostFunction;
import ch.ethz.idsc.owly.rrts.adapter.RrtsNodes;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.DefaultRrts;
import ch.ethz.idsc.owly.rrts.core.Rrts;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

enum R2OutsideCharDemo {
  ;
  public static void main(String[] args) throws Exception {
    ImageRegion imageRegion = R2ImageRegions.outside_0b36();
    RnTransitionSpace rnss = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(Tensors.vector(0, 0), imageRegion.range());
    TransitionRegionQuery trq = new SampledTransitionRegionQuery( //
        SimpleTrajectoryRegionQuery.timeInvariant(imageRegion), RealScalar.of(.1));
    // ---
    Rrts rrts = new DefaultRrts(rnss, nc, trq, LengthCostFunction.IDENTITY);
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 5).get();
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(60, 477);
    owlyFrame.jFrame.setBounds(100, 100, 550, 550);
    owlyFrame.addBackground(imageRegion);
    BoxRandomSample rnUniformSampler = new BoxRandomSample(Tensors.vector(0, 0), imageRegion.range());
    int frame = 0;
    while (frame++ < 20 && owlyFrame.jFrame.isVisible()) {
      for (int c = 0; c < 50; ++c)
        rrts.insertAsNode(rnUniformSampler.randomSample(), 15);
      owlyFrame.setRrts(root, trq);
      Thread.sleep(10);
    }
    System.out.println(rrts.rewireCount());
    RrtsNodes.costConsistency(root, rnss, LengthCostFunction.IDENTITY);
  }
}
