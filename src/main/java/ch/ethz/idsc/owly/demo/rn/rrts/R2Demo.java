// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.demo.rn.RnNodeCollection;
import ch.ethz.idsc.owly.demo.rn.RnTransitionSpace;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.rrts.adapter.LengthCostFunction;
import ch.ethz.idsc.owly.rrts.adapter.RrtsNodes;
import ch.ethz.idsc.owly.rrts.core.DefaultRrts;
import ch.ethz.idsc.owly.rrts.core.Rrts;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2Demo {
  ;
  public static void main(String[] args) {
    int wid = 7;
    Tensor min = Tensors.vector(0, 0);
    Tensor max = Tensors.vector(wid, wid);
    RnTransitionSpace rnts = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(min, max);
    TransitionRegionQuery trq = StaticHelper.polygon1();
    // ---
    Rrts rrts = new DefaultRrts(rnts, nc, trq, LengthCostFunction.IDENTITY);
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 5).get();
    RnUniformSampler rnUniformSampler = new RnUniformSampler(min, max);
    for (int c = 0; c < 1000; ++c)
      rrts.insertAsNode(rnUniformSampler.nextSample(), 15);
    System.out.println("rewireCount=" + rrts.rewireCount());
    RrtsNodes.costConsistency(root, rnts, LengthCostFunction.IDENTITY);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(42, 456);
    owlyFrame.jFrame.setBounds(100, 100, 500, 500);
    owlyFrame.setRrts(root, trq);
  }
}
