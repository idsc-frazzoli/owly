// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.demo.rn.RnNodeCollection;
import ch.ethz.idsc.owly.demo.rn.RnTransitionSpace;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.region.PolygonRegion;
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
import ch.ethz.idsc.tensor.io.AnimationWriter;
import ch.ethz.idsc.tensor.pdf.Distribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;
import ch.ethz.idsc.tensor.pdf.UniformDistribution;

enum R2ExpandDemo {
  ;
  public static void main(String[] args) throws Exception {
    int wid = 7;
    Tensor MIN = Tensors.vector(0, 0);
    Tensor MAX = Tensors.vector(wid, wid);
    RnTransitionSpace rnss = new RnTransitionSpace();
    RrtsNodeCollection nc = new RnNodeCollection(MIN, MAX);
    TransitionRegionQuery trq = //
        new SampledTransitionRegionQuery(new SimpleTrajectoryRegionQuery( //
            new TimeInvariantRegion( //
                new PolygonRegion(Tensors.matrix(new Number[][] { //
                    { 3, 1 }, //
                    { 4, 1 }, //
                    { 4, 6 }, //
                    { 1, 6 }, //
                    { 1, 3 }, //
                    { 3, 3 } //
                })))), RealScalar.of(.1));
    // ---
    Rrts rrts = new DefaultRrts(rnss, nc, trq, LengthCostFunction.IDENTITY);
    RrtsNode root = rrts.insertAsNode(Tensors.vector(0, 0), 5);
    Distribution distributionX = UniformDistribution.of(MIN.Get(0), MAX.Get(0));
    Distribution distributionY = UniformDistribution.of(MIN.Get(1), MAX.Get(1));
    AnimationWriter gsw = AnimationWriter.of(UserHome.Pictures("r2rrts.gif"), 250);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(42, 456);
    owlyFrame.jFrame.setBounds(100, 100, 500, 500);
    int frame = 0;
    while (frame++ < 40 && owlyFrame.jFrame.isVisible()) {
      for (int c = 0; c < 10; ++c) {
        Tensor pnt = Tensors.of( //
            RandomVariate.of(distributionX), //
            RandomVariate.of(distributionY));
        rrts.insertAsNode(pnt, 20);
      }
      owlyFrame.setRrts(root, trq);
      gsw.append(owlyFrame.offscreen());
      Thread.sleep(100);
    }
    int repeatLast = 3;
    while (0 < repeatLast--)
      gsw.append(owlyFrame.offscreen());
    gsw.close();
    System.out.println(rrts.rewireCount());
    RrtsNodes.costConsistency(root, rnss, LengthCostFunction.IDENTITY);
  }
}
