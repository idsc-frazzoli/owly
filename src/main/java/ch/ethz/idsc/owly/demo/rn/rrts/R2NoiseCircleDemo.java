// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

enum R2NoiseCircleDemo {
  ;
  public static void main(String[] args) {
    NoiseCircleHelper nch = new NoiseCircleHelper(StaticHelper.noise1(), //
        new StateTime(Tensors.vector(0, 0), RealScalar.ZERO), Tensors.vector(4, 2));
    nch.plan(400);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(122, 300);
    owlyFrame.jFrame.setBounds(100, 100, 500, 500);
    owlyFrame.setRrts(nch.root, nch.obstacleQuery);
  }
}
