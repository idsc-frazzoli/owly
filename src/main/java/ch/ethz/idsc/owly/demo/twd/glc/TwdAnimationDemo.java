// code by jph
package ch.ethz.idsc.owly.demo.twd.glc;

import ch.ethz.idsc.owly.demo.twd.TwdEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.tensor.Tensors;

enum TwdAnimationDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(TwdEntity.createDefault(Tensors.vector(0, 0, 0)));
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
