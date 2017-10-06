// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.tensor.Tensors;

enum R2RrtsAnimationDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    R2RrtsEntity r2RrtsEntity = new R2RrtsEntity(Tensors.vector(0, 0));
    r2RrtsEntity.obstacleQuery = StaticHelper.noise1();
    owlyAnimationFrame.set(r2RrtsEntity);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
