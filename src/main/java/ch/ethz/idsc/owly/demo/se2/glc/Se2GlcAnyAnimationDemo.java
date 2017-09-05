// code by jl
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.se2.any.Se2AnyEntity;
import ch.ethz.idsc.owly.demo.util.ImageRegions;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum Se2GlcAnyAnimationDemo {
  ;
  public static void main(String[] args) throws Exception {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Tensor root = Tensors.vector(2.5, 0.75, 0);
    root = Tensors.vector(6, 6, 1);
    Se2AnyEntity se2AnyEntity = new Se2AnyEntity(root, 12);
    se2AnyEntity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    // Region obstacleRegion = new InvertedRegion(EmptyRegion.INSTANCE);
    ImageRegion obstacleRegion = ImageRegions.loadFromRepository("/io/track0_100.png", Tensors.vector(10, 10), false);
    // Region obstacleRegion = new R2NoiseRegion(0.8);
    obstacleRegion = R2ImageRegions.inside_gtob();
    se2AnyEntity.startLife(obstacleRegion, root); // (trq, root);
    owlyAnimationFrame.set(se2AnyEntity);
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.addBackground(obstacleRegion);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
