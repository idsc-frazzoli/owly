// code by jph
package ch.ethz.idsc.owly.demo.lv.glc;

import java.util.Collection;

import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.gui.ren.VectorFieldRender;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.VectorFields;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.sample.BoxRandomSample;
import ch.ethz.idsc.owl.math.sample.RandomSample;
import ch.ethz.idsc.owl.math.sample.RandomSampleInterface;
import ch.ethz.idsc.owl.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.lv.LvControls;
import ch.ethz.idsc.owly.demo.lv.LvStateSpaceModel;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class LvAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    StateSpaceModel stateSpaceModel = LvStateSpaceModel.of(1, 2);
    Collection<Flow> controls = LvControls.create(stateSpaceModel, 2);
    owlyAnimationFrame.set(new LvEntity(stateSpaceModel, Tensors.vector(2, 0.3), controls));
    owlyAnimationFrame.setObstacleQuery(EmptyTrajectoryRegionQuery.INSTANCE);
    // ---
    Tensor range = Tensors.vector(6, 5);
    VectorFieldRender vectorFieldRender = new VectorFieldRender();
    RandomSampleInterface sampler = new BoxRandomSample(Tensors.vector(0, 0), range);
    Tensor points = Tensor.of(RandomSample.of(sampler, 1000).stream());
    vectorFieldRender.uv_pairs = //
        VectorFields.of(stateSpaceModel, points, LvEntity.FALLBACK_CONTROL, RealScalar.of(0.04));
    owlyAnimationFrame.addBackground(vectorFieldRender);
    // ---
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new LvAnimationDemo().start();
  }
}
