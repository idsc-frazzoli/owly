// code by jph
package ch.ethz.idsc.owly.demo.rice.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.rice.Rice2Controls;
import ch.ethz.idsc.owly.demo.rice.Rice2StateSpaceModel;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.VectorFields;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.gui.ren.VectorFieldRender;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.sample.BoxRandomSample;
import ch.ethz.idsc.owly.math.sample.RandomSample;
import ch.ethz.idsc.owly.math.sample.RandomSampleInterface;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class Rice1dNoiseDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Scalar mu = RealScalar.ZERO;
    Collection<Flow> controls = Rice2Controls.create1d(mu, 15);
    owlyAnimationFrame.set(new Rice1dEntity(mu, Tensors.vector(0, 0), controls));
    Region<Tensor> region = new R2NoiseRegion(RealScalar.of(.5));
    owlyAnimationFrame.setObstacleQuery(SimpleTrajectoryRegionQuery.timeInvariant(region));
    // ---
    Tensor range = Tensors.vector(6, 1);
    VectorFieldRender vectorFieldRender = new VectorFieldRender();
    RandomSampleInterface sampler = new BoxRandomSample(range.negate(), range);
    Tensor points = Tensor.of(RandomSample.of(sampler, 1000).stream());
    vectorFieldRender.uv_pairs = //
        VectorFields.of(Rice2StateSpaceModel.of(mu), points, Rice1dEntity.FALLBACK_CONTROL, RealScalar.of(.2));
    owlyAnimationFrame.addBackground(vectorFieldRender);
    // ---
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new Rice1dNoiseDemo().start();
  }
}
