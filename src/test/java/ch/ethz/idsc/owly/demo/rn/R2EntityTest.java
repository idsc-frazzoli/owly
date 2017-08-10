// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class R2EntityTest extends TestCase {
  public void testSimple() {
    final StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    final Flow ux = StateSpaceModels.createFlow(stateSpaceModel, Tensors.vector(1, 0));
    final List<TrajectorySample> trajectory = new ArrayList<>();
    trajectory.add(TrajectorySample.head(new StateTime(Tensors.vector(0, 0), RealScalar.ZERO)));
    trajectory.add(new TrajectorySample(new StateTime(Tensors.vector(1, 0), RealScalar.ONE), ux));
    trajectory.add(new TrajectorySample(new StateTime(Tensors.vector(2, 0), RealScalar.of(2)), ux));
    // ---
    {
      AbstractEntity abstractEntity = new R2Entity(Tensors.vector(0, 0));
      abstractEntity.setTrajectory(trajectory);
      int index = abstractEntity.indexOfPassedTrajectorySample(trajectory);
      assertEquals(index, 0);
    }
    {
      AbstractEntity abstractEntity = new R2Entity(Tensors.vector(0.5, 0));
      abstractEntity.setTrajectory(trajectory);
      int index = abstractEntity.indexOfPassedTrajectorySample(trajectory);
      assertEquals(index, 0);
    }
    {
      AbstractEntity abstractEntity = new R2Entity(Tensors.vector(0.7, 0));
      abstractEntity.setTrajectory(trajectory);
      int index = abstractEntity.indexOfPassedTrajectorySample(trajectory);
      assertEquals(index, 1);
    }
    {
      AbstractEntity abstractEntity = new R2Entity(Tensors.vector(1.3, 0));
      abstractEntity.setTrajectory(trajectory);
      int index = abstractEntity.indexOfPassedTrajectorySample(trajectory);
      assertEquals(index, 1);
    }
    {
      AbstractEntity abstractEntity = new R2Entity(Tensors.vector(1.7, 0));
      abstractEntity.setTrajectory(trajectory);
      int index = abstractEntity.indexOfPassedTrajectorySample(trajectory);
      assertEquals(index, 2);
    }
    {
      AbstractEntity abstractEntity = new R2Entity(Tensors.vector(1.9, 0));
      abstractEntity.setTrajectory(trajectory);
      int index = abstractEntity.indexOfPassedTrajectorySample(trajectory);
      assertEquals(index, 2);
    }
  }
}
