// code by Eric Simonton
// adapted by jph
package ch.ethz.idsc.owly.tree;

import java.util.Arrays;
import java.util.List;

import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class NdTreeMapTest extends TestCase {
  public void testSome() {
    NdTreeMap<String> testTree = new NdTreeMap<String>( //
        Tensors.vector(-2, -3), Tensors.vector(8, 9), //
        10, 10);
    testTree.add(Tensors.vector(1, 1), "d1");
    testTree.add(Tensors.vector(1, 0), "d2");
    testTree.add(Tensors.vector(0, 1), "d3");
    testTree.add(Tensors.vector(1, 1), "d4");
    testTree.add(Tensors.vector(0.1, 0.1), "d5");
    testTree.add(Tensors.vector(6, 7), "d6");
    Distance distancer = Distance.EUCLIDEAN;
    {
      Cluster<String> cluster = testTree.buildCluster(Tensors.vector(0, 0), 1, distancer);
      assertTrue(cluster.iterator().next().value.equals("d5"));
    }
    {
      Cluster<String> cluster = testTree.buildCluster(Tensors.vector(5, 5), 1, distancer);
      assertTrue(cluster.iterator().next().value.equals("d6"));
    }
    {
      Cluster<String> cluster = testTree.buildCluster(Tensors.vector(1.1, 0.9), 2, distancer);
      List<String> list = Arrays.asList("d1", "d4");
      for (Point<String> point : cluster)
        assertTrue(list.contains(point.value));
    }
  }
}
