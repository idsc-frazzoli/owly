// code by jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.data.tree.Nodes;

public enum DebugUtils {
  ;
  // ---
  public static final void nodeAmountCompare(TrajectoryPlanner trajectoryPlanner) {
    nodeAmountCompare( //
        Nodes.rootFrom(trajectoryPlanner.getBestOrElsePeek()), trajectoryPlanner.domainMap().size());
  }

  // ---
  public static final void nodeAmountCompare(GlcNode root, int size) {
    if (size != Nodes.ofSubtree(root).size()) {
      System.out.println("****NODE CHECK****");
      System.out.println("Nodes in DomainMap: " + size);
      System.out.println("Nodes in Tree from Root: " + Nodes.ofSubtree(root).size());
      throw new RuntimeException();
    }
  }
}
