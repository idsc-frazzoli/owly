// code by jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Tensors;

public class Parameters {
//Initial condition
  Tensors x0;
  //Discretization resolution
  int res;
  //State space dimension
  int state_dim;
  //Input space dimension
  int control_dim;
  //Maximum iterations
  int max_iter;
  //Change time coordinate to be appropriate
  double time_scale;
  //Initial partition size
  double partition_scale;
  //Adjust initial depth limit
  int depth_scale;
  //integration step
  double dt_max;
  //scaling of grid
}
