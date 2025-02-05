
When exporting models from Blender, make sure the following options are enabled to ensure the coordinate frames are aligned:

* When exporting chassis and tire OBJ models:
  * Set the forward direction to +X
  * Set the upwards direction to +Z
  * Set the model scale to `0.01`
  * Untick *Write Normals*.
  * Untick *Include UVs* (textures will not be visible but the model will be smaller)
  * Tick *Only OBJ objects*
* When export tire DAE models:
  * Set the forward direction to +Y
  * Set the upwards direction to +Z
  * Tick *Material Groups*.
  * Untick *Write Normals*
  * Untick *Include UVs*  (textures will not be visible but the model will be smaller)

