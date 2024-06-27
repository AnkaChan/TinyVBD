# TinyVBD
A tiny version of VBD, focused on mass spring strand simulation.

This code is a tiny demo of our Siggraph 2024 paper: <br />
[Vertex Block Descent](https://arxiv.org/abs/2403.06321 "The NEXT simulator")

The only dependency is Eigen3. https://eigen.tuxfamily.org/index.php?title=Main_Page

If you compile and run it, it will output the simulation results as a series of JSON files to a subdirectory of where it is run.

You can use the VisualizeStrand.blend to convert those to fbx files for visualization. This is a Blender python scripting file that is used to convert the JSON output of the program into geometry.

The default configuration will run the high mass ratio experiment.
Except for the default configuration, there are other experiment configs that you can try, all you need is to uncomment those options.

