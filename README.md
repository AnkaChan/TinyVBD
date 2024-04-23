# TinyVBD
A tiny version of VBD, focused on mass spring strand simulation.

This code is a tiny demo of our Siggraph 2024 paper: <br />
[Vertex Block Descent](https://arxiv.org/abs/2403.06321 "The NEXT simulator")

The only dependency is Eigen3.

If you compile and run it, it will output the simulation results as a series of JSON files to C:\Data\Test_20Verts_30degree_withSkip_stiffness1e8.
You can use the VisualizeStrand.blend to convert them to obj files for visualization.

The default configuration will run the high mass ratio experiment.
Except for the default configuration, there are other experiment configs that you can try, all you need is to uncomment those options.