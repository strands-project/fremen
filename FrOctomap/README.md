FrOctoMap
======

The FrOctoMap [1] method allows for efficient volumetric representation of dynamic three-dimensional environments over long periods of time.
It is based on combination of a well-established 3D mapping framework called Octomaps [2] and an idea to model environment dynamics by its frequency spectrum [3].
The proposed method allows not only for efficient representation, but also reliable prediction of the future states of dynamic three-dimensional environments.
This repository contains the spatio-temporal mapping framework intended for ROS.

1. T.Krajnik, J.M.Santos, B.Seemann, T.Duckett: <b>[FROctomap: An Efficient Spatio-Temporal Environment Representation.](http://labe.felk.cvut.cz/~tkrajnik/papers/fremen_2014_TAROS.pdf)</b> TAROS 2014.
2. A. Hornung,. K.M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard, *OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees* in Autonomous Robots, 2013; DOI: 10.1007/s10514-012-9321-0.
3. T.Krajnik, J.P.Fentanes, G.Cielniak, C.Dondrup, T.Duckett: <b>[Spectral Analysis for Long-Term Robotic Mapping.](http://labe.felk.cvut.cz/~tkrajnik/papers/fremen_2014_ICRA.pdf)</b> ICRA 2014.

How to use the software:

1.  Get a rosbag with octomaps. You can get one here: http://purl.org/robotics/octomaps
2.  Launch the FRoctomap framework: roslaunch fremen froctomap.launch 
3.  Now, you can save and update the fremen model via services '/save_grid' and  '/update_grid'. For parameters, check our paper.
4.  You can also predict or reconstruct the Octomap by providing relevant timestamp (currently a simple sequence number) via service './generate\_octomap'.

This is a demo intended for review purposes of TAROS 2014 to demonstrate its ability to compress observations made over long periods of time.
The main problem of this implementation of the FrOctoMap is that it requires the environment measurements to be performed on a regular basis.
This issue can be solved by using non-uniform Fourier transform, that does not have this requirement, but is more computationally expensive.
