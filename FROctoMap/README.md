FrOctoMap
======

The method allows for efficient volumetric representation of dynamic three-dimensional environments over long periods of time.
It's based on combination of a well-established 3D mapping framework called Octomaps [1] and an idea to model environment dynamics by its frequency spectrum [2].
The proposed method allows not only for efficient representation, but also reliable prediction of the future states of dynamic three-dimensional environments.
This repository contains the spatio-temporal mapping framework intended for ROS.
----------------------------------------------
1. A. Hornung,. K.M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard, *OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees* in Autonomous Robots, 2013; DOI: 10.1007/s10514-012-9321-0.
2. T.Krajnik, J.P.Fentanes, G.Cielniak, C.Dondrup, T.Duckett: <b>[Spectral Analysis for Long-Term Robotic Mapping.](http://labe.felk.cvut.cz/~tkrajnik/papers/fremen_2014_ICRA.pdf)</b> ICRA 2014.
-----------------------------------------------

How to use the software:

1.  Get a rosbag with octomaps. You can get one here: http://purl.org/robotics/octomaps
2.  Launch the FRoctomap framework: roslaunch fremen froctomap.launch 
3.  Now, you can save and update the fremen model via services '/save_grid' and  '/update_grid'. For parameters, check our paper.
4.  You can also predict or reconstruct the Octomap by providing relevant timestamp (currently a simple sequence number) via service '/generate_octomap'.

This is just a demo intended for review purposes of TAROS 2014. 
The full version will be released after the paper presentation (if it's accepted). 
