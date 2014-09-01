FreMen
======

Frequency Map Enhancement (FreMEn) is an idea that allows to introduce dynamics into spatial models used in the mobile robotics domain.
Many of these spatial models describe the enviroment by a set of discrete components with binary states.
For example, cells of an occupancy grid are occupied or free, edges of a topological map are traversable or not, doors are opened or closed, rooms are vacant or occupied, landmarks are visible or occluded, etc.
Typically, the state of each model component is uncertain, because it is measured indirectly by means of sensors which are affected by noise.
This is typically represented by means of a probability, that is normally considered a static variable.
We consider probability of each environment state as a function of time and represent it by a combination of harmonic components.

The idea assumes that in populated evironments, many of the observed changes are caused by humans performing their daily activities.
Therefore, the environment's dynamics can be modeled by its frequency spectrum, as a combination of harmonic functions that correspond to periodic processes influencing the environment.
Such a representation not only allows representation of environment dynamics over arbitrary timescales with constant memory requirements, but also prediction of future environment states.
The proposed approach can be applied to many of the state-of-the-art environment models.

Models like occupancy grids, topological or landmark maps can be easily extended by FreMEn.
For reference, see our IROS 2014, TAROS 2014 and ICRA 2014 papers.
