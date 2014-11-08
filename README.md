FreMEn
======
Frequency Map Enhancement (FreMEn) is a method that allows to introduce dynamics into spatial models used in the mobile robotics domain.
Many of these models describe the environment by a set of discrete components with binary states.
For example, cells of an occupancy grid are occupied or free, edges of a topological map are traversable or not, doors are opened or closed, rooms are vacant or occupied, landmarks are visible or occluded, etc.
Typically, the state of every model component is uncertain, because it is measured indirectly by means of sensors which are affected by noise.
The uncertainty is typically represented by means of a probability, that is normally considered a static variable.
Thus, the probability of a particular component being in a particular state remains constant unless the state is being measured by the robot.

Frequency Map Enhancement considers the probability of each environment state as a function of time and represents it by a combination of harmonic components.
The idea assumes that in populated environments, many of the observed changes are caused by humans performing their daily activities.
Therefore, the environment's dynamics is naturally periodic and can be modelled by its frequency spectrum that represent a combination of harmonic functions that correspond to periodic processes influencing the environment.
Such a model not only allows representation of environment dynamics over arbitrary timescales with constant memory requirements, but also prediction of future environment states.
The proposed approach can be applied to many of the state-of-the-art environment models.

In particular, we have shown that occupancy grids, topological or landmark maps can be easily extended by FreMEn.
- We have shown that the FreMEn allows to represent millions of observations by a few spectral parameters, can reliably predict environment states and detect anomalies [1].
- Applying FreMEn to visibility of visual features improves mobile robot localisation in [changing environments](https://www.youtube.com/watch?v=8AwQrtuNwuA&list=UUJxXV1gKZsmoeoUKE4xo0kA) [2].
- Combining FreMEn with Octomaps results in an efficient spatio-temporal environment model called FrOctoMAp [3] that achieves compression rates up to 1:100000 for timescales over three months.

======
1. T.Krajnik, J.P.Fentanes, G.Cielniak, C.Dondrup, T.Duckett: <b>[Spectral Analysis for Long-Term Robotic Mapping.](http://labe.felk.cvut.cz/~tkrajnik/papers/fremen_2014_ICRA.pdf)</b> ICRA 2014.
2. T.Krajnik, J.P.Fentanes, O.M.Mozos, T.Duckett, J.Ekekrantz, M.Hanheide: <b>[Long-term topological localisation for service robots in dynamic environments using spectral maps.](http://labe.felk.cvut.cz/~tkrajnik/papers/fremen_2014_IROS.pdf)</b> IROS 2014.
3. T.Krajnik, J.M.Santos, B.Seemann, T.Duckett: <b>[FROctomap: An Efficient Spatio-Temporal Environment Representation.](http://labe.felk.cvut.cz/~tkrajnik/papers/fremen_2014_TAROS.pdf)</b> TAROS 2014.

