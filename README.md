# BoundleAdjustment No Ceres OR G2O
Implement the BA use levenberg-marquardt method. Different with Ceres, this project use the block struct to make the compute sparse. At the same time, we used the bal data set for optimization testing, the effect is as follows.

## achieve effect

- before optimization
![](./images/before_optimize.png)

- after optimization
![](./images/optimize.png)

## ceres automatic derivation
At the same time, in terms of implementation, we did not use a third-party library, but implemented an automatic derivation method similar to ceres, which is also a good project for learning BA. The core part of the implementation is to overload the operator of the jet structure, so that the two data parts represent the function value and the derivative of the current point respectively, so as to realize automatic derivation.

## Reference
- [http://ceres-solver.org/](http://ceres-solver.org/)
- [https://grail.cs.washington.edu/projects/bal/](https://grail.cs.washington.edu/projects/bal/)
- [https://github.com/starcosmos1225/BoundleAdjustment](https://github.com/starcosmos1225/BoundleAdjustment)
