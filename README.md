# Feasibility with Control Barrier Functions
The infeasibility issues between safety constraints and input constraints might occur in safety-critical optimal control using control barrier functions, such CBF-QP and CLF-CBF-QP. We propose optimal-decay control barrier functions constraints that handles this infeasibility. This is the reference implementation of our paper:

### Safety-Critical Control using Optimal-decay Control Barrier Functions with Guaranteed Point-wise Feasibility

[PDF]() | [Code](AdaptiveCruiseControl) | [Figures](AdaptiveCruiseControl/figures)

*Jun Zeng, Bike Zhang, Zhongyu Li, and Koushil Sreenath*

#### Citing
If you find this code useful in your work, please consider citing:
```shell
@article{zeng2021optimal-decay,
  title={Safety-Critical Control using Optimal-decay Control Barrier Functions with Guaranteed Point-wise Feasibility},
  author={Zeng, Jun and Zhang, Bike and Li, Zhongyu and Sreenath, Koushil},
  year={2021}
}
```
### Instructions
We illustrate the problem about feasibility and safety using control barrier functions in this paper using an example of adpative cruise control.

* `testFeasibility.m` shows that infeasibility occurs in nominal CLF-CBF-QP and the proposed optimal-decay CLF-CBF-QP handles this issue.
* `testHyperparameter.m` shows that how hyperparameters influence the performance of safety.
* `testSafety.m` shows that control invariance might no longer be guaranteed if the initial condition is very challenging when the input constraints are introduced, which reveals the only point-wise feasibility (i.e. persistent feasible only for a subset of the safe set).
* `testLieDerivative.m` shows the calculation of lie derivatives for the example of adaptive cruise control.
