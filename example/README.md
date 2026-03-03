# MPC

This repo implements a Model Predictive Controller (MPC) for a quadrotor, using [Acados](https://docs.acados.org/index.html#) library. The MPC is implemented in Python and C++.

1. Generate c_code:
```python
python3 -m mpc.acados_solver -c solver_definition_mpc.yaml
```


