import sys866_lib as ps

instance = ps.SimulinkInstance(
    sim_name='sys866_sim',
    pid_block='sys866_pid',
    init_Kp=1.0,
    init_Ki=0.5,
    init_Kd=0.1,
    dt_sim=0.01
)

instance.set_pid_params(Kp=2.0, Ki=1.0, Kd=0.2)
instance.set_pid_params(Kp=3.0, Ki=1.0, Kd=0.4)
instance.set_pid_params(Kp=1.0, Ki=4.0, Kd=0.8)

instance.close()