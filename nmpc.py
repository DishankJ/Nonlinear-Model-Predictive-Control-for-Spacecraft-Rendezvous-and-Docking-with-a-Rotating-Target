import numpy as np
from casadi import *
import do_mpc

from plotter import Plotter
import config

class MPC:
    def __init__(self):
        self.model = do_mpc.model.Model('discrete')
        self.m = config.m
        self.I_z = config.I_z
        self.Ts = config.Ts
        self.rad_target = config.rad_target
        self.rad_chaser = config.rad_chaser
        self.omega = config.omega
        self.sim_time = config.sim_time
        self.T_horizon = config.T_horizon
        self.R = config.R
        self.rad_hold = config.rad_hold
        self.gamma = config.gamma
        self.alpha = config.alpha
        self.u_min = config.u_min
        self.u_max = config.u_max
        self.x0 = config.x0
        self.obs = config.obs
        self.obs_on = config.obs_on
        self.safety_dist = config.safety_dist

        self.model = self.define_model()
        self.mpc = self.define_mpc()
        self.simulator = self.define_simulator()
        self.estimator = do_mpc.estimator.StateFeedback(self.model)
        self.set_init_state()
    
    def define_model(self):
        model = do_mpc.model.Model('discrete')
        _x = model.set_variable(var_type='_x', var_name='x', shape=(6, 1)) # n_states is 6
        _u = model.set_variable(var_type='_u', var_name='u', shape=(3, 1)) # n_inputs is 3

        A = SX.zeros(6, 6)
        A[0,3] = self.Ts
        A[1,4] = self.Ts
        A[2,5] = self.Ts
        A = SX.eye(6) + A
        B = SX.zeros(6, 3)
        B[3,0] = 1/self.m
        B[4,1] = 1/self.m
        B[5,2] = 1/self.I_z
        x_next = A@_x + self.Ts*B@_u
        model.set_rhs('x', x_next)

        model.set_variable('_tvp', 'x_target')
        model.set_variable('_tvp', 'y_target')
        model.set_variable('_tvp', 'x_target_d')
        model.set_variable('_tvp', 'y_target_d')

        X = SX.zeros(6, 1)
        theta_des = np.arctan2(model.tvp['y_target'] - model.x['x', 1], model.tvp['x_target'] - model.x['x', 0])
        X[0] = model.x['x', 0] - model.tvp['x_target']
        X[1] = model.x['x', 1] - model.tvp['y_target']
        X[2] = model.x['x', 2] - np.arctan2(sin(theta_des - model.x['x', 2]), cos(theta_des - model.x['x', 2]))
        X[3] = model.x['x', 3] - model.tvp['x_target_d']
        X[4] = model.x['x', 4] - model.tvp['y_target_d']
        X[5] = model.x['x', 5] - self.omega

        m_cost = transpose(X)@config.P@X
        l_cost = transpose(X)@config.Q@X

        model.set_expression('m_cost', m_cost)
        model.set_expression('l_cost', l_cost)

        model.setup()
        return model
    
    def define_mpc(self):
        mpc = do_mpc.controller.MPC(self.model)
        
        setup_mpc = {
            'n_horizon': self.T_horizon,
            'n_robust': 0,
            't_step': self.Ts,
            'state_discretization': 'discrete',
            'store_full_solution': True,
        }
        mpc.set_param(**setup_mpc)

        mterm = self.model.aux['m_cost']
        lterm = self.model.aux['l_cost']
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(u=self.R)

        mpc.bounds['lower', '_u', 'u'] = self.u_min
        mpc.bounds['upper', '_u', 'u'] = self.u_max

        tvp_struct_mpc = mpc.get_tvp_template()
        self.i = 0
        def tvp_fun_mpc(t_now):
            # if t_now % 2 == 0:
            if self.i > 2:
                if mpc.data['_x'][int(t_now/self.Ts)-1, 0]**2 + mpc.data['_x'][int(t_now/self.Ts)-1, 1]**2 < self.rad_hold**2:
                    self.rad_hold = self.alpha * self.rad_hold
                    if self.rad_hold < self.rad_target + 0.1:
                        self.rad_hold = self.rad_target + 0.1
            tvp_struct_mpc['_tvp', :, 'x_target'] = -self.rad_hold * np.sin(self.omega * t_now)
            tvp_struct_mpc['_tvp', :, 'x_target_d'] = -self.rad_hold * np.cos(self.omega * t_now)*self.omega
            tvp_struct_mpc['_tvp', :, 'y_target'] = self.rad_hold * np.cos(self.omega * t_now)
            tvp_struct_mpc['_tvp', :, 'y_target_d'] = -self.rad_hold * np.sin(self.omega * t_now)*self.omega
            self.i += 1
            return tvp_struct_mpc
        mpc.set_tvp_fun(tvp_fun_mpc)

        if self.obs_on:
            cbf_constraints = self.get_cbf_constraints()
            for i, cbc in enumerate(cbf_constraints):
                mpc.set_nl_cons('cbf_constraint'+str(i), cbc, ub=0)

        mpc.setup()
        return mpc
    
    def get_cbf_constraints(self):
        A = SX.zeros(6, 6)
        A[0,3] = self.Ts
        A[1,4] = self.Ts
        A[2,5] = self.Ts
        A = SX.eye(6) + A
        B = SX.zeros(6, 3)
        B[3,0] = 1/self.m
        B[4,1] = 1/self.m
        B[5,2] = 1/self.I_z
        x_k1 = A@self.model.x['x'] + self.Ts*B@self.model.u['u']

        cbf_constraints = []
        for obs in self.obs:
            h_k1 = self.h(x_k1, obs)
            h_k = self.h(self.model.x['x'], obs)
            cbf_constraints.append(-h_k1 + (1-self.gamma)*h_k)

        return cbf_constraints


    def h(self, x, obstacle):
        # CBF
        x_obs, y_obs, r_obs = obstacle
        h = (x[0] - x_obs)**2 + (x[1] - y_obs)**2 - (self.rad_chaser + r_obs + self.safety_dist)**2
        return h
    
    def define_simulator(self):
        simulator = do_mpc.simulator.Simulator(self.model)
        simulator.set_param(t_step=self.Ts, abs_tol=1e-4, rel_tol=1e-4)
        tvp_template = simulator.get_tvp_template()

        def tvp_fun(t_now):
            return tvp_template
        simulator.set_tvp_fun(tvp_fun)
        simulator.setup()
        return simulator
    
    def set_init_state(self):
        self.mpc.x0 = self.x0
        self.simulator.x0 = self.x0
        self.estimator.x0 = self.x0
        self.mpc.set_initial_guess()

def main():
    controller = MPC()
    x0 = controller.x0
    for k in range(config.sim_time):
        u0 = controller.mpc.make_step(x0)
        y_next = controller.simulator.make_step(u0)
        x0 = controller.estimator.make_step(y_next)

    plotter = Plotter(controller)
    plotter.plot_results()
    plotter.create_path_animation()

if __name__ == '__main__':
    main()