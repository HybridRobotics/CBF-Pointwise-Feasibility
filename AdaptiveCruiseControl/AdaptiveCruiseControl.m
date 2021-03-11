classdef AdaptiveCruiseControl < handle
    properties
        param = AdaptiveCruiseControlParam()
        method_flag
        x_curr
        u_curr
        dt
        time_curr
        time_log = [];
        x_log = [];
        u_log = [];
        omega_log = [];
        cbf_log = [];
    end
    
    methods
        function self = AdaptiveCruiseControl(x0)
            self.x_curr = x0;
            self.time_curr = 0.0;
        end
        
        function sim(self, time)
            while self.time_curr <= time
                [u_opt, exitflag] = self.clf_cbf_qp();
                if exitflag == -2
                    disp("Stop simulation as optimization infeasible");
                    return
                end
                if exitflag == 0
                    disp("Stop as maximum iteration (almost infeasible)");
                    return
                end
                if exitflag == 999
                    disp("Stop as it's too unsafe");
                    return
                end
                self.u_curr = u_opt;
                self.forward_dynamics();
                self.time_curr = self.time_curr + self.dt;
            end
        end
        
        function [u_opt, exitflag] = clf_cbf_qp(self)
            % decode param
            g = self.param.g;
            m = self. param.m;
            f0 = self.param.f0;
            f1 = self.param.f1;
            f2 = self.param.f2;
            vd = self.param.vd;
            v0 = self.param.v0;
            gamma = self.param.gamma;
            alph = self.param.alph;
            c_a = self.param.c_a;
            c_d = self.param.c_d;
            p = self.param.p;
            
            % soft-decay clf-cbf-qp param
            p_sb = self.param.p_sb;
            omega0 = self.param.omega0;
            
            % decode state
            x = self.x_curr(1);
            v = self.x_curr(2);
            z = self.x_curr(3);
            
            if self.method_flag == false
                % control Lyapunov function
                y = v - vd;
                Fr = f0 + f1*v + f2*v*v;
                
                phi0 = -(2*(v-vd)/m)*Fr + gamma*(v-vd)^2;
                phi1 = 2*(v-vd)/m;
                
                A_clf = [phi1 -1];
                b_clf = -phi0;
                
                % control barrier function
                h = z - 1.8*v;
                LfB = v0 - v + (9*Fr)/(5*m);
                LgB = -9/(5*m);
                
                A_cbf = [-LgB 0];
                b_cbf = LfB + alph*h;
                
                % input constraint
                A_cc = [1 0;
                    -1 0];
                b_cc = [c_a*m*g;
                    c_d*m*g];
                
                % optimization-based controller
                H_acc = 2*[1/(m*m) 0;
                    0 p];
                
                F_acc = -2*[Fr/(m*m); 0];
                
                A = [A_clf; A_cbf; A_cc];
                b = [b_clf; b_cbf; b_cc];
            elseif self.method_flag == true
                % control Lyapunov function
                y = v - vd;
                Fr = f0 + f1*v + f2*v*v;
                
                phi0 = -(2*(v-vd)/m)*Fr + gamma*(v-vd)^2;
                phi1 = 2*(v-vd)/m;
                
                A_clf = [phi1 -1 0];
                b_clf = -phi0;
                
                % control barrier function
                h = z - 1.8*v;
                LfB = v0 - v + (9*Fr)/(5*m);
                LgB = -9/(5*m);
                
                A_cbf = [-LgB 0 -alph*h];
                b_cbf = LfB;
                
                % input constraint
                A_cc = [1 0 0;
                    -1 0 0];
                b_cc = [c_a*m*g;
                    c_d*m*g];
                
                % optimization-based controller
                H_acc = 2*[1/(m*m) 0 0;
                    0 p 0;
                    0 0 p_sb];
                
                F_acc = -2*[Fr/(m*m); 0; omega0*p_sb];
                
                A = [A_clf; A_cbf; A_cc];
                b = [b_clf; b_cbf; b_cc];
            else
                error("Wrong datatype, must declare attribute method_flag as boolean variable");
            end
            options = optimoptions('quadprog','Display','off');
            [u_star,~,exitflag,~] = quadprog(H_acc, F_acc, A, b, [], [], [], [], [], options);
            if exitflag ~= 1 % && exitflag ~= 0
                u_opt = [];
                return;
            end
            u_opt = u_star(1);
            % logging data
            self.time_log = [self.time_log, self.time_curr];
            self.x_log = [self.x_log, self.x_curr];
            self.u_log = [self.u_log, u_opt];
            self.cbf_log = [self.cbf_log, h];
            if self.method_flag == true
                omega = u_star(3);
                self.omega_log = [self.omega_log, omega];
            end
            % jump out if unsafe too much (TODO: remove this hacking return)
            if h < -5
                exitflag = 999;
                return
            end
            
        end
        
        function forward_dynamics(self)
            m = self.param.m;
            f0 = self.param.f0;
            f1 = self.param.f1;
            f2 = self.param.f2;
            v0 = self.param.v0;
            
            x = self.x_curr(1);
            v = self.x_curr(2);
            z = self.x_curr(3);
            
            Fr = f0 + f1*v + f2*v*v;
            f = [v; -(1/m)*Fr; v0 - v];
            g = [0; 1/m; 0];
            
            dx = f + g * self.u_curr;
            self.x_curr = self.x_curr + self.dt * dx;
        end
    end
end