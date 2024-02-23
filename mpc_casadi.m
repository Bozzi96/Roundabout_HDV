classdef mpc_casadi
    %MPC_CASADI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        f
        args
        solver

        Ts
        % Model
        n
        m
        WMRNumber
        WMR_id
        M % number of WMR

        % state history
        u0
        x0
        X0
        xs
        
        % start MPC
        u_cl
        mpciter
        Hp
        Hc

    end
    
    methods
        function MPCobj = mpc_casadi(Hp,Hc,Ts,x0,Q,R,WMRNumber,WMR_id,rho,dmin,dmax,u0)
            %% Set CasADi
            import casadi.*
        
            MPCobj.Ts = Ts;
            MPCobj.WMRNumber = WMRNumber;
            MPCobj.WMR_id = WMR_id;
            
            % select only the x0 that I need (this won't happen in real
            % implementation)
            if WMRNumber == 1
                x0 = x0(:,1);
            else
                if WMRNumber == 2
                    x0 = x0(:,[1 2]);
                else
                    x0 = x0(:,[1 WMRNumber-1:WMRNumber]);
                end
            end
            MPCobj.M = size(x0,2);
            x0 = reshape(x0,size(x0,1)*size(x0,2),1);

            % Storico
            MPCobj.mpciter = 0;
            if isempty(u0)
                MPCobj.u0 = zeros(Hc,MPCobj.M*1);
            else
                MPCobj.u0 = u0(2)*ones(Hc,MPCobj.M*1); % prendo il secondo perché il primo è l'inizializzazione del ciclo di controllo (design di Casadi)
            end
            MPCobj.X0 = repmat(x0,1,Hp+1)';
            
            % start MPC
            MPCobj.u_cl = MPCobj.u0(1,:);
            MPCobj.x0 = x0;

            MPCobj.Hp = Hp;
            MPCobj.Hc = Hc;

            opts = struct;
            opts.ipopt.max_iter = 100;
            opts.ipopt.print_level = 0;
            opts.print_time = 0;
            opts.ipopt.acceptable_tol = 1e-6;
            opts.ipopt.acceptable_obj_change_tol = 1e-6;

            %% Constraints
%             V_max = 0.3; V_min = 0;
%             a_max = 0.1; a_min = -a_max;
            V_max = 15; V_min = 0;
            a_max = 1.5; a_min = -a_max;
            
            %% Model
            
            if WMRNumber > 2
                X1 = SX.sym('X1'); V1 = SX.sym('V1'); X2 = SX.sym('X2'); V2 = SX.sym('V2'); X3 = SX.sym('X3'); V3 = SX.sym('V3');
                states = [X1;V1;X2;V2;X3;V3]; n = length(states); MPCobj.n = n;
                
                accel1 = SX.sym('accel1'); accel2 = SX.sym('accel2'); accel3 = SX.sym('accel3');
                controls = [accel1;accel2;accel3]; m = length(controls); MPCobj.m = m;
                
                rhs = [X1+Ts*V1+0.5*Ts*Ts*accel1
                       V1+Ts*accel1
                       X2+Ts*V2+0.5*Ts*Ts*accel2
                       V2+Ts*accel2
                       X3+Ts*V3+0.5*Ts*Ts*accel3
                       V3+Ts*accel3];% right hand side of the state equations
                
                MPCobj.f = Function('f',{states, controls}, {rhs}); % nonlinear mapping function
                
                Xmatr = [1 0 -1 0 0 0
                         1 0 0 0 -1 0
                         0 0 1 0 -1 0
                         0 1 0 0 0 0
                         0 0 0 1 0 0
                         0 0 0 0 0 1];
                refnum = size(Xmatr,1);
                U = SX.sym('U', m,Hc);
                P = SX.sym('P',n+refnum); % params (includes initial and reference)
                state_tilde = SX.sym('state_tilde',6);
                P_y = SX.sym('P_y',6);
                
                state = SX.sym('state',n,(Hp+1)); % state matrix over the prediction horizon
                
                obj = 0; % cost function

                g = []; % constraints vector
                gd = []; %inequalities constrants vector (distance)
    
                % compute objective and constraints symbolically
                st = state(:,1);
                g = [g; st-P(1:n)];
                
                for k = 1:Hp
                    st = state(:,k);
                    if k <= Hc
                        con = U(:,k);
                    end

                    obj = obj + (Xmatr*st-P(n+1:n+refnum))'*Q*(Xmatr*st-P(n+1:n+refnum)) + con'*R*con +...
                        rho/2*( (st - state_tilde + (1/rho)*P_y)'*(st - state_tilde + (1/rho)*P_y) );
                    st_next = state(:,k+1);
                    st_next_euler = MPCobj.f(st,con);
                    g = [g; st_next-st_next_euler];
                    gd = [gd; st(1)-st(3); st(1)-st(5); st(3)-st(5)];
                    
                end
                P = [P; state_tilde; P_y];
                g = [g;gd];
                
                % set nonlinear programming structure
                % make the decision variables one column vector
                OPT_variables = [reshape(state,n*(Hp+1),1); reshape(U,m*Hc,1)];
                nlp_prob = struct('f',obj,'x',OPT_variables,'g',g,'p',P);
                
                tempSolver = nlpsol('solver','ipopt',nlp_prob,opts);
                MPCobj.solver = tempSolver;
                
%                 args = struct;
                
                % set equality constraints (continuity constraints)
                args.lbg(1:n*(Hp+1)) = 0;
                args.ubg(1:n*(Hp+1)) = 0;
                
                % set inequalities constraints
                % distance
                args.lbg(n*(Hp+1)+1:3:n*(Hp+1)+3*Hp) = (WMRNumber-2)*dmin;
                args.ubg(n*(Hp+1)+1:3:n*(Hp+1)+3*Hp) = (WMRNumber-2)*dmax;
                args.lbg(n*(Hp+1)+2:3:n*(Hp+1)+3*Hp) = (WMRNumber-1)*dmin;
                args.ubg(n*(Hp+1)+2:3:n*(Hp+1)+3*Hp) = (WMRNumber-1)*dmax;
                args.lbg(n*(Hp+1)+3:3:n*(Hp+1)+3*Hp) = dmin;
                args.ubg(n*(Hp+1)+3:3:n*(Hp+1)+3*Hp) = dmax;
    
                
                % constraints on state
                % (BOX MARGINS CONSTRAINTS)
                args.lbx(1:n:n*(Hp+1),1) = -inf;  %x
                args.ubx(1:n:n*(Hp+1),1) = inf;   %x
                args.lbx(2:n:n*(Hp+1),1) = V_min;  %v
                args.ubx(2:n:n*(Hp+1),1) = V_max;   %v
                args.lbx(3:n:n*(Hp+1),1) = -inf;  %x
                args.ubx(3:n:n*(Hp+1),1) = inf;   %x
                args.lbx(4:n:n*(Hp+1),1) = V_min;  %v
                args.ubx(4:n:n*(Hp+1),1) = V_max;   %v
                args.lbx(5:n:n*(Hp+1),1) = -inf;  %x
                args.ubx(5:n:n*(Hp+1),1) = inf;   %x
                args.lbx(6:n:n*(Hp+1),1) = V_min;  %v
                args.ubx(6:n:n*(Hp+1),1) = V_max;   %v
    
                % control a
                args.lbx(n*(Hp+1)+1:m:n*(Hp+1)+m*Hc) = a_min;
                args.ubx(n*(Hp+1)+1:m:n*(Hp+1)+m*Hc) = a_max;
                args.lbx(n*(Hp+1)+2:m:n*(Hp+1)+m*Hc) = a_min;
                args.ubx(n*(Hp+1)+2:m:n*(Hp+1)+m*Hc) = a_max;
                args.lbx(n*(Hp+1)+3:m:n*(Hp+1)+m*Hc) = a_min;
                args.ubx(n*(Hp+1)+3:m:n*(Hp+1)+m*Hc) = a_max;
            else
                if WMRNumber == 2
                    X1 = SX.sym('X1'); V1 = SX.sym('V1'); X2 = SX.sym('X2'); V2 = SX.sym('V2');
                    states = [X1;V1;X2;V2]; n = length(states); MPCobj.n = n;
                    
                    accel1 = SX.sym('accel1'); accel2 = SX.sym('accel2');
                    controls = [accel1;accel2]; m = length(controls); MPCobj.m = m;
                    
                    rhs = [X1+Ts*V1+0.5*Ts*Ts*accel1
                           V1+Ts*accel1
                           X2+Ts*V2+0.5*Ts*Ts*accel2
                           V2+Ts*accel2];% right hand side of the state equations
    
                    MPCobj.f = Function('f',{states, controls}, {rhs}); % nonlinear mapping function
    
                    Xmatr = [1 0 -1 0
                             0 1 0 0
                             0 0 0 1];
                    refnum = size(Xmatr,1);
                    U = SX.sym('U', m,Hc);
                    P = SX.sym('P',n+refnum); % params (includes initial, reference)
                    state_tilde = SX.sym('state_tilde',4);
                    P_y = SX.sym('P_y',4);
    
                    state = SX.sym('state',n,(Hp+1)); % state matrix over the prediction horizon
                    
                    obj = 0; % cost function
    
                    g = []; % constraints vector
                    gd = []; %inequalities constrants vector (distance)
        
                    % compute objective and constraints symbolically
                    st = state(:,1);
                    g = [g; st-P(1:n)];
    
                    for k = 1:Hp
                        st = state(:,k);
                        if k <= Hc
                            con = U(:,k);
                        end
                    
                        obj = obj + (Xmatr*st-P(n+1:n+refnum))'*Q*(Xmatr*st-P(n+1:n+refnum)) + con'*R*con +...
                              rho/2*( (st-state_tilde + (1/rho)*P_y)' * (st-state_tilde + (1/rho)*P_y) );
        
                        st_next = state(:,k+1);
                        st_next_euler = MPCobj.f(st,con);
                        g = [g; st_next-st_next_euler];
                        gd = [gd; st(1)-st(3)]; 
                    end
                    g = [g;gd];
                    P = [P; state_tilde; P_y];
                    
                    % set nonlinear programming structure
                    % make the decision variables one column vector
                    OPT_variables = [reshape(state,n*(Hp+1),1); reshape(U,2*Hc,1)];
                    nlp_prob = struct('f',obj,'x',OPT_variables,'g',g,'p',P);
    
                    tempSolver = nlpsol('solver','ipopt',nlp_prob,opts);
                    MPCobj.solver = tempSolver;
                    
                    args = struct;
                    
                    % set equality constraints (continuity constraints)
                    args.lbg(1:n*(Hp+1)) = 0;
                    args.ubg(1:n*(Hp+1)) = 0;
                    
                    % set inequalities constraints
                    % distance
                    args.lbg(n*(Hp+1)+1:n*(Hp+1)+Hp) = dmin;
                    args.ubg(n*(Hp+1)+1:n*(Hp+1)+Hp) = dmax;
                   
                    
                    % constraints on state
                    % (BOX MARGINS CONSTRAINTS)
                    args.lbx(1:n:n*(Hp+1),1) = -inf;  %x
                    args.ubx(1:n:n*(Hp+1),1) = inf;   %x
                    args.lbx(2:n:n*(Hp+1),1) = V_min;  %v
                    args.ubx(2:n:n*(Hp+1),1) = V_max;   %v
                    args.lbx(3:n:n*(Hp+1),1) = -inf;  %x
                    args.ubx(3:n:n*(Hp+1),1) = inf;   %x
                    args.lbx(4:n:n*(Hp+1),1) = V_min;  %v
                    args.ubx(4:n:n*(Hp+1),1) = V_max;   %v
        
                    % control a
                    args.lbx(n*(Hp+1)+1:m:n*(Hp+1)+m*Hc) = a_min;
                    args.ubx(n*(Hp+1)+1:m:n*(Hp+1)+m*Hc) = a_max;
                    args.lbx(n*(Hp+1)+2:m:n*(Hp+1)+m*Hc) = a_min;
                    args.ubx(n*(Hp+1)+2:m:n*(Hp+1)+m*Hc) = a_max;
                else    %%%%%% LEADER %%%%%%
                    X1 = SX.sym('X1'); V1 = SX.sym('V1');
                    states = [X1;V1]; n = length(states); MPCobj.n = n;
                    
                    accel1 = SX.sym('accel1');
                    controls = [accel1]; m = length(controls); MPCobj.m = m;
                    
                    rhs = [X1+Ts*V1+0.5*Ts*Ts*accel1
                           V1+Ts*accel1];% right hand side of the state equations
    
                    MPCobj.f = Function('f',{states, controls}, {rhs}); % nonlinear mapping function
    
                    Xmatr = [0 1];
                    refnum = size(Xmatr,1);
                    U = SX.sym('U', m,Hc);
                    P = SX.sym('P',n+refnum); % params (includes initial, reference)
                    state_tilde = SX.sym('state_tilde',2);
                    P_y = SX.sym('P_y',2);
    
                    state = SX.sym('state',n,(Hp+1)); % state matrix over the prediction horizon
                    
                    obj = 0; % cost function
    
                    g = []; % constraints vector
                    gd = []; %inequalities constrants vector (distance)
        
                    % compute objective and constraints symbolically
                    st = state(:,1);
                    g = [g; st-P(1:n)];
    
                    for k = 1:Hp
                        st = state(:,k);
                        if k <= Hc
                            con = U(:,k);
                        end
                    
                        obj = obj + (Xmatr*st-P(n+1:n+refnum))'*Q*(Xmatr*st-P(n+1:n+refnum)) + con'*R*con +...
                              rho/2*( (st-state_tilde + (1/rho)*P_y)' * (st-state_tilde + (1/rho)*P_y) );
        
                        st_next = state(:,k+1);
                        st_next_euler = MPCobj.f(st,con);
                        g = [g; st_next-st_next_euler]; 
                    end
                    P = [P; state_tilde; P_y];
                    
                    % set nonlinear programming structure
                    % make the decision variables one column vector
                    OPT_variables = [reshape(state,n*(Hp+1),1); reshape(U,m*Hc,1)];
                    nlp_prob = struct('f',obj,'x',OPT_variables,'g',g,'p',P);
    
                    tempSolver = nlpsol('solver','ipopt',nlp_prob,opts);
                    MPCobj.solver = tempSolver;
                    
                    args = struct;
                    
                    % set equality constraints (continuity constraints)
                    args.lbg(1:n*(Hp+1)) = 0;
                    args.ubg(1:n*(Hp+1)) = 0;
                   
                    
                    % constraints on state
                    % (BOX MARGINS CONSTRAINTS)
                    args.lbx(1:n:n*(Hp+1),1) = -inf;  %x
                    args.ubx(1:n:n*(Hp+1),1) = inf;   %x
                    args.lbx(2:n:n*(Hp+1),1) = V_min;  %v
                    args.ubx(2:n:n*(Hp+1),1) = V_max;   %v
        
                    % control a
                    args.lbx(n*(Hp+1)+1:m:n*(Hp+1)+m*Hc) = a_min;
                    args.ubx(n*(Hp+1)+1:m:n*(Hp+1)+m*Hc) = a_max;
                end
            end
            MPCobj.args = args;
        end
        
        function MPCobj = step(MPCobj,xs,eta_t,y)
            % eta_t size = (MPConj.M - 1)*n/MPCobj.M
            % w_t size = (MPConj.M - 1)*m/MPCobj.M
            % y size = eta_t + w_t
            MPCobj.args.p = [MPCobj.x0;xs;eta_t;y]; % set values of the paramters vector
            MPCobj.args.x0 = [reshape(MPCobj.X0',MPCobj.n*(MPCobj.Hp+1),1); reshape(MPCobj.u0',MPCobj.m*MPCobj.Hc,1)]; % initial value of the optimization variables
            sol = MPCobj.solver('x0',MPCobj.args.x0,'lbx',MPCobj.args.lbx,'ubx',MPCobj.args.ubx,'lbg',MPCobj.args.lbg,'ubg',MPCobj.args.ubg,'p',MPCobj.args.p); % solve the MPC problem over the prediction horizon
        
            u = reshape(full(sol.x(MPCobj.n*(MPCobj.Hp+1)+1:end))',MPCobj.m,MPCobj.Hc)'; % get controls from the solution
            MPCobj.u_cl = [MPCobj.u_cl; u(1,:)];
            MPCobj = MPCobj.shift(u); % update state and control vector of the model
            MPCobj.X0 = reshape(full(sol.x(1:MPCobj.n*(MPCobj.Hp+1)))',MPCobj.n,MPCobj.Hp+1)'; % get solution trajectory

            % shift trajectory to initialize next step
            MPCobj.X0 = [MPCobj.X0(2:end,:); MPCobj.X0(end,:)];
            MPCobj.mpciter = MPCobj.mpciter + 1;
        end

        function MPCobj = shift(MPCobj,u)
            st = MPCobj.x0;
            con = u(1,:)';
            st = MPCobj.f(st,con);
            MPCobj.x0 = full(st);
        
            MPCobj.u0 = [u(2:size(u,1),:); u(size(u,1),:)]; % duplicate the last entry because it is the best guess we can make about the next solution
        end
    end
end

