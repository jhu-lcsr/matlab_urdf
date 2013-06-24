function t = rne_id(chain, vars)
  % Calculate the joint torques given an arm state
  % USAGE: T = rne_id(CHAIN, VARS)
  %   T: Joint torques
  %   CHAIN: An arm chain structure constructed from load_ne_id()
  %   VARS: An n*3-by-1 vector of joint variables (theta, theta dot, theta dot dot)
  % AUTHOR: Jonthan Bohren

  % Pre-allocate cell arrats
  T = cell(chain.n_links,1);
  R = cell(chain.n_links,1);
  Rt = cell(chain.n_links,1);
  P = cell(chain.n_links,1);

  % End transforms
  T{chain.n_links} = eye(4);
  R{chain.n_links} = eye(3);
  P{chain.n_links} = zeros(3,1);

  W = cell(chain.n_links,1);
  dWdt = cell(chain.n_links,1);

  dVdt = cell(chain.n_links,1);
  dVcdt = cell(chain.n_links,1);

  F = cell(chain.n_links,1);
  N = cell(chain.n_links,1);

  % Use non-symbolic g
  %g = sym('g','real');
  g = 9.81;

  W{1} = [0;0;0];
  dWdt{1} = [0;0;0];
  dVdt{1} = [0;0;g];

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Compute forward kinematics and dynamics equations for each link 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for id = 1:chain.n_links-1
    link = chain.links(id);

    % Get joint
    joint = chain.joints(id+1);

    % Declare symbolic joint variable
    q = vars(3*(id-1)+1);
    
    T{id} = tformRPY(joint.origin.xyz', joint.origin.rpy')*tformAA(joint.axis.xyz',q);
    R{id} = T{id}(1:3,1:3);
    Rt{id} = R{id}';
    P{id} = T{id}(1:3,4);

    disp(id)
    T{id}
  end

  for id = 1:chain.n_links-1
    % Get link and joint
    link = chain.links(id+1);

    % Grab joint variables
    q = vars(3*(id-1)+1);
    dqdt = vars(3*(id-1)+2);
    ddqdt = vars(3*(id-1)+3);

    % Get the angular velocity from the last frame
    w = W{id};
    dwdt = dWdt{id};
    dvdt = dVdt{id};

    % Get joint axis
    a = chain.joints(id+1).axis.xyz';
    % Get COM origin
    com = chain.links(id+1).origin.xyz';

    % Calculate the angular velocity for this link
    W{id+1} = Rt{id}*w+dqdt*a;

    % Calculate the angular acceleration for this link
    dWdt{id+1} = Rt{id}*dwdt+R{id}*cross(w,dqdt*a)+ddqdt*a;

    % Calculate the linear acceleration for this link origin
    dVdt{id+1} = Rt{id}*(cross(dwdt,P{id}) + cross(w,cross(w,P{id})) + dvdt );

    % Calculate the linear acceleration for this link COM
    dVcdt{id+1} = cross(dWdt{id+1},com) + cross(W{id+1}, cross(W{id+1},com)) + dVdt{id+1};

    m = link.mass;
    I = link.inertia;

    % Calculate the force on this link's origin
    F{id+1} = m*dVcdt{id+1};

    % Calculate the moment on this link's origin
    N{id+1} = I*dWdt{id+1} + cross(W{id+1},I*W{id+1});

    
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Generate inverse dynamics equations for each link 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  f = zeros(3,chain.n_links+1);
  n = zeros(3,chain.n_links+1);
  t = zeros(1,chain.n_links+1);

  R{chain.n_links+1} = eye(3);
  P{chain.n_links+1} = [0;0;0];

  for id = chain.n_links:-1:2;
    disp(sprintf('Link %d',id));
    link = chain.links(id);

    % Get the force and torque from the next frame
    fi = f(:,id+1);
    ni = n(:,id+1);


    % Get joint axis
    a = chain.joints(id).axis.xyz';
    % Get COM origin
    com = link.origin.xyz';

    % Calculate forces and moments on this joint
    f(:,id) = R{id}*fi+F{id};
    n(:,id) = N{id} + R{id}*ni + cross(com,F{id}) + cross(P{id},R{id}*fi);

    % Project moment onto the joint axis
    t(id) = n(:,id)'*a;
  end
end
