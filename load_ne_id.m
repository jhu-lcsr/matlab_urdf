function chain = load_ne_id(filename)
  % Load the newton-euler inverse-dynamics from a given URDF
  % USAGE: CHAIN = load_ne_id(FILENAME)
  %   CHAIN: The arm chain structure
  %   FILENAME: The URDF file path
  % AUTHOR: Jonthan Bohren

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Load the URDF xml file
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % This returns a Java XML object
  disp('Reading URDF...');
  xurdf = xmlread(filename);

  % We're going to assume one robot description per URDF file
  xrobots = xurdf.getElementsByTagName('robot');
  if xrobots.getLength() ~= 1
    error('URDF contains more than one robot, but this script assumes a single robot descriptioon per URDF');
  end

  disp('Loading links and joints...');

  % Get links and joints from URDF
  xlinks = xurdf.getElementsByTagName('link');
  xjoints = xurdf.getElementsByTagName('joint');

  % Count links, joints
  n_links = xlinks.getLength;
  n_joints = xjoints.getLength;

  % Create the link and joint maps
  links_map = containers.Map();
  joints_map = containers.Map();

  % Iterate over links
  for k = 0:n_links-1
    disp(sprintf('Loading link %d/%d...',k+1,n_links));
    link = struct();
    link.id = k;
    link.xml = xlinks.item(k);
    link.name = char(link.xml.getAttribute('name'));
    link.parent_joint = '';
    link.child_joint = '';

    xinertial = link.xml.getElementsByTagName('inertial').item(0);

    origin = xinertial.getElementsByTagName('origin').item(0);
    if length(origin)
      link.origin = struct();
      link.origin.xyz = eval(['[',char(origin.getAttribute('xyz')),']']);
      link.origin.rpy = eval(['[',char(origin.getAttribute('rpy')),']']);
    end

    xmass = xinertial.getElementsByTagName('mass').item(0);
    link.mass = eval(char(xmass.getAttribute('value')));

    xinertia = xinertial.getElementsByTagName('inertia').item(0);

    ixx = eval(xinertia.getAttribute('ixx'));
    iyy = eval(xinertia.getAttribute('iyy'));
    izz = eval(xinertia.getAttribute('izz'));
    ixy = eval(xinertia.getAttribute('ixy'));
    iyz = eval(xinertia.getAttribute('iyz'));
    ixz = eval(xinertia.getAttribute('ixz'));

    link.inertia = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];

    % Transform inertia tensor into the origin of the link
    Tcom = tformRPY(link.origin.xyz', link.origin.rpy');
    Rcom = Tcom(1:3,1:3);
    link.inertia = Rcom*link.inertia*(Rcom');

    % Store this link in the links map
    links_map = [links_map; containers.Map({char(link.name)},{link})];
  end

  % Iterate over joints
  for k = 0:n_joints-1
    disp(sprintf('Loading joint %d/%d...',k+1,n_joints));
    joint = struct();
    joint.id = k;
    joint.xml = xjoints.item(k);
    joint.name = char(joint.xml.getAttribute('name'));
    joint.type = char(joint.xml.getAttribute('type'));
    joint.parent = '';
    joint.child = '';
    joint.origin = struct('xyz',[0 0 0],'rpy',[0 0 0]);
    joint.axis = struct('xyz',[0 0 1]);

    % Get joint properties
    origin = joint.xml.getElementsByTagName('origin').item(0);
    if length(origin)
      joint.origin = struct();
      joint.origin.xyz = eval(['[',char(origin.getAttribute('xyz')),']']);
      joint.origin.rpy = eval(['[',char(origin.getAttribute('rpy')),']']);
    end

    axis = joint.xml.getElementsByTagName('axis').item(0);
    if length(axis)
      joint.axis = struct();
      joint.axis.xyz = eval(['[',char(axis.getAttribute('xyz')),']']);
    end

    % Get parent link name
    parent = joint.xml.getElementsByTagName('parent').item(0);
    if length(parent)
      joint.parent = char(parent.getAttribute('link'));

      % Store the joint name in the parent link
      parent = links_map(joint.parent);
      parent.child_joint = joint.name;
      links_map(joint.parent) = parent;
    end

    % Get child link name
    % (assume a chain, so each joint has only one child)
    child = joint.xml.getElementsByTagName('child').item(0);
    if length(child)
      joint.child = char(child.getAttribute('link'));

      % Store the joint name in the child link
      child = links_map(joint.child);
      child.parent_joint = joint.name;
      links_map(joint.child) = child;
    end

    % Store this joint in the joints map
    joints_map = [joints_map; containers.Map({char(joint.name)},{joint})];
  end

  % Find the root link
  disp('Finding root joint...');
  root_link_name = '';

  for link_name = links_map.keys
    if length(links_map(char(link_name)).parent_joint) == 0
      root_link_name = char(link_name);
    end
  end

  if ~length(root_link_name)
    error('Robot has no single root!');
  end

  % Sort the links
  disp('Sorting links...');
  link_name = root_link_name;

  lid = 0;
  while 1
    % Increment link id
    lid = lid + 1;

    link = links_map(link_name);
    link.id = lid;

    links(lid) = link;
    
    if length(link.child_joint) == 0
      break;
    end

    joint = joints_map(link.child_joint);
    joint.id = lid+1;
    joints(lid+1) = joint;

    % Get next link
    link_name = links_map(joint.child).name;
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Store arm data in structure
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  chain = struct();

  chain.links = links;
  chain.n_links = n_links;

  chain.joints = joints;
  chain.n_joints = n_joints;

  chain.varlist = {};
  for k=2:chain.n_links
    chain.varlist = [chain.varlist, {sprintf('q%d',k), sprintf('dq%ddt',k), sprintf('ddq%ddt',k)}];
  end
end
