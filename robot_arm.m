robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

L1 = 0.8;
L2 = 0.8;

% Create first link
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

% Create second link
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

% Create end effector
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

%% Define the "21ECE" trajectory
char_height = 0.3;
char_width = 0.2;
spacing = 0.1;
n_points = 51; % Use a number divisible by 3

% Initialize empty points array
points = [];

% Function to add a character's points with offset
function points = add_char_points(points, char_points, x_offset)
    char_points(:,1) = char_points(:,1) + x_offset;
    points = [points; char_points];
end

%% Character '2'
% Define key points for the '2' shape
t = linspace(0, 1, n_points)';

% Top curve (upper part of '2')
theta_top = linspace(pi/4, 3*pi/4, round(n_points/2))';
x_top = char_width * (0.5 + 0.3*cos(theta_top));
y_top = char_height * (0.8 + 0.2*sin(theta_top));

% Bottom curve (lower part of '2')
theta_bottom = linspace(3*pi/4, 5*pi/4, round(n_points/2))';
x_bottom = char_width * (0.5 + 0.3*cos(theta_bottom));
y_bottom = char_height * (0.2 + 0.1*sin(theta_bottom));

% Combine the curves
x2 = [x_top; x_bottom(2:end)];
y2 = [y_top; y_bottom(2:end)];

% Add diagonal connection
x_diag = linspace(x2(end), x2(1), round(n_points/4))';
y_diag = linspace(y2(end), y2(1), round(n_points/4))';
x2 = [x2; x_diag(2:end-1)];
y2 = [y2; y_diag(2:end-1)];

% Ensure we have exactly n_points
if length(x2) > n_points
    x2 = x2(1:n_points);
    y2 = y2(1:n_points);
elseif length(x2) < n_points
    x2 = interp1(linspace(0,1,length(x2)), x2, linspace(0,1,n_points))';
    y2 = interp1(linspace(0,1,length(y2)), y2, linspace(0,1,n_points))';
end

points = add_char_points(points, [x2 y2 zeros(length(x2),1)], 0);

%% Character '1'
x1 = (char_width/2) * ones(n_points,1);
y1 = char_height * linspace(0, 1, n_points)';
points = add_char_points(points, [x1 y1 zeros(n_points,1)], char_width + spacing);

%% Character 'E'
segment_points = round(n_points/3); % Ensure integer number of points
xE1 = zeros(n_points,1);
yE1 = char_height * linspace(0, 1, n_points)';
xE2 = char_width * linspace(0, 1, segment_points)';
yE2 = char_height * ones(segment_points,1);
xE3 = char_width * linspace(0, 1, segment_points)';
yE3 = char_height/2 * ones(segment_points,1);
xE4 = char_width * linspace(0, 0.7, segment_points)';
yE4 = zeros(segment_points,1);

points = add_char_points(points, [xE1 yE1 zeros(n_points,1)], 2*(char_width + spacing));
points = add_char_points(points, [xE2 yE2 zeros(segment_points,1)], 2*(char_width + spacing));
points = add_char_points(points, [xE3 yE3 zeros(segment_points,1)], 2*(char_width + spacing));
points = add_char_points(points, [xE4 yE4 zeros(segment_points,1)], 2*(char_width + spacing));

%% Character 'C'
theta = linspace(pi/2, 3*pi/2, n_points)';
xC = char_width/2 * (1 + cos(theta));
yC = char_height/2 * (1 + sin(theta));
points = add_char_points(points, [xC yC zeros(n_points,1)], 3*(char_width + spacing));

% Character 'E' (again)
points = add_char_points(points, [xE1 yE1 zeros(n_points,1)], 4*(char_width + spacing));
points = add_char_points(points, [xE2 yE2 zeros(segment_points,1)], 4*(char_width + spacing));
points = add_char_points(points, [xE3 yE3 zeros(segment_points,1)], 4*(char_width + spacing));
points = add_char_points(points, [xE4 yE4 zeros(segment_points,1)], 4*(char_width + spacing));

%% Inverse Kinematics
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(size(points, 1), ndof);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0;
for i = 1:size(points, 1)
    point = points(i,1:3);
    qSol = ik(endEffector, trvec2tform(point), weights, qInitial);
    qs(i,:) = qSol;
    qInitial = qSol;
end

%% Visualization
figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1), points(:,2), 'k', 'LineWidth', 2)
axis([-0.1 5*(char_width+spacing) -0.1 char_height+0.1])
title('Robot Tracing "21ECE"')

% Animation
framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:size(qs, 1)
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
