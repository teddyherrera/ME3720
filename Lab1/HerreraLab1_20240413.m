% Subscribes to the ros topic fusion/pose_gt to recieve the messages sent
% to the topic
% "DataFormat","struct" takes the msg as a data structure rather than an
% object and improves MATLABs performance when creating, updating and using
% ROS messages
posesub = rossubscriber("fusion/pose_gt","DataFormat","struct");

% recieves data from the subscriber. When a new message is recieved, the
% function will return and store it in the varaible. The second argument is
% timeout in seconds
posedata = receive(posesub);
% the structure of the recieved message has two fields Pose and Twist 

% the fields within Pose are layered with msgs that have additional fields relevant to what 
% we are interested in. So have to go through each field and subfield to get the relevant data.
% 1. posedata.Pose the data structure within gives us PoseWithCovariance and 
% has two fields Pose and Covariance
% 2.a. posedata.Pose.Pose produces the messagetype: 'geometry_msg/Pose'
% with fields Position and Orientation
% 2.a.i. posedata.Pose.Pose.Position provides us with Point data for in [X, Y, Z]
% 2.a.ii. posedata.Pose.Pose.Orientation provides us with the Quaternions
% [X, Y, Z, W] 
% 2.b. posedata.Pose.Covariance calls the field Covariance which produces a column vector
position = posedata.Pose.Pose.Position;
posX = posedata.Pose.Pose.Position.X
posY = posedata.Pose.Pose.Position.Y
posZ = posedata.Pose.Pose.Position.Z;
posCovar = posedata.Pose.Covariance

orientation = posedata.Pose.Pose.Orientation;
orientX = posedata.Pose.Pose.Orientation.X
orientY = posedata.Pose.Pose.Orientation.Y
orientZ = posedata.Pose.Pose.Orientation.Z
orientW = posedata.Pose.Pose.Orientation.W

% To investigate the Twist field within posedata use posedata.Twist
% 1. posedata.Twist yieldsa message type
% 'geometry_msgs/TwistWithCovariance' and fields Twist and Covariance
% 2.a. posedata.Twist.Covariance yields a columnn vector 
% 2.b. posedata.Twist.Twist yields a a messagetype 'geometry_msgs/Twist'
% and fields Linear and Angular
% 2.b.i. posedata.Twist.Twist.Linear yields a velocity vector in freespace  with it's with Linear X, Y,
% and Z components
% 2.b.ii. posedata.Twist.Twist.Angular yields an angular velocity vector
% about the X, Y, and Z directions w.r.t the same frame of reference
linearVelocity = posedata.Twist.Twist.Linear;
u = posedata.Twist.Twist.Linear.X % surge, linear velocity in x direction
v = posedata.Twist.Twist.Linear.Y % sway, linear velocity in the y direction
w = posedata.Twist.Twist.Linear.Z % heave, linear velocity in the z direction
twistCovar = posedata.Twist.Covariance;

angularVelocity = posedata.Twist.Twist.Angular;
p = posedata.Twist.Twist.Angular.X % angular velocity in the x direction
q = posedata.Twist.Twist.Angular.Y % angular velocity in the y direction
r = posedata.Twist.Twist.Angular.Z % angular velocity in the z direction 

%% I think these are thruster that Dr. Horner wanted us to send commands to
% /aft_port_thruster                                                      
% /aft_stbd_thruster                                                                                                             
% /bow_port_thruster                                                       
% /bow_stbd_thruster 
% /vert_port_thruster                                                      
% /vert_stbd_thruster 

%% Initialize a publisher variable that will send ROS Float64 messages to 
% the desired ROS topic, in this case it will be the thrusters listed.

aftPortThruster = rospublisher("/aft_port_thruster", "std_msgs/Float64","DataFormat","struct");
aftStbdThruster = rospublisher("/aft_stbd_thruster","std_msgs/Float64","DataFormat","struct");
bowStbdThruster = rospublisher("/bow_stbd_thruster","std_msgs/Float64","DataFormat","struct");
bowPortThruster = rospublisher("/bow_port_thruster","std_msgs/Float64","DataFormat","struct");
vertPortThruster = rospublisher("/vert_port_thruster","std_msgs/Float64","DataFormat","struct");
vertStbdThruster = rospublisher("/vert_stbd_thruster","std_msgs/Float64","DataFormat","struct");
pause(2) % Waits to ensure the publisher the registered

%% General format for the messages sent to each thruster
%  1. create a message variable to send to the desired ROS topic
%  2. populate the message with the desired data to be sent to the ROS topic
%  3. Publish the message to the specified ROS topic. formatted as send(publisher variable, message variable)

% I tested each thruster with positive & negative values, thrusters not
% being tested were set to 0. 
aftPortThrusterMsg = rosmessage(aftPortThruster);
aftPortThrusterMsg.Data = 0 % Positive value = CW rotation about z-axis
send(aftPortThruster, aftPortThrusterMsg) 

aftStbdThrusterMsg = rosmessage(aftStbdThruster);
aftStbdThrusterMsg.Data = 0 % Positive value = CW rotation about z-axis
send(aftStbdThruster, aftStbdThrusterMsg)

bowPortThrusterMsg = rosmessage(bowPortThruster);
bowPortThrusterMsg.Data = 0 % Positive value = CCW rotation about z-axis
send(bowPortThruster, bowPortThrusterMsg)

bowStbdThrusterMsg = rosmessage(bowStbdThruster);
bowStbdThrusterMsg.Data = 0 % positive value = CCW rotation about z-axis
send(bowStbdThruster, bowStbdThrusterMsg)

vertPortThrusterMsg = rosmessage(vertPortThruster);
vertPortThrusterMsg.Data = 0 % positive value = moves down the z-axis (positive heave w.r.t. NED frame)
send(vertPortThruster,vertPortThrusterMsg)

vertStbdThrusterMsg = rosmessage(vertStbdThruster);
vertStbdThrusterMsg.Data = 0 % postive value = move up the z-axis (negative heave w.r.t. NED frame)
send(vertStbdThruster, vertStbdThrusterMsg)


