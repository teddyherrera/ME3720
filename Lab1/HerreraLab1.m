% Subscribes to the ros topic fusion/pose_gt to recieve the messages sent
% to the topic
% "DataFormat","struct" takes the msg as a data structure rather than an
% object and improves MATLABs performance when creating, updating and using
% ROS messages
posesub = rossubscriber("fusion/pose_gt","DataFormat","struct")

% recieves data from the subscriber. When a new message is recieved, the
% function will return and store it in the varaible. The second argument is
% timeout in seconds
posedata = receive(posesub)
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
position = posedata.Pose.Pose.Position
posX = posedata.Pose.Pose.Position.X
posY = posedata.Pose.Pose.Position.Y
posZ = posedata.Pose.Pose.Position.Z
posCovar = posedata.Pose.Covariance

orientation = posedata.Pose.Pose.Orientation
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
linearVelocity = posedata.Twist.Twist.Linear
u = posedata.Twist.Twist.Linear.X % surge, linear velocity in x direction
v = posedata.Twist.Twist.Linear.Y % sway, linear velocity in the y direction
w = posedata.Twist.Twist.Linear.Z % heave, linear velocity in the z direction
twistCovae = posedata.Twist.Covariance

angularVelocity = posedata.Twist.Twist.Angular
p = posedata.Twist.Twist.Angular.X % angular velocity in the x direction
q = posedata.Twist.Twist.Angular.Y % angular velocity in the y direction
r = posedata.Twist.Twist.Angular.Z % angular velocity in the z direction 


