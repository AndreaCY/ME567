//   CREATE ROBOT STRUCTURE

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "robot_xxxx";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";  

        
// specify and create data objects for the links of the robot
robot.links = {
    "base": {},  
    "body": {}, 
    "bigarm_left": {} , 
    "bigarm_right": {}, 
    "smallarm_left": {}, 
    "smallarm_right": {},
    "hand_left": {}, 
    "hand_right": {},
    "brain": {}  
};

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.base_body = {parent:"base", child:"body"};
robot.joints.base_body.origin = {xyz: [0.0,0.4,0.0], rpy:[-Math.PI/2,0,0]};
robot.joints.base_body.axis = [0.0,0.0,-1.0]; 

robot.joints.body_bigarm_left = {parent:"body", child:"bigarm_left"};
robot.joints.body_bigarm_left.origin = {xyz: [0.0,0.4,1.8], rpy:[0,0,0]};
robot.joints.body_bigarm_left.axis = [0.0,0.0,0.0]; 

robot.joints.body_bigarm_right = {parent:"body", child:"bigarm_right"};
robot.joints.body_bigarm_right.origin = {xyz: [0.0,-0.4,1.8], rpy:[-Math.PI,0,0]};
robot.joints.body_bigarm_right.axis = [0.0,0.0,0.0]; 

robot.joints.bigarm_smallarm_left = {parent:"bigarm_left", child:"smallarm_left"};
robot.joints.bigarm_smallarm_left.origin = {xyz: [0.0,0.2,-0.8], rpy:[0,-Math.PI/2,0]};
robot.joints.bigarm_smallarm_left.axis = [0.0,0.0,0.0]; 

robot.joints.bigarm_smallarm_right = {parent:"bigarm_right", child:"smallarm_right"};
robot.joints.bigarm_smallarm_right.origin = {xyz: [0.0,0.2,0.8], rpy:[0,-Math.PI/2,0]};
robot.joints.bigarm_smallarm_right.axis = [0.0,0.0,0.0];

robot.joints.smallarm_hand_left = {parent:"smallarm_left", child:"hand_left"};
robot.joints.smallarm_hand_left.origin = {xyz: [-0.15,0.0,0.9], rpy:[Math.PI/2,0,0]};
robot.joints.smallarm_hand_left.axis = [0.0,0.0,0.0];

robot.joints.smallarm_hand_right = {parent:"smallarm_right", child:"hand_right"};
robot.joints.smallarm_hand_right.origin = {xyz: [0.15,0.0,0.9], rpy:[Math.PI/2,0,0]};
robot.joints.smallarm_hand_right.axis = [0.0,0.0,0.0];

robot.joints.body_brain = {parent:"body", child:"brain"};
robot.joints.body_brain.origin = {xyz: [0.0,0.0,2], rpy:[-Math.PI/2,0,0]};
robot.joints.body_brain.axis = [0.0,0.0,0.0];  

robot.endeffector = {};
robot.endeffector.frame = "body_brain";
robot.endeffector.position = [[0],[0],[0.5],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["base"] = new THREE.CubeGeometry( 1, 0.4, 1 );
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0) );

links_geom["body"] = new THREE.CubeGeometry( 0.5, 0.8, 2 );
links_geom["body"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 1) );

links_geom["bigarm_left"] = new THREE.CubeGeometry( 0.4, 0.4, 1 );
links_geom["bigarm_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, -0.3) );

links_geom["bigarm_right"] = new THREE.CubeGeometry( 0.4, 0.4, 1 );
links_geom["bigarm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0.3) );

links_geom["smallarm_left"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["smallarm_left"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.15, 0, 0.4) );

links_geom["smallarm_right"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["smallarm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0.15, 0, 0.4) );

links_geom["hand_left"] = new THREE.CubeGeometry( 0.5, 0.2, 0.5 );
links_geom["hand_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.1, 0) );

links_geom["hand_right"] = new THREE.CubeGeometry( 0.5, 0.2, 0.5 );
links_geom["hand_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.1, 0) );

links_geom["brain"] = new THREE.CubeGeometry( 0.3, 0.4, 0.5 );
links_geom["brain"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.2, 0) );

