
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();

}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:

kineval.buildFKTransforms = function buildFKTransforms () {
    traverseFKBase();
    var a;
    for (a = 0; a < robot.links[robot.base].children.length; a++){
        traverseFKJoint(robot.links[robot.base].children[a]);
    }
}

function traverseFKBase() {
    var A = matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(robot.origin.rpy[2]),generate_rotation_matrix_Y(robot.origin.rpy[1])),generate_rotation_matrix_X(robot.origin.rpy[0]));
    var B = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
    var l_lateral_X = [[1],[0],[0],[1]];
    var l_head_Z = [[0],[0],[1],[1]];   

    robot.links[robot.base].xform = matrix_multiply(B,A);
    robot_lateral = matrix_multiply(robot.links[robot.base].xform,l_lateral_X);
    robot_heading = matrix_multiply(robot.links[robot.base].xform,l_head_Z);

    if (robot.links_geom_imported) {
        var mis_xform = matrix_multiply(generate_rotation_matrix_Y((-0.5)*Math.PI),generate_rotation_matrix_X((-0.5)*Math.PI));
        robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, mis_xform);
    }
}

function traverseFKLink(l) {
    var a;
    robot.links[l].xform = robot.joints[robot.links[l].parent].xform;
    if (typeof robot.links[l].children === 'undefined'){
        return;
    }

    for (a = 0; a < robot.links[l].children.length; a++){
        traverseFKJoint(robot.links[l].children[a]);
    }
}

function traverseFKJoint(j) {
    var A = matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(robot.joints[j].origin.rpy[2]),generate_rotation_matrix_Y(robot.joints[j].origin.rpy[1])),generate_rotation_matrix_X(robot.joints[j].origin.rpy[0]));
    var B = generate_translation_matrix(robot.joints[j].origin.xyz[0], robot.joints[j].origin.xyz[1], robot.joints[j].origin.xyz[2]);
    var A_q;
    if (robot.links_geom_imported){
        if (robot.joints[j].type === "revolute" || robot.joints[j].type === "continuous"){
            A_q = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[j].angle,robot.joints[j].axis)));
        }
        else if (robot.joints[j].type === "prismatic"){
            A_q = generate_translation_matrix(robot.joints[j].angle*robot.joints[j].axis[0],robot.joints[j].angle*robot.joints[j].axis[1],robot.joints[j].angle*robot.joints[j].axis[2]);
        }
        else{A_q = generate_identity(4);}
    }
    else{
        A_q = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[j].angle,robot.joints[j].axis)));
    }
    robot.joints[j].xform = matrix_multiply(matrix_multiply(robot.links[robot.joints[j].parent].xform,matrix_multiply(B,A)), A_q);
    traverseFKLink(robot.joints[j].child);
}
