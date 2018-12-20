
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-
    kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code

     // get endeffector Cartesian position in the world
    endeffector_world = 
    matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);
    
     // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0) 
        +Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0) 
        +Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + 
        " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    var IK_link_chain = []; 
    var joint_push = endeffector_joint;
    var base_flag = false;

    while (!base_flag) {
        IK_link_chain.push(joint_push);
        if (robot.joints[joint_push].parent !== robot.base) {
            joint_push = robot.links[robot.joints[joint_push].parent].parent; 
        }
        else{
            base_flag = true;
        }
    }

    // Jacobian matrices, 6 x N intial size
    J = [[],[],[],[],[],[]];    

    // Parameters needed after
    IK_chain_forward = IK_link_chain.slice();
    IK_chain_forward.reverse();
    endeffector_world = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    target_world = matrix_copy(endeffector_target_world.position); 

    for (var i=0; i<IK_chain_forward.length; i++) {
        joint_origin_world = matrix_multiply(robot.joints[IK_chain_forward[i]].xform, [[0],[0],[0],[1]]);
        joint_axis_local = [[robot.joints[IK_chain_forward[i]].axis[0]],[robot.joints[IK_chain_forward[i]].axis[1]],[robot.joints[IK_chain_forward[i]].axis[2]],[0]];
        joint_axis_world = matrix_multiply(robot.joints[IK_chain_forward[i]].xform, joint_axis_local);
        
        // angular velocity
        J[3][i] = joint_axis_world[0][0]; 
        J[4][i] = joint_axis_world[1][0]; 
        J[5][i] = joint_axis_world[2][0]; 


        vec_origin_endeffector = vector_subtraction(endeffector_world, joint_origin_world);

        J_cross = vector_cross(joint_axis_world, vec_origin_endeffector);
        J[0][i] = J_cross[0]; 
        J[1][i] = J_cross[1]; 
        J[2][i] = J_cross[2];


        if (robot.joints[ IK_chain_forward[i] ].type === 'prismatic') {
            J[0][i] = joint_axis_world[0][0]; 
            J[1][i] = joint_axis_world[1][0]; 
            J[2][i] = joint_axis_world[2][0];
            J[3][i] = 0;
            J[4][i] = 0; 
            J[5][i] = 0; 
        }
        
    }

    xform_matrix = matrix_copy(robot.joints[endeffector_joint].xform);
    euler_angle = angle_from_rotation_matrix(xform_matrix);
    
    var delta_position = vector_subtraction(endeffector_target_world.position, endeffector_world); // column vector
    var delta_orientation = vector_subtraction(endeffector_target_world.orientation, euler_angle); // column vector
    if (kineval.params.ik_orientation_included) {
        dx = [[delta_position[0][0]],[delta_position[1][0]],[delta_position[2][0]],[delta_orientation[0]],[delta_orientation[1]],[delta_orientation[2]]];
    }
    else {
        dx = [[delta_position[0][0]],[delta_position[1][0]],[delta_position[2][0]],[0],[0],[0]];        
    }

    if (kineval.params.ik_pseudoinverse)
        dq = matrix_multiply(matrix_pseudoinverse(J), dx);
    else
        dq = matrix_multiply(matrix_transpose(J), dx);

    for (var i=0; i<IK_chain_forward.length; i++)
        robot.joints[IK_chain_forward[i]].control += kineval.params.ik_steplength * dq[i][0];

}

function angle_from_rotation_matrix(xform) {

    var xform11 = xform[0][0], xform12 = xform[0][1], xform13 = xform[0][2];
    var xform21 = xform[1][0], xform22 = xform[1][1], xform23 = xform[1][2];
    var xform31 = xform[2][0], xform32 = xform[2][1], xform33 = xform[2][2];
    var angle_x, angle_y, angle_z;
    var EPS = 1e-5;

    angle_y = Math.asin(Math.max(-1, Math.min(xform13, 1)));
    if (Math.abs(xform13) < 1-EPS) {
        angle_x = Math.atan2(-xform23, xform33);
        angle_z = Math.atan2(-xform12, xform11);
    }
    else {
        angle_x = Math.atan2(xform32, xform22);
        angle_z = 0;
    }
    return [angle_x, angle_y, angle_z];
}

function vector_subtraction(v1, v2) {

    var is_v1_col = !(typeof v1[0][0] === 'undefined');
    var is_v2_col = !(typeof v2[0][0] === 'undefined');
    var n = v1.length;
    var vector_difference = [];

    if (is_v1_col && is_v2_col) {
        for(var i=0; i<n; i++) {
            vector_difference[i] = [];
            vector_difference[i][0] = v1[i][0] - v2[i][0];
        }        
    }
    if (!is_v1_col && !is_v2_col) {
        for(var i=0; i<n; i++) {
            vector_difference[i] = v1[i] - v2[i];
        }        
    }
    return vector_difference;
}
