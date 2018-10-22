
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
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
        + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
        + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }

}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    // Generate the dx in world system
    var matrix_transf = robot.joints[endeffector_joint].xform;
    var endeffector_world_xyz = matrix_multiply(matrix_transf, endeffector_position_local);
    var endeffector_world_rpy = [[Math.atan2(matrix_transf[2][1], matrix_transf[2][2])], [Math.atan2(-matrix_transf[2][0], Math.sqrt(Math.pow(matrix_transf[2][1], 2)+Math.pow(matrix_transf[2][2], 2)))], [Math.atan2(matrix_transf[1][0], matrix_transf[0][0])]];
    var endeffector_world = [[endeffector_world_xyz[0][0]], [endeffector_world_xyz[1][0]], [endeffector_world_xyz[2][0]], [endeffector_world_rpy[0][0]], [endeffector_world_rpy[1][0]], [endeffector_world_rpy[2][0]]];
    var i;
    var dx = [];
    for (i = 0; i< 6; i++){
        dx[i] = [];
        if (i < 3){
            dx[i][0] = endeffector_target_world.position[i] - endeffector_world[i][0];
        }
        else{
            if (kineval.params.ik_orientation_included){
                dx[i][0] = endeffector_target_world.orientation[i-3] - endeffector_world[i][0];
            }
            else{
                dx[i][0] = 0;
            }
        }
    }

    // Generate the jacobian matrix
    var jacobian_matrix = [];
    var count_joint = 0;
    var flag_jac = true;
    var chain_joint_name = endeffector_joint;
    // var endeffector_origin_world = matrix_multiply(matrix_transf, [endeffector_position_local[0][0],endeffector_position_local[1][0],endeffector_position_local[2][0],[1]]);
    while (flag_jac){
        if (robot.joints[chain_joint_name].parent === robot.base){
            flag_jac = false;
        }
        var matrix_tran_joint = [];
        var axis_world_joint = [];
        for (i = 0; i< 3; i++){
            matrix_tran_joint[i] = [];
            for (j = 0; j< 3; j++){
                matrix_tran_joint[i][j] = robot.joints[chain_joint_name].xform[i][j];
            }
        }
        axis_world_joint = matrix_multiply(matrix_tran_joint, [[robot.joints[chain_joint_name].axis[0]],[robot.joints[chain_joint_name].axis[1]], [robot.joints[chain_joint_name].axis[2]]]);
        axis_world_joint = [axis_world_joint[0][0], axis_world_joint[1][0], axis_world_joint[2][0]];
        axis_world_joint = vector_normalize(axis_world_joint);
        if (robot.joints[chain_joint_name].type === "prismatic"){
            jacobian_matrix[count_joint] = [axis_world_joint[0], axis_world_joint[1], axis_world_joint[2], 0, 0, 0];
        }
        else{
            var origin_world_joint = matrix_multiply(robot.joints[chain_joint_name].xform, [[0],[0],[0],[1]]);
            var origin_difference_joint = [endeffector_world_xyz[0][0] - origin_world_joint[0][0], endeffector_world_xyz[1][0] - origin_world_joint[1][0], endeffector_world_xyz[2][0] - origin_world_joint[2][0]];
            var cross_axis_origindif = vector_cross(axis_world_joint, origin_difference_joint);
            jacobian_matrix[count_joint] = [cross_axis_origindif[0], cross_axis_origindif[1], cross_axis_origindif[2], axis_world_joint[0], axis_world_joint[1], axis_world_joint[2]];
        }
        count_joint++;
        chain_joint_name = robot.links[robot.joints[chain_joint_name].parent].parent;
    }

    // generate the state control
    var state_control_matrix = [];
    if (kineval.params.ik_pseudoinverse){
        state_control_matrix = matrix_multiply(matrix_pseudoinverse(matrix_transpose(jacobian_matrix)), dx);
    }
    else{
        state_control_matrix = matrix_multiply(jacobian_matrix, dx);
    }
    count_joint = 0;
    flag_jac = true;
    chain_joint_name = endeffector_joint;
    while (flag_jac){
        if (robot.joints[chain_joint_name].parent === robot.base){
            flag_jac = false;
        }
        robot.joints[chain_joint_name].control += kineval.params.ik_steplength*state_control_matrix[count_joint][0];
        count_joint++;
        chain_joint_name = robot.links[robot.joints[chain_joint_name].parent].parent;
    }

}



