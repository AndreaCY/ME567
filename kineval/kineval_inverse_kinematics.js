
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

    // STENCIL: implement inverse kinematics iteration
    var m_transfer = [];
    var endeffector_xyz = [];
    var endeffector_rpy = [];
    var endeffector_worldposition = [];

    m_transfer = robot.joints[endeffector_joint].xform;
    endeffector_xyz = matrix_multiply(m_transfer, endeffector_position_local);
    endeffector_rpy = [[Math.atan2(m_transfer[2][1], m_transfer[2][2])], [Math.atan2(-m_transfer[2][0], Math.sqrt(Math.pow(m_transfer[2][1], 2)+Math.pow(m_transfer[2][2], 2)))], [Math.atan2(m_transfer[1][0], m_transfer[0][0])]];
    endeffector_worldposition = [[endeffector_xyz[0][0]], [endeffector_xyz[1][0]], [endeffector_xyz[2][0]], [endeffector_rpy[0][0]], [endeffector_rpy[1][0]], [endeffector_rpy[2][0]]];
 
    var minix = [];
    var a;   
    for (a = 0; a< 6; a++){
        minix[a] = [];
        if (a < 3){
            minix[a][0] = endeffector_target_world.position[a] - endeffector_worldposition[a][0];
        }
        else{
            if (kineval.params.ik_orientation_included){
                minix[a][0] = endeffector_target_world.orientation[a-3] - endeffector_worldposition[a][0];
            }
            else{minix[a][0] = 0;}
        }
    }

    // create jacobian_matrix
    var jacobian_matrix = [];
    var count_joint = 0;
    var flag_jacobian = true;
    var joint_onchain = endeffector_joint;
    
    // use a flag for determination
    while (flag_jacobian){
        if (robot.joints[joint_onchain].parent === robot.base){
            flag_jacobian = false;
        }
        
        var m_transfer_joint = [];
        var world_joint_axis = [];
        var i,j;
        var world_joint_orig = [];
        var origin_difference_joint = [];
        var origin_difference_cro_axis = [];

        //transfer joint matrix
        for (i = 0; i< 3; i++){
            m_transfer_joint[i] = [];
            for (j = 0; j< 3; j++){
                m_transfer_joint[i][j] = robot.joints[joint_onchain].xform[i][j];
            }
        }

        //define axis of world joint using transfer joint matrix
        world_joint_axis = matrix_multiply(m_transfer_joint, [[robot.joints[joint_onchain].axis[0]],[robot.joints[joint_onchain].axis[1]], [robot.joints[joint_onchain].axis[2]]]);
        world_joint_axis = [world_joint_axis[0][0], world_joint_axis[1][0], world_joint_axis[2][0]];
        world_joint_axis = vector_normalize(world_joint_axis);

        if (robot.joints[joint_onchain].type === "prismatic"){
            jacobian_matrix[count_joint] = [world_joint_axis[0], world_joint_axis[1], world_joint_axis[2], 0, 0, 0];
        }
        else{
            world_joint_orig = matrix_multiply(robot.joints[joint_onchain].xform, [[0],[0],[0],[1]]);
            origin_difference_joint = [endeffector_xyz[0][0] - world_joint_orig[0][0], endeffector_xyz[1][0] - world_joint_orig[1][0], endeffector_xyz[2][0] - world_joint_orig[2][0]];
            origin_difference_cro_axis = vector_cross(world_joint_axis, origin_difference_joint);
            jacobian_matrix[count_joint] = [origin_difference_cro_axis[0], origin_difference_cro_axis[1], origin_difference_cro_axis[2], world_joint_axis[0], world_joint_axis[1], world_joint_axis[2]];
        }
        
        joint_onchain = robot.links[robot.joints[joint_onchain].parent].parent;
        count_joint += 1;
    }

    // Create the control
    var control_matrix = [];
    if (kineval.params.ik_pseudoinverse){
        control_matrix = matrix_multiply(matrix_pseudoinverse(matrix_transpose(jacobian_matrix)), minix);
    }
    else{
        control_matrix = matrix_multiply(jacobian_matrix, minix);
    }
    count_joint = 0;
    flag_jacobian = true;
    joint_onchain = endeffector_joint;
    while (flag_jacobian){
        if (robot.joints[joint_onchain].parent === robot.base){
            flag_jacobian = false;
        }
        robot.joints[joint_onchain].control += kineval.params.ik_steplength * control_matrix[count_joint][0];
        joint_onchain = robot.links[robot.joints[joint_onchain].parent].parent;
        count_joint += 1;
    }

}



