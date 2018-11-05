
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints
    if (!kineval.params.update_pd_clock){
        var current_date = new Date();

        if (lastseconds.getSeconds() < current_date.getSeconds()){
            kineval.params.dance_pose_index = kineval.params.dance_sequence_index.shift();
            // change all joints
            for (x in robot.joints) {
                kineval.params.setpoint_target[x] = kineval.setpoints[kineval.params.dance_pose_index][x];
            }

            kineval.params.dance_sequence_index.push(kineval.params.dance_pose_index);
        }
        // set the latest as current for new turn
        lastseconds = current_date;
    }
}

kineval.setpointClockMovement = function execute_clock() {
    // if update not requested, exit routine 
    if (!kineval.params.update_pd_clock) return;

    var current_date = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = current_date.getSeconds()/(60*2*Math.PI);
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {
    // if update not requested, exit routine 
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return;

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (x in robot.joints) {
        robot.joints[x].control = (kineval.params.setpoint_target[x] - robot.joints[x].angle)*robot.joints[x].servo.p_gain;
    }
}
