
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    // step size for angle and distance
    eps = 1.0;

    // initialize T_a, T_b
    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
        rrt_iter_count++;

        var q_rand = randomConfig();
        var last_Tanum = T_a.newest;
        T_a = rrt_extend(T_a, q_rand);

        if (rrt_alg == 0){              // rrt
            if (distance_q_q(T_a.vertices[T_a.newest].vertex, q_goal_config) < eps/2){
                rrt_iterate = false;
                // path building
                var path_idx = T_a.newest;
                var path = [];
                while(path_idx != null){
                    path.push(T_a.vertices[path_idx]);
                    T_a.vertices[path_idx].geom.material.color = {r:1,g:0,b:0};
                    path_idx = T_a.vertices[path_idx].parent;
                }
                kineval.motion_plan_traversal_index = 0;
                kineval.motion_plan = path.reverse();
                return "reached";
            }
            else if (last_Tanum == T_a.newest){
                rrt_iterate = true;
                return "failed";
            }
            else{
                rrt_iterate = true;
                return "extended";
            }
        }
        if (rrt_alg == 1){              // rrt_connect
            // if (distance_q_q(T_a.vertices[T_a.newest].vertex, q_goal_config) < eps/2){
            //     rrt_iterate = false;
            //     // path building
            //     var path_idx = T_a.newest;
            //     var path_T_a = [];
            //     while(path_idx != null){
            //         path_T_a.push(T_a.vertices[path_idx]);
            //         T_a.vertices[path_idx].geom.material.color = {r:1,g:0,b:0};
            //     }
            //     kineval.motion_plan_traversal_index = 0;
            //     kineval.motion_plan = path_T_a.reverse();
            //     return "reached";
            // }
            if (last_Tanum == T_a.newest){
                rrt_iterate = true;
                return "failed";
            }
            else{
                var q_new = T_a.vertices[T_a.newest].vertex;
                if (rrt_connect(q_new)){
                    rrt_iterate = false;
                    // path building
                    var path_idx = T_a.newest;
                    var path_T_a = [];
                    var is_start = 0;
                    while(path_idx != null){
                        if (path_idx == 0){
                            is_start = 1;
                        }
                        path_T_a.push(T_a.vertices[path_idx]);
                        T_a.vertices[path_idx].geom.material.color = {r:1,g:0,b:0};
                        path_idx = T_a.vertices[path_idx].parent;
                    }
                    var path_idx = T_b.newest;
                    var path_T_b = [];
                    while(path_idx != null){
                        path_T_b.push(T_b.vertices[path_idx]);
                        T_b.vertices[path_idx].geom.material.color = {r:1,g:0,b:0};
                        path_idx = T_b.vertices[path_idx].parent;
                    }
                    kineval.motion_plan_traversal_index = 0;
                    if (is_start == 1){
                        kineval.motion_plan = (path_T_a.reverse()).concat(path_T_b);
                    }
                    else{
                        kineval.motion_plan = (path_T_b.reverse()).concat(path_T_a);
                    }
                    return "reached";
                }
                var temp = T_a;
                T_a = T_b;
                T_b = temp;
                rrt_iterate = true;
                return "extended";
            }

        }
        if (rrt_alg == 2){              // rrt_star
            var q_nearest_idx = nearest_neighbor(T_a, q_rand);
            var q_new = new_config(q_rand, T_a.vertices[q_nearest_idx].vertex);
            if (!kineval.poseIsCollision(q_new)){
                var q_near_new_idx = find_near_vertices_RRTSTAR(q_new, eps*2);
                var q_min_idx = chooseparent_RRTSTAR(q_near_new_idx, q_nearest_idx, q_new);
                rewire_RRTSTAR(q_near_new_idx, q_min_idx);
            }
            if (distance_q_q(T_a.vertices[T_a.newest].vertex, q_goal_config) < eps/2){
                rrt_iterate = false;
                // path building
                var path_idx = T_a.newest;
                var path = [];
                while(path_idx != null){
                    path.push(T_a.vertices[path_idx]);
                    T_a.vertices[path_idx].geom.material.color = {r:1,g:0,b:0};
                    path_idx = T_a.vertices[path_idx].parent;
                }
                kineval.motion_plan_traversal_index = 0;
                kineval.motion_plan = path.reverse();
                return "reached";
            }
            else if (last_Tanum == T_a.newest){
                rrt_iterate = true;
                return "failed";
            }
            else{
                rrt_iterate = true;
                return "extended";
            }

        }

    }

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs


function rrt_connect(q_new){
    while (!(distance_q_q(T_b.vertices[T_b.newest].vertex, q_new) < eps/2)){
        var last_Tanum = T_b.newest;
        T_b = rrt_extend(T_b, q_new);
        if (last_Tanum == T_b.newest){
            return false;
        }
    }
    return true;
}

function rrt_extend(tree, q_rand){
    var q_nearest_idx = nearest_neighbor(tree, q_rand);
    var q_new = new_config(q_rand, tree.vertices[q_nearest_idx].vertex);
    if (!kineval.poseIsCollision(q_new)){
        tree_add_vertex(tree, q_new);
        tree.vertices[tree.newest].parent = q_nearest_idx;
    }
    return tree;
}

function nearest_neighbor(tree, q_rand){
    var i;
    var tree_index = 0;
    var dist_qrand_q = distance_q_q(q_rand, tree.vertices[0].vertex);
    for (i = 1; i <= tree.newest; i++){
        var new_dist_qrand_q = distance_q_q(q_rand, tree.vertices[i].vertex);
        if (dist_qrand_q > new_dist_qrand_q){
            dist_qrand_q = new_dist_qrand_q;
            tree_index = i;
        }
    }
    return tree_index;
}

function new_config(q_rand, q_nearest){
    var dist_qrand_qnearst = distance_q_q(q_rand, q_nearest);
    var prop = eps/dist_qrand_qnearst;
    var q_new = [];
    var i;
    for (i = 0; i < q_rand.length; i++){
        q_new.push(q_nearest[i] + (q_rand[i] - q_nearest[i])*prop);
    }
    for (x in robot.joints) {
        switch (robot.joints[x].type) {
            case "revolute":
            case "prismatic":
                if (q_new[q_names[x]] > robot.joints[x].limit.upper){
                    q_new[q_names[x]] = robot.joints[x].limit.upper;
                }
                if (q_new[q_names[x]] < robot.joints[x].limit.lower){
                    q_new[q_names[x]] = robot.joints[x].limit.lower;
                }
                break;
            case "fixed":
                q_new[q_names[x]] = 0;
                break;
            case "continuous":
            default:
                break;
        }
    }
    q_new[1] = 0;
    q_new[3] = 0;
    q_new[5] = 0;
    return q_new;

}

function randomConfig(){

    var x_min = robot_boundary[0][0];
    var x_max = robot_boundary[1][0];
    var z_min = robot_boundary[0][2];
    var z_max = robot_boundary[1][2];
    var angle_min = 0;
    var angle_max = 2*Math.PI;
    var sampling_num_dis_z = (z_max - z_min)/eps;
    var sampling_num_dis_x = (x_max - x_min)/eps;
    var sampling_num_ang = (angle_max - angle_min)/eps;
    var x_random = Math.floor(sampling_num_dis_x*Math.random())*eps + x_min;
    var z_random = Math.floor(sampling_num_dis_z*Math.random())*eps + z_min;
    var angle_random = Math.floor(sampling_num_ang*Math.random())*eps + angle_min;
    var q_rand = [
        x_random,
        0,
        z_random,
        0,
        angle_random,
        0
    ];
    for (x in robot.joints){
        var angle_random = Math.floor(sampling_num_ang*Math.random())*eps + angle_min;
        q_rand.push(angle_random);
    }
    if (rrt_alg != 1){ // trick
        var num_rand = Math.random();
        if (num_rand > 0.75){
            return q_rand;
        }
        else{
            return q_goal_config;
        }
    }
    return q_rand;

}

function distance_q_q(q1, q2){
    var i;
    var dist = 0;
    for (i = 0; i < q1.length; i++){
        if (i > 5){
            dist += 0.01*Math.pow((q1[i]-q2[i]),2);
        }
        else{
            dist += Math.pow((q1[i]-q2[i]),2);
        }
    }
    dist = Math.sqrt(dist);
    return dist;
}








