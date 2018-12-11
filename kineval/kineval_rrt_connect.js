
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
        
        // deal with different types of joint
        if ((robot.joints[x].type === 'prismatic')||(robot.joints[x].type === 'revolute'))
            q_start_config = q_start_config.concat(clamp_angle(robot.joints[x].angle, robot.joints[x].limit.lower, robot.joints[x].limit.upper));
        else 
            q_start_config = q_start_config.concat(mod_angle(robot.joints[x].angle));        
    }

    // set goal configuration as the zero configuration
    var i;
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // add parameter
    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    T_a_flag = true; 

    stepsize = 0.4; 
    threshold_for_goal = 0.2;
    radius_rrt = 1.5 * stepsize;
    margin_dist  = 3;
    joint_angle_scale = 0.15;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}


function robot_rrt_planner_iterate() {

    var length = T_a.vertices[0].vertex.length; 
    rrt_iter_count_max = 3000; 
    rrt_alg = 2;  // 0: RRT-star ; 1: RRT-connect 

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();
        var q_random = generate_random_config(length);

        if (rrt_alg === 1) {// RRT-connect algorithm
            extend_status = RRTextend(T_a, q_random);
            if (extend_status !== 'trapped') {
                connect_status = RRTconnect(T_b, T_a.vertices[T_a.newest].vertex);                
                if (connect_status === 'reached') {
                    rrt_iterate = false;
                    path_T_a = path_visualization(T_a, T_a.vertices[0], T_a.vertices[T_a.newest]);
                    path_T_b = path_visualization(T_b, T_b.vertices[0], T_b.vertices[T_b.newest]);
                    
                    kineval.motion_plan = [];
                    kineval.motion_plan_traversal_index = 0;

                    if (T_a_flag) {
                        var path_Ta_reverse = path_T_a.reverse();
                        kineval.motion_plan = path_Ta_reverse.concat(path_T_b);
                    }
                    else {
                        var path_Tb_reverse = path_T_b.reverse();
                        kineval.motion_plan = path_Tb_reverse.concat(path_T_a);
                    }
                    return connect_status;
                }
            }
            
            var T_temp = T_a;
            T_a = T_b;
            T_b = T_temp;

            T_a_flag = !T_a_flag;
            rrt_iter_count++;
            
            if (rrt_iter_count > rrt_iter_count_max) {rrt_iterate = false;}

        }  


        if (rrt_alg === 2) {//RRT-STAR

            var temp_f = (Math.random() <= threshold_for_goal);
            if (temp_f) 
                q_random = q_goal_config;

            extend_status = RRTstarextend(T_a, q_random);

            if (extend_status === "reached" && temp_f) {
                rrt_iterate = false;

                path_T_a = path_visualization(T_a, T_a.vertices[0], T_a.vertices[T_a.newest]);

                kineval.motion_plan = [];
                kineval.motion_plan_traversal_index = 0;
                kineval.motion_plan = path_T_a.reverse();
                return "reached";
            }

            rrt_iter_count++;

            if (rrt_iter_count > rrt_iter_count_max) {

                rrt_iterate = false;
                var opt_config = nearest_v(T_a, q_goal_config);
                path_T_a = path_visualization(T_a, T_a.vertices[0], T_a.vertices[opt_config.index]);

                kineval.motion_plan = [];
                kineval.motion_plan_traversal_index = 0;
                kineval.motion_plan = path_T_a.reverse();
                return "reached";
            }
        }

        // enforce joint limit to angle configurations, just in case
        traverse_normalize(T_a);
        traverse_normalize(T_b);
    }

    // path not found yet
    return false;
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
    tree.vertices[0].edges  = [];
    tree.vertices[0].visited  = false;
    tree.vertices[0].path_len = 0;

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
    new_vertex.visited = false; // for path retrieval

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;

    return new_vertex;
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

    var dist_q1_q2 = dist_config(tree.vertices[q1_idx].vertex, tree.vertices[q2_idx].vertex);
    dist_q1_q2 += tree.vertices[q2_idx].path_len;
    tree.vertices[q1_idx].path_len = dist_q1_q2;
}

function generate_random_config(l) {

    var q_random = [];
    q_random[1] = 0; 
    q_random[3] = 0; 
    q_random[5] = 0; 

    var x_corner1 = robot_boundary[0][0];
    var x_corner2 = robot_boundary[1][0];
    var z_corner1 = robot_boundary[0][2];
    var z_corner2 = robot_boundary[1][2];

    var x_random = (x_corner1 - margin_dist) + ((x_corner2 + margin_dist) - (x_corner1 - margin_dist)) * Math.random();
    var z_random = (z_corner1 - margin_dist) + ((z_corner2 + margin_dist) - (z_corner1 - margin_dist)) * Math.random();
    q_random[0] = x_random;
    q_random[2] = z_random;
    q_random[4] = 2 * Math.PI * Math.random();

    var i;
    for (i=6; i<l; i++) {
        if ((robot.joints[q_index[i]].type === 'revolute') || (robot.joints[q_index[i]].type === 'prismatic')) {
            var U = robot.joints[q_index[i]].limit.upper; 
            var L = robot.joints[q_index[i]].limit.lower; 
            q_random[i] = L + Math.random() * (U - L);
        }
        else
            q_random[i] = 2 * Math.PI * Math.random();
    }
    return q_random;
}

function new_configuration(start, end, stepsize) {

    var diff_config = [];
    var i;
    for (i=0; i<start.length; i++) {
        if (i < 3)
            diff_config[i] = end[i] - start[i];
        else
            diff_config[i] = (end[i] - start[i]) * joint_angle_scale;
    }
    
    var diff_config_normal = vector_normalize(diff_config);
    var new_config = {};
    var new_q = [];
    for (i=0; i<end.length; i++) {
        new_q[i] = start[i] + stepsize * diff_config_normal[i];
    }

    new_config.q = new_q;
    new_config.collision = kineval.poseIsCollision(new_q);

    return new_config;
}

function dist_config(q1, q2) {

    var distance = 0;
    var i;   
    if (rrt_alg === 1) {// for RRT-connect
        for (i=0; i<q1.length; i++)
            distance += (q1[i]-q2[i])*(q1[i]-q2[i]);
    }
    if (rrt_alg === 2) {// for RRT-star
        for (i=0; i<q1.length; i++){
            if (i < 3)
                distance += (q1[i]-q2[i])*(q1[i]-q2[i]);
            else
                distance += (q1[i]-q2[i])*(q1[i]-q2[i])*joint_angle_scale*joint_angle_scale;
        }       
    }
    return Math.sqrt(distance);
}

function nearest_v(T, q) {

    var distance = dist_config(q, T.vertices[0].vertex);
    var min_dist  = distance;
    var min_index = 0;
    var i;
    for (i=1; i<T.vertices.length; i++) {
        distance = dist_config(q, T.vertices[i].vertex);
        if (distance < min_dist) {
            min_dist  = distance;
            min_index = i; 
        }
    }
    var nearest_q    = {};
    nearest_q.index  = min_index;
    nearest_q.config = T.vertices[min_index].vertex;
    return nearest_q;
}

function path_visualization(T, start, latest) {

    var path = [];
    path = dfs_search(T, start, latest);
    var i;
    for (i=0; i<path.length; i++) {
        path[i].geom.material.color = {r:1,g:0,b:0};
    }
    return path;
}

function dfs_search(T, current, end) {
    
    var path_found;
    if (current === end) 
        return [current];

    current.visited = true;
    for (var i=0; i<current.edges.length; i++) {
        if (!current.edges[i].visited) {
            path_found = dfs_search(T, current.edges[i], end);
            if (path_found) {
                return path_found.concat(current);
            }
        }
    }
    return false; 
}

function traverse_normalize(T) {
    var i;
    var j;
    for (i=0; i<T.vertices.length; i++) {
        for (j=3; j<T.vertices[i].vertex.length; j++) {
            if ((j>5) && ((robot.joints[q_index[j]].type === 'prismatic')||(robot.joints[q_index[j]].type === 'revolute')))
                T.vertices[i].vertex[j] = clamp_angle(T.vertices[i].vertex[j], robot.joints[q_index[j]].limit.lower, robot.joints[q_index[j]].limit.upper);
            else
                T.vertices[i].vertex[j] = mod_angle(T.vertices[i].vertex[j]);
        }
    }
}

function RRTconnect(T, q) {
    extend_status = "advanced"; 
    while (extend_status === "advanced") {
        extend_status = RRTextend(T, q);
    }   
    return extend_status;
}

function RRTextend(T, q) {
    var vertex_nearest_q = nearest_v(T, q);
    var new_config = new_configuration(vertex_nearest_q.config, q, stepsize);     

    if (!new_config.collision) {

        var new_vertex = tree_add_vertex(T, new_config.q);
        tree_add_edge(T, T.vertices.length-1, vertex_nearest_q.index);

        var distance = dist_config(new_config.q, q); 
        if (distance > stepsize)
            return "advanced"; 
        else
            return "reached";
    }
    return "trapped"; 
}

function RRTstarextend(T, q) {

    var vertex_nearest_q = nearest_v(T, q); 
    var new_config = new_configuration(vertex_nearest_q.config, q, stepsize);

    if (!new_config.collision) {

        var neighbors = get_neighborhood(T, new_config.q);
        var parent_index = choose_parent(T, neighbors, new_config.q);

        var new_vertex = tree_add_vertex(T, new_config.q);
        tree_add_edge(T, T.vertices.length-1, parent_index);

        rewire_neighborhood(neighbors, new_vertex);        
        
        // check the extend status
        var distance = dist_config(new_config.q, q);
        if (distance > stepsize)
            return "advanced"; 
        else
            return "reached";
    }
    return "trapped"; 
}


function mod_angle(theta) {
    if (theta > (2 * Math.PI)) 
        return theta % (2 * Math.PI);
    else if (theta < 0)
        return (2 * Math.PI) + (theta % (2 * Math.PI));
    else 
        return theta;     
}

function clamp_angle(theta, L, U) {
    return Math.min(U, Math.max(L, theta));
}

function get_neighborhood(T, q) {
    var neighbors = [];
    var i;
    for (i=0; i<T.vertices.length; i++) {
        distance = dist_config(q, T.vertices[i].vertex);
        if (distance <= radius_rrt)
            neighbors.push(T.vertices[i])
    }
    return neighbors;
}

function choose_parent(T, neighbors, q) {

    var cost = dist_config(q, neighbors[0].vertex) + neighbors[0].path_len;
    var min_cost  = cost;
    var min_index = 0; 
    var i;
    for (i=1; i<neighbors.length; i++){
        cost = dist_config(q, neighbors[i].vertex) + neighbors[i].path_len;
        if (cost < min_cost) {
            min_cost  = cost;
            min_index = i; 
        }
    }
    return T.vertices.indexOf(neighbors[min_index]);
}

function rewire_neighborhood(neighbors, new_config) {
    var i;

    for (i=0; i<neighbors.length; i++) {
        var z_near = neighbors[i];
        var z_near_parent = z_near.edges[0];
        if (typeof(z_near_parent) === 'undefined'){continue;}

        var new_cost = new_config.path_len + dist_config(new_config.vertex, z_near.vertex);
        if (new_cost < z_near.path_len) {

            z_near.edges[0] = new_config;
            new_config.edges.push(z_near);
            z_near.path_len = new_cost;

            var child_index = z_near_parent.edges.indexOf(z_near);
            z_near_parent.edges.splice(child_index, 1);
        }
    }
}

function steer(q1, q2) {

    var distance = dist_config(q1, q2);
    if (distance < stepsize) {return distance;}
    else {
        var tmp_config = new_configuration(q1, q2, stepsize); 
        if (!tmp_config.collision) {
            var dist_steer = steer(tmp_config.q, q2);
            if (dist_steer >= 0) {
                return dist_steer + stepsize; 
            }
            else {
                return -1; 
            }
        }
        else {
            return -1; 
        }
    }
}