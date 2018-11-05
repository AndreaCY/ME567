//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

function quaternion_from_axisangle(theta, axis_normal) {
    var quaternion_f_a = [Math.cos(0.5*theta), Math.sin(0.5*theta)*axis_normal[0], 
                        Math.sin(0.5*theta)*axis_normal[1], Math.sin(0.5*theta)*axis_normal[2]];
    return quaternion_f_a;
}

function quaternion_normalize(a) {
    var normal = Math.sqrt(Math.pow(a[0],2) + Math.pow(a[1],2) + Math.pow(a[2],2) + Math.pow(a[3],2));
    var qua_normal = [a[0]/normal, a[1]/normal, a[2]/normal, a[3]/normal];
    return qua_normal;
}

function quaternion_to_rotation_matrix(a) {
    var qua_t_m = [[1-2*(Math.pow(a[2],2) + Math.pow(a[3],2)), 2*(a[1]*a[2] - a[0]*a[3]), 2*(a[0]*a[2] + a[1]*a[3]),0],
                    [2*(a[1]*a[2] + a[0]*a[3]), 1 - 2*(Math.pow(a[1],2) + Math.pow(a[3],2)), 2*(a[2]*a[3] - a[0]*a[1]),0],
                    [2*(a[1]*a[3] - a[0]*a[2]), 2*(a[0]*a[1] + a[2]*a[3]), 1 - 2*(Math.pow(a[1],2) + Math.pow(a[2],2)),0], [0,0,0,1]];
    return qua_t_m;
}

function quaternion_multiply(a,b) {
    var qua_multi = [a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3], 
                    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2], 
                    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1], 
                    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]];
    return qua_multi;
}