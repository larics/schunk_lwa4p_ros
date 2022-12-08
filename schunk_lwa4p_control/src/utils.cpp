




// TODO: Finish this utils.cpp
double ControlArm::VectorSize(geometry_msgs::Vector3 vector) {
    // TODO: Add this to utils method
    return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}
double ControlArm::DotProduct(geometry_msgs::Vector3 v_A,
                              geometry_msgs::Vector3 v_B) {
    // TODO: Move this to utils method
    return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}
geometry_msgs::Vector3 ControlArm::getClosestPointOnLine(geometry_msgs::Vector3 line_point,
                                  geometry_msgs::Vector3 line_vector,
                                  geometry_msgs::Vector3 point) {
    // TODO: Add this to utils methods
    double x1 = line_point.x - point.x;
    double y1 = line_point.y - point.y;
    double z1 = line_point.z - point.z;
    double vx = line_vector.x;
    double vy = line_vector.x;
    double vz = line_vector.x;
    geometry_msgs::Vector3 p1, vector, result;
    p1.x = x1;
    p1.y = y1;
    p1.z = z1;
    // std::cout<<"transform base power_line "<<p1;
    vector.x = vx;
    vector.y = vy;
    vector.z = vz;
    // std::cout<<"line vector "<<vector;
    double t = -DotProduct(p1, vector) / VectorSize(vector) / VectorSize(p1);
    // std::cout<<"t "<<t<<std::endl;
    result.x = x1 + t * vx;
    result.y = y1 + t * vy;
    result.z = z1 + t * vz;
    return result;
}