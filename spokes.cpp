#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>

#include "PostScript.hpp"

using namespace Eigen;

template <typename Real>
Matrix<Real, 2, 1> findSegmentIntersection(
    const Matrix<Real, 2, 1>& p1, const Matrix<Real, 2, 1>& p2,
    const Matrix<Real, 2, 1>& q1, const Matrix<Real, 2, 1>& q2)
{

    // Define direction vectors
    Matrix<Real, 2, 1> r = p2 - p1;
    Matrix<Real, 2, 1> s = q2 - q1;

    // Construct the matrix A and vector b: A * [t, u]^T = b
    Matrix<Real, 2, 2> A;
    A << r, -s;
    Matrix<Real, 2, 1> b = q1 - p1;

    // Solve for x = [t, u]^T using ColPivHouseholderQr for numerical stability
    Matrix<Real, 2, 1> x = A.colPivHouseholderQr().solve(b);
    Real t = x(0);
    Real u = x(1);



    // The segments intersect if and only if both parameters fall within [0, 1]
    assert (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0);

    return  p1 + t * r; // Or q1 + u * s
}

template <typename Real, int n>
class Node {
public:
    using Vector = Matrix<Real, n, 1>;
    int idx = 0; // aux index of the node in the system's node list
    Vector force; // total force on the node
    Vector position;
    Vector displacement;
    Node() = default;
    Node(const Vector& position) : position(position), displacement(Vector::Zero()) {}
};

template <typename Real, int n>
class Spring {
public:
    using Vector = Matrix<Real, n, 1>;
    Node<Real, n>& nodeA;
    Node<Real, n>& nodeB;
    Real k; // stiffness
    Real elongation() const {
        return (nodeB.position + nodeB.displacement - nodeA.position - nodeA.displacement).norm() - (nodeB.position - nodeA.position).norm();
        //Vector dir = (nodeB.position - nodeA.position).normalized();
        //return dir.dot(nodeB.displacement - nodeA.displacement);
    }
    Real force() const {
        return k * elongation();
    }
    Vector forceVector() const {
        Vector newDir = (nodeB.position + nodeB.displacement - nodeA.position - nodeA.displacement).normalized();
        return force() * newDir;
    }
    Real length() const {
        return (nodeB.position - nodeA.position).norm();
    }
    Spring() = default;
    Spring(Node<Real, n>& nodeA, Node<Real, n>& nodeB, Real k) : nodeA(nodeA), nodeB(nodeB), k(k) {}
};

template <typename Real, int n>
Real springStiffness(const Matrix<Real, n, 1>& a,
                     const Matrix<Real, n, 1>& b,
                     Real modulus, Real diameter) {
    Real length = (b - a).norm();
    Real area = M_PI * diameter * diameter / 4.0;
    return modulus * area / length;
}


template <typename Real, int n>
class System {
public:
    using Vector = Matrix<Real, n, 1>;
    std::deque<Node<Real, n>> nodes;
    std::vector<Spring<Real, n>> springs;
    std::vector<int> freeNodes; // indices of free nodes (not fixed in space)
    void addNode(const Vector& position) {
        nodes.emplace_back(position);
        nodes.back().idx = nodes.size() - 1;
    }
    void addSpring(int nodeA, int nodeB, double k)
    {
        springs.push_back(Spring<Real, n>(nodes[nodeA], nodes[nodeB], k));
    }
    void addMaterialSpring(int nodeA, int nodeB, Real modulus, Real diameter)
    {
        addSpring(nodeA, nodeB, springStiffness(nodes[nodeA].position, nodes[nodeB].position, modulus, diameter));
    }
    void calculateForces() {
        for(auto& node : nodes) {
            node.force.setZero();
        }
        for(auto& spring : springs) {
            Vector f = spring.forceVector();
            //std::cout<<spring.elongation()<<" / "<<spring.length()<<"     ->   "<<spring.force()<<" N"<<std::endl;
            spring.nodeA.force += f;
            spring.nodeB.force -= f;
        }
    }
    void freeNode(int idx) {
        freeNodes.push_back(idx);
    }
};

double hubMoment(System<double, 2>& wheel, size_t numSpokes) {
    double moment = 0.0;
    wheel.calculateForces();
    for(size_t i = 0; i < numSpokes; i++) {
        Vector2d force = wheel.nodes[i].force;
        Vector2d leverArm = wheel.nodes[i].position; // assuming the hub is at the origin
        moment += leverArm.x() * force.y() - leverArm.y() * force.x(); // cross product in 2D
    }
    return moment;
}
void naiveSolve(System<double, 2>& system) {
    double maxShift = 1e-4;
    for(int iter = 0; iter < 100000000; iter++) {
        //std::cout<<hubMoment(system, 16)<<std::endl;
        system.calculateForces();
        for(size_t i : system.freeNodes) {
            Vector2d shift = system.nodes[i].force.normalized() * maxShift / (iter+1);
            system.nodes[i].displacement += shift;
        }
        //std::cout<<system.nodes[system.freeNodes[0]].force.transpose()<<std::endl;
    }
    //system.calculateForces();
    //for(size_t i : system.freeNodes) {
    //    std::cout<<"f = "<<system.nodes[i].force.norm()<<std::endl;
    //}
}

// creates a system representing a wheel with spokes connecting rim holes to hub holes, but without any constraints (i.e. unsoldered)
System<double, 2> unsolderedWheel(const std::vector<Eigen::Vector2d>& rimHoles,
                                  const std::vector<Eigen::Vector2d>& hubHoles,
                                  double modulus, double diameter)
{
    assert(hubHoles.size() == rimHoles.size());
    System<double, 2> system;
    for(const auto& hole : hubHoles) {
        system.addNode(hole);
    }
    for(const auto& hole : rimHoles) {
        system.addNode(hole);
    }
    for(size_t i=0; i < rimHoles.size(); i++) {
        size_t j = i + hubHoles.size(); // index of corresponding rim hole
        system.addMaterialSpring(i, j, modulus, diameter);
    }
    return system;
}

// creates a system representing a wheel with spokes connecting rim holes to hub holes,
// with leading and trailing spokes soldered together at the intersection point (i.e. soldered)
System<double, 2> solderedWheel(const std::vector<Eigen::Vector2d>& rimHoles,
                                const std::vector<Eigen::Vector2d>& hubHoles,
                                double modulus, double diameter)
{
    assert(hubHoles.size() == rimHoles.size());
    System<double, 2> system;
    for(const auto& hole : hubHoles) {
        system.addNode(hole);
    }
    for(const auto& hole : rimHoles) {
        system.addNode(hole);
    }
    for(size_t i=0; i < hubHoles.size(); i+=2) {
        size_t j = i + hubHoles.size(); // index of corresponding rim hole node
        Vector2d intersection = findSegmentIntersection(hubHoles[i], rimHoles[i], hubHoles[i+1], rimHoles[i+1]);
        system.addNode(intersection); // add intersection node
        size_t intersectionIdx = system.nodes.size() - 1; // index of the intersection node

        system.addMaterialSpring(i, intersectionIdx, modulus, diameter); // leading spoke
        system.addMaterialSpring(j, intersectionIdx, modulus, diameter); // leading spoke

        system.addMaterialSpring(i+1, intersectionIdx, modulus, diameter); // trailing spoke
        system.addMaterialSpring(j+1, intersectionIdx, modulus, diameter); // trailing spoke

        system.freeNode(intersectionIdx); // free the soldered node for optimisation
    }
    return system;
}


void drawWheel(PostScript& ps,
               double wheelRadius, double hubRadius,
               const std::vector<Eigen::Vector2d>& rimHoles,
               const std::vector<Eigen::Vector2d>& hubHoles)
{
    assert(hubHoles.size() == rimHoles.size());
    ps.setLineWidth(0.0002);
    ps.circle(0, 0, wheelRadius);
    ps.circle(0, 0, 1.1*hubRadius);
    for(size_t i = 0; i < hubHoles.size(); i++) {
        ps.line(hubHoles[i].x(), hubHoles[i].y(), rimHoles[i].x(), rimHoles[i].y());
    }
    for(size_t i = 0; i < hubHoles.size(); i+=2) {
        Vector2d intersection = findSegmentIntersection(hubHoles[i], rimHoles[i], hubHoles[i+1], rimHoles[i+1]);
        ps.circle(intersection.x(), intersection.y(), 0.003, true);
    }
}

int main(){
    double d = 2e-3; // diameter in meters
    double E = 200e9; // Young's modulus in Pascals
    double rimRadius = 0.3; // wheel radius in meters (effective rim radius)
    double hubRadius = 0.03; // hub radius in meters
    int numSpokes = 16; // number of spokes
    int numCrossed = 3; // number of crossings per spoke
    std::vector<Eigen::Vector2d> hubHoles;
    std::vector<Eigen::Vector2d> rimHoles;

    for(int i = 0; i < numSpokes; i++) {
        double angle = 2 * M_PI * i / numSpokes;
        int skipHoles = numCrossed; //  2*numCrossed - 1;
        rimHoles.emplace_back(rimRadius * cos(angle), rimRadius * sin(angle));
        bool isLeading = (i % 2 == 0);
        if(isLeading) {
            angle += skipHoles * 2 * M_PI / numSpokes;
        } else {
            angle -= skipHoles * 2 * M_PI / numSpokes;
        }
        hubHoles.emplace_back(hubRadius * cos(angle), hubRadius * sin(angle));
    }

    double rotationAngle = 0.001; // rotation of the hub
    Matrix2d R;
    R << cos(rotationAngle), -sin(rotationAngle),
         sin(rotationAngle), cos(rotationAngle);
    Matrix2d D = R - Matrix2d::Identity(); // displacement of the hub holes after rotation

    System<double, 2> wheel = unsolderedWheel(rimHoles, hubHoles, E, d);
    System<double, 2> wheel2 = solderedWheel(rimHoles, hubHoles, E, d);
    for(size_t i = 0; i < hubHoles.size(); i++) {
        wheel.nodes[i].displacement = D * wheel.nodes[i].position;
        wheel2.nodes[i].displacement = D * wheel2.nodes[i].position;
    }

    std::cout<<"Hub moment of an unsoldered wheel: "<<hubMoment(wheel, numSpokes)<<" Nm"<<std::endl;
    naiveSolve(wheel2);
    std::cout<<"Hub moment of a soldered wheel: "<<hubMoment(wheel2, numSpokes)<<" Nm"<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Unsoldered wheel spoke tensions:"<<std::endl;
    for(auto& spoke : wheel.springs) {
        std::cout<<"Spoke tension change: "<<spoke.force()<<" N"<<std::endl;
        //std::cout<<"Spoke elongation: "<<spoke.elongation()<<" m"<<std::endl;
    }
    std::cout<<std::endl;
    std::cout<<"Soldered wheel spoke tensions:"<<std::endl;
    for(size_t i=0; i<wheel2.springs.size(); i+=2) {
        std::cout<<"Spoke tension change: "<<wheel2.springs[i].force()<<" N  "<<wheel2.springs[i+1].force()<<std::endl;
        //std::cout<<"Spoke elongation: "<<wheel2.springs[i].elongation() + wheel2.springs[i+1].elongation()<<" m"<<std::endl;
    }

    PostScript ps(-1100*rimRadius, -1100*rimRadius, 1100*rimRadius, 1100*rimRadius);
    ps.scale(1000.0);
    drawWheel(ps, rimRadius, hubRadius, rimHoles, hubHoles);

    //ps.setColor(255, 0, 0);
    //for(auto&& node : wheel.nodes) {
    //    ps.line(node.position.x(), node.position.y(),
    //            node.position.x() + node.displacement.x(),
    //            node.position.y() + node.displacement.y());
    //}

    ps.saveFile("wheel.ps");
}
