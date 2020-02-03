#include <eigen3/Eigen/Dense>
#include <cmath>
#include <unistd.h>
#include <random>
#include <chrono>
#include <iostream>
#include "vibes.h"

using namespace std;
using namespace Eigen;

void kalman_predict(Vector4d& x1, Matrix4d& Gx1, Vector4d& xup, Matrix4d& Gup, Vector4d& u, Matrix4d& Galpha, Matrix4d& A){
    Gx1 = A * Gup * A.transpose() + Galpha;
    x1 = A * xup + u;
}

void kalman_correct(Vector4d&xup, Matrix4d& Gup, Vector4d& x0, Matrix4d& Gx0, Vector3d& y, Matrix3d& Gbeta, MatrixXd& C){
    MatrixXd S = C * Gx0 * C.transpose() + Gbeta;
    MatrixXd K = Gx0 * C.transpose() * S.inverse();
    VectorXd ytilde = y - C * x0;
    Gup = (MatrixXd::Identity(x0.size(), x0.size()) - K * C) * Gx0;
    xup = x0 + K * ytilde;
}

void kalman(Vector4d& x0, Matrix4d& Gx0, Vector4d& u, Matrix4d& Galpha, Matrix4d& A, Vector3d& y, Matrix3d& Gbeta, MatrixXd& C){
    Vector4d xup;
    Matrix4d Gup;
    kalman_correct(xup, Gup, x0, Gx0, y, Gbeta, C);
    kalman_predict(x0, Gx0, xup, Gup, u, Galpha, A);
}

Vector2d command(Vector4d& x, Vector2d& w, Vector2d& dw, Vector2d& ddw, const double T){
    Matrix2d A;
    A << cos(x[2]) -x[3] * sin(x[2]), cos(x[2]) + x[3] * sin(x[2]),
    sin(x[2]) + x[3] * cos(x[2]), sin(x[2]) - x[3] * cos(x[2]);
    Vector2d b = {-x[3] * abs(x[3]) * cos(x[2]), -x[3] * abs(x[3]) * sin(x[2])};
    Vector2d y = {x[0], x[1]};
    Vector2d dy = {x[3] * cos(x[2]), x[3] * sin(x[2])};
    Vector2d v = pow(1.0/T, 2) * (w - y) + (2.0/T) * (dw - dy) + ddw;
    Vector2d u = A.inverse() * (v - b);
    return u;
}

void lissajous(Vector2d& w, Vector2d& dw, Vector2d& ddw, double t, const double p, const double q, const double a, double b){
    // p = 1, q = 2, donne la courbe attendue. Multiplier p et q par un même facteur permet de gérer la vitesse de
    // parcours de la courbe.
    w << a * sin(p * t), b * sin(q * t);
    dw << a * p * cos(p * t), b * q * cos(q * t);
    ddw << -a * pow(p, 2) * sin(p * t), -b * pow(q, 2) * sin(q * t);
}

void waypoint(Vector2d& w, Vector2d& dw, Vector2d& ddw, double t, double x, double y){
    w << x, y;
    dw << 0, 0;
    ddw << 0, 0;
}

Vector4d dyn(Vector4d& x, Vector2d& u){
    Vector4d xdot = {x[3] * cos(x[2]), x[3] * sin(x[2]), u[0] - u[1], u[0] + u[1] - x[3] * abs(x[3])};
    return xdot;
}

int main(){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    vibes::beginDrawing();
    vibes::newFigure("DDBoat");
    vibes::setFigureProperties("DDBoat",vibesParams("x", 100, "y", 100, "width", 700, "height", 700));
    Vector4d x = {10, 30, 0, 0.1};
    Vector2d w, dw, ddw, u;
    const double p = 0.1,q = 0.2,a = 40,b = 20, T = 10;
    double dt = 0.1;

    Vector4d x0 = {10., 30., 0., 0.1};
    Matrix4d Gx0 = 10 * MatrixXd::Identity(4, 4);
    Matrix4d Galpha = MatrixXd::Zero(4, 4);
    MatrixXd C(3, 4);
    C << 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.;
    Matrix2d B;
    B << 1, -1, 1, 1;
    Matrix3d Gbeta;
    Gbeta << 5, 0, 0, 0, 5, 0, 0, 0, M_PI/180.;
    std::normal_distribution<double> distribution (0.0,5.0);
    std::normal_distribution<double> distribution2 (0.0,M_PI/180.);

    for (float t = 0; t < 100; t+= dt){
        //vibes::clearFigure("DDBoat");
        vibes::axisLimits(-50, 50, -50, 50, "DDBoat");
        lissajous(w, dw, ddw, t, p, q, a, b);
        //waypoint(w, dw, ddw, t, 0, 0);
        u = command(x0, w, dw, ddw, T);
        Vector4d ukal = {0, 0, dt * (B * u)[0], dt * (B * u)[1]};
        Vector3d ykal = {x[0] + distribution(generator), x[1] + distribution(generator), x[2] + distribution2(generator)};
        Matrix4d A;
        A << 1, 0, 0, dt * cos(x0[2]), 0, 1, 0, dt * sin(x0[2]), 0, 0, 1, 0, 0, 0, 0, 1 - dt * abs(x0[3]);
        kalman(x0, Gx0, ukal, Galpha, A, ykal, Gbeta, C);
        x = x + dt * dyn(x, u);
        vibes::drawAUV(x[0], x[1], x[2]*180./M_PI, 1, "red");
        vibes::drawAUV(x0[0], x0[1], x0[2]*180./M_PI, 2, "green");
        vibes::drawPoint(w[0], w[1], 1, "blue");
        vibes::drawPoint(ykal[0], ykal[1], 1, "black");
        vibes::drawConfidenceEllipse(x0[0], x0[1], Gx0(0, 0), Gx0(1, 0), Gx0(1 ,1), 3.0);
        usleep(40000.);
    }
    vibes::endDrawing();
    return 0;
}
