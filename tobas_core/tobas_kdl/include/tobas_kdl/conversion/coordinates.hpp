#pragma once

#include "../frames.hpp"
#include "../rotationalinertia.hpp"
#include "../euler.hpp"

namespace tobas_kdl
{
void vectorNedToNwu(const Vector& src, Vector& des);
void vectorNwuToNed(const Vector& src, Vector& des);
void vectorNedToNwu(Vector& arg);
void vectorNwuToNed(Vector& arg);

void twistNedToNwu(const Twist& src, Twist& des);
void twistNwuToNed(const Twist& src, Twist& des);
void twistNedToNwu(Twist& arg);
void twistNwuToNed(Twist& arg);

/* 慣性テンソルをNED座標系からNWU座標系に変換する (memo: 2-23)． */
void rotInertiaNedToNwu(const RotationalInertia& src, RotationalInertia& des);
void rotInertiaNwuToNed(const RotationalInertia& src, RotationalInertia& des);
void rotInertiaNedToNwu(RotationalInertia& arg);
void rotInertiaNwuToNed(RotationalInertia& arg);

/* 回転がつなぐ2つのフレームを共にNED->NWUに変換するときのオイラー角の変化を求める． */
void eulerNedToNwu(const Euler& src, Euler& des);
void eulerNwuToNed(const Euler& src, Euler& des);
void eulerNedToNwu(Euler& arg);
void eulerNwuToNed(Euler& arg);

void rotationNedToNwu(const tobas_kdl::Rotation& src, tobas_kdl::Rotation& des);
void rotationNwuToNed(const tobas_kdl::Rotation& src, tobas_kdl::Rotation& des);
void rotationNedToNwu(tobas_kdl::Rotation& arg);
void rotationNwuToNed(tobas_kdl::Rotation& arg);

void frameNedToNwu(const tobas_kdl::Frame& src, tobas_kdl::Frame& des);
void frameNwuToNed(const tobas_kdl::Frame& src, tobas_kdl::Frame& des);
void frameNedToNwu(tobas_kdl::Frame& arg);
void frameNwuToNed(tobas_kdl::Frame& arg);
}  // namespace tobas_kdl
