#pragma once

#include "../frames.hpp"
#include "../rotationalinertia.hpp"
#include "../euler.hpp"

namespace KDL
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

void rotationNedToNwu(const KDL::Rotation& src, KDL::Rotation& des);
void rotationNwuToNed(const KDL::Rotation& src, KDL::Rotation& des);
void rotationNedToNwu(KDL::Rotation& arg);
void rotationNwuToNed(KDL::Rotation& arg);

void frameNedToNwu(const KDL::Frame& src, KDL::Frame& des);
void frameNwuToNed(const KDL::Frame& src, KDL::Frame& des);
void frameNedToNwu(KDL::Frame& arg);
void frameNwuToNed(KDL::Frame& arg);
}  // namespace KDL
