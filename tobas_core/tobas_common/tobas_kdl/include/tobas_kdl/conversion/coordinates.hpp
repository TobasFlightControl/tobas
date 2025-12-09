#pragma once

#include "../euler.hpp"
#include "../frames.hpp"
#include "../rotational_inertia.hpp"

namespace kdl
{
void vectorFrdToFlu(const Vector& src, Vector& des);
void vectorFluToFrd(const Vector& src, Vector& des);
void vectorFrdToFlu(Vector& arg);
void vectorFluToFrd(Vector& arg);

void twistFrdToFlu(const Twist& src, Twist& des);
void twistFluToFrd(const Twist& src, Twist& des);
void twistFrdToFlu(Twist& arg);
void twistFluToFrd(Twist& arg);

/* 慣性テンソルを FRD (Front-Right-Down) 座標系から FLU (Front-Left-Up) 座標系に変換する (memo: 2-23)． */
void rotInertiaFrdToFlu(const RotationalInertia& src, RotationalInertia& des);
void rotInertiaFluToFrd(const RotationalInertia& src, RotationalInertia& des);
void rotInertiaFrdToFlu(RotationalInertia& arg);
void rotInertiaFluToFrd(RotationalInertia& arg);

/* 回転がつなぐ2つのフレームを共にFRD座標系からFLU座標系に変換するときのオイラー角の変化を求める． */
void eulerFrdToFlu(const Euler& src, Euler& des);
void eulerFluToFrd(const Euler& src, Euler& des);
void eulerFrdToFlu(Euler& arg);
void eulerFluToFrd(Euler& arg);

void rotationFrdToFlu(const kdl::Rotation& src, kdl::Rotation& des);
void rotationFluToFrd(const kdl::Rotation& src, kdl::Rotation& des);
void rotationFrdToFlu(kdl::Rotation& arg);
void rotationFluToFrd(kdl::Rotation& arg);

void frameFrdToFlu(const kdl::Frame& src, kdl::Frame& des);
void frameFluToFrd(const kdl::Frame& src, kdl::Frame& des);
void frameFrdToFlu(kdl::Frame& arg);
void frameFluToFrd(kdl::Frame& arg);
}  // namespace kdl
