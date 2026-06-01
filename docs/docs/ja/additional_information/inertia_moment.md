# 代表的な慣性モーメント

質量$m$の密度均一な物体の重心周りの慣性モーメントの求め方を紹介します．
URDF を作る際にご利用ください．

## 3 辺の長さが$x$，$y$，$z$の直方体

---

$$
\begin{aligned}
I_{xx} &= \frac{m (y^2 + z^2)}{12} \\
I_{yy} &= \frac{m (z^2 + x^2)}{12} \\
I_{zz} &= \frac{m (x^2 + y^2)}{12} \\
I_{xy} &= I_{yz} = I_{zx} = 0 \\
\end{aligned}
$$

```xml
<xacro:macro name="box_inertia" params="x y z m">
  <inertia ixx="${m * (y*y + z*z) / 12}" ixy="0.0" ixz="0.0" iyy="${m * (z*z + x*x) / 12}" iyz="0.0" izz="${m * (x*x + y*y) / 12}"/>
</xacro:macro>
```

## 半径$r$，高さ$h$の円柱

---

$$
\begin{aligned}
I_{xx} &= I_{yy} = m (\frac{r^2}{4} + \frac{h^2}{12}) \\
I_{zz} &= \frac{m r^2}{2} \\
I_{xy} &= I_{yz} = I_{zx} = 0 \\
\end{aligned}
$$

```xml
<xacro:macro name="cylinder_inertia" params="r h m">
  <inertia ixx="${m*r*r/4 + m*h*h/12}" ixy="0.0" ixz="0.0" iyy="${m*r*r/4 + m*h*h/12}" iyz="0.0" izz="${m*r*r/2}"/>
</xacro:macro>
```

## 半径$r$の球

---

$$
\begin{aligned}
I_{xx} &= I_{yy} = I_{zz} = \frac{2}{5} m r^2 \\
I_{xy} &= I_{yz} = I_{zx} = 0 \\
\end{aligned}
$$

```xml
<xacro:macro name="sphere_inertia" params="r m">
  <inertia ixx="${0.4 * m * r*r}" ixy="0.0" ixz="0.0" iyy="${0.4 * m * r*r}" iyz="0.0" izz="${0.4 * m * r*r}"/>
</xacro:macro>
```
