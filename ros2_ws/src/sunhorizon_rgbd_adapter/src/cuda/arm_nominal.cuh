// Runtime candidate arithmetic contract for GCC 11.4 -O3 AArch64, as captured from
// dynamic_self_geometry.o (inverse_rotate @0x0, first_surface @0x54f4).
// CUDA --fmad=false; only the explicitly listed fused operations are fused.
// This does not modify CPU arithmetic or any predicate/tolerance/budget.
// It requires full-frame parity under the verified compiler/object identity.
__device__ Vec3 arm_inverse_rotate(const RigidTransform& t,const Vec3& v) {
  const auto& r=t.rotation;
  // ARM first multiplies the Y term, fuses X into it, then fuses Z.
  return {__fma_rn(r[6],v.z,__fma_rn(r[0],v.x,__dmul_rn(r[3],v.y))),
          __fma_rn(r[7],v.z,__fma_rn(r[1],v.x,__dmul_rn(r[4],v.y))),
          __fma_rn(r[8],v.z,__fma_rn(r[2],v.x,__dmul_rn(r[5],v.y)))};
}
__device__ double arm_nominal_dot(const Vec3& a,const Vec3& b) {
  // ARM nominal triangle dot: multiply X; fuse Y; fuse Z.
  return __fma_rn(a.z,b.z,__fma_rn(a.y,b.y,__dmul_rn(a.x,b.x)));
}
__device__ bool arm_nominal_triangle_hit(const Triangle& triangle,const Vec3& origin,
 const Vec3& direction,bool& hit,double& lambda) {
  const auto& v=triangle.vertices;
  const Vec3 e1=subtract(v[1],v[0]),e2=subtract(v[2],v[0]);
  // The actual ARM nominal-ray p=cross(direction,e2) uses different
  // fusion sides for X versus Y/Z. Global CUDA fmad=true does not specify this.
  const Vec3 p{__fma_rn(direction.y,e2.z,-__dmul_rn(direction.z,e2.y)),
    __fma_rn(-direction.x,e2.z,__dmul_rn(direction.z,e2.x)),
    __fma_rn(-direction.y,e2.x,__dmul_rn(direction.x,e2.y))};
  const double determinant=arm_nominal_dot(e1,p);
  if(!finite(p)||!std::isfinite(determinant))return false;
  if(determinant==0){hit=false;return true;}
  const Vec3 t=subtract(origin,v[0]);
  // q=cross(t,e1): all three ARM components fuse the negative product.
  const Vec3 q{__fma_rn(-e1.y,t.z,__dmul_rn(e1.z,t.y)),
    __fma_rn(-e1.z,t.x,__dmul_rn(e1.x,t.z)),
    __fma_rn(-e1.x,t.y,__dmul_rn(e1.y,t.x))};
  const double u=__ddiv_rn(arm_nominal_dot(t,p),determinant);
  const double w=__ddiv_rn(arm_nominal_dot(direction,q),determinant);
  lambda=__ddiv_rn(arm_nominal_dot(e2,q),determinant);
  if(!finite(t)||!finite(q)||!std::isfinite(u)||!std::isfinite(w)||!std::isfinite(u+w)||!std::isfinite(lambda))return false;
  hit=u>=-kBarycentricRoundoff&&w>=-kBarycentricRoundoff&&u+w<=1.+kBarycentricRoundoff&&lambda>kMinimumPositiveIntersection;
  return true;
}
