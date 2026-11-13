#include "sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <map>
#include <stdexcept>
#include <utility>

namespace sunhorizon_obstacle_coverage::dynamic_self_geometry {
namespace {
constexpr std::size_t kMaximumTriangles = 1000000;
constexpr std::size_t kMaximumNodes = 2000000;
constexpr std::size_t kMaximumDepth = 63;
constexpr std::size_t kMaximumLinks = 64;
constexpr double kRigidTolerance = 1e-6;
constexpr double kMinimumPositiveIntersection = 1e-9;
constexpr double kBarycentricRoundoff = 64 * std::numeric_limits<double>::epsilon();
const double kSurfaceBoundsSafeCoordinate=std::sqrt(std::numeric_limits<double>::max())/64.;

bool valid_build_limits(const BuildLimits& limits) {
  return limits.max_triangles>0 && limits.max_triangles<=kMaximumTriangles &&
      limits.max_nodes>0 && limits.max_nodes<=kMaximumNodes &&
      limits.max_depth>0 && limits.max_depth<=kMaximumDepth &&
      limits.leaf_triangles>0 && limits.leaf_triangles<=limits.max_triangles;
}

bool finite(const Vec3& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
Vec3 subtract(const Vec3& a, const Vec3& b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
double dot(const Vec3& a, const Vec3& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
double axis(const Vec3& p, int i) { return i==0 ? p.x : (i==1 ? p.y : p.z); }
Vec3 centroid(const Triangle& t) {
  return {t.vertices[0].x/3+t.vertices[1].x/3+t.vertices[2].x/3,
          t.vertices[0].y/3+t.vertices[1].y/3+t.vertices[2].y/3,
          t.vertices[0].z/3+t.vertices[1].z/3+t.vertices[2].z/3};
}
Vec3 inverse_rotate(const RigidTransform& t, const Vec3& v) {
  const auto& r=t.rotation;
  return {r[0]*v.x+r[3]*v.y+r[6]*v.z,
          r[1]*v.x+r[4]*v.y+r[7]*v.z,
          r[2]*v.x+r[5]*v.y+r[8]*v.z};
}
bool rigid(const RigidTransform& t) {
  if (!finite(t.translation)) return false;
  const auto& r=t.rotation;
  for (double v:r) if (!std::isfinite(v)) return false;
  for (int a=0;a<3;++a) for (int b=0;b<3;++b) {
    double d=0;
    for (int k=0;k<3;++k) d+=r[3*a+k]*r[3*b+k];
    if (!std::isfinite(d) || std::abs(d-(a==b ? 1. : 0.))>kRigidTolerance) return false;
  }
  const double determinant=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
  return std::isfinite(determinant) && std::abs(determinant-1.)<=kRigidTolerance;
}

// False means arithmetic was invalid, rather than a proven miss.
bool box_entry(const Vec3& lo, const Vec3& hi, const Vec3& origin,
               const Vec3& direction, double closest, bool& hit, double& entry) {
  double begin=0, end=closest;
  for (int i=0;i<3;++i) {
    const double o=axis(origin,i),d=axis(direction,i),a=axis(lo,i),b=axis(hi,i);
    if (d==0) { if (o<a || o>b) { hit=false; return true; } continue; }
    double x=(a-o)/d,y=(b-o)/d;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    if (x>y) std::swap(x,y);
    begin=std::max(begin,x);end=std::min(end,y);
    if (begin>end) { hit=false; return true; }
  }
  hit=end>kMinimumPositiveIntersection;entry=begin;
  return true;
}
bool triangle_hit(const Triangle& triangle, const Vec3& origin,
                  const Vec3& direction, bool& hit, double& lambda) {
  const auto& v=triangle.vertices;
  const Vec3 e1=subtract(v[1],v[0]),e2=subtract(v[2],v[0]);
  const Vec3 p=cross(direction,e2);
  const double determinant=dot(e1,p);
  if (!finite(p) || !std::isfinite(determinant)) return false;
  if (determinant==0) { hit=false; return true; }
  const Vec3 t=subtract(origin,v[0]);const Vec3 q=cross(t,e1);
  const double u=dot(t,p)/determinant,w=dot(direction,q)/determinant;
  lambda=dot(e2,q)/determinant;
  if (!finite(t) || !finite(q) || !std::isfinite(u) || !std::isfinite(w) || !std::isfinite(u+w) || !std::isfinite(lambda)) return false;
  hit=u>=-kBarycentricRoundoff && w>=-kBarycentricRoundoff && u+w<=1.+kBarycentricRoundoff && lambda>kMinimumPositiveIntersection;
  return true;
}

// Distance to a finite triangle, including its edges and vertices. In
// particular, a nearby infinite plane extension is never a surface witness.
bool triangle_distance_squared(const Triangle& triangle,const Vec3& point,double& squared) {
  const auto& v=triangle.vertices;
  squared=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<3;++i) {
    const auto edge=subtract(v[(i+1)%3],v[i]),delta=subtract(point,v[i]);
    const double length=dot(edge,edge),along=dot(delta,edge);
    if(!std::isfinite(length)||!(length>0.)||!std::isfinite(along))return false;
    const double fraction=std::clamp(along/length,0.,1.);
    const Vec3 residual{delta.x-fraction*edge.x,delta.y-fraction*edge.y,delta.z-fraction*edge.z};
    const double candidate=dot(residual,residual);
    if(!finite(residual)||!std::isfinite(candidate))return false;
    squared=std::min(squared,candidate);
  }
  const auto normal=cross(subtract(v[1],v[0]),subtract(v[2],v[0]));
  const double area=dot(normal,normal),offset=dot(subtract(point,v[0]),normal);
  if(!finite(normal)||!std::isfinite(area)||!(area>0.)||!std::isfinite(offset))return false;
  const double scale=offset/area;
  const Vec3 projected{point.x-scale*normal.x,point.y-scale*normal.y,point.z-scale*normal.z};
  if(!std::isfinite(scale)||!finite(projected))return false;
  bool inside=true;
  for(std::size_t i=0;i<3;++i) {
    const double side=dot(cross(subtract(v[(i+1)%3],v[i]),subtract(projected,v[i])),normal);
    if(!std::isfinite(side))return false;
    if(side<0.)inside=false;
  }
  if(inside) {
    const auto residual=subtract(point,projected);const double candidate=dot(residual,residual);
    if(!finite(residual)||!std::isfinite(candidate))return false;
    squared=std::min(squared,candidate);
  }
  return std::isfinite(squared);
}
// Only authorized measured facets use this FLOAT32 round-trip predicate.
// It neither changes nominal CAD intersections nor grants millimetre rim coverage.
bool measured_triangle_hit(const Triangle& triangle,const Vec3& origin,const Vec3& direction,
    double bound,const QueryBudget& budget,Result& counters,bool& hit,double& lambda) {
  if(!triangle_hit(triangle,origin,direction,hit,lambda)){counters.reason="measured_triangle_arithmetic";return false;}
  if(hit)return true;
  const double dd=dot(direction,direction),gate=bound*bound;
  if(!std::isfinite(dd)||!(dd>0)){counters.reason="measured_roundoff_arithmetic";return false;}
  double nearest=std::numeric_limits<double>::infinity(),farthest=0.;
  const auto ambiguous=[&](const char* reason,double begin,double end) {
    if(!counters.measured_reference_ambiguous) {
      counters.reason=reason;
    }
    counters.measured_reference_ambiguous=true;
    if(!std::isfinite(counters.ambiguous_min_axial_depth_m)||begin<counters.ambiguous_min_axial_depth_m)
      counters.ambiguous_min_axial_depth_m=begin;
    if(!std::isfinite(counters.ambiguous_max_axial_depth_m)||end>counters.ambiguous_max_axial_depth_m)
      counters.ambiguous_max_axial_depth_m=end;
  };
  for(std::size_t k=0;k<3;++k) {
    if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
    ++counters.triangle_tests;
    const auto a=subtract(triangle.vertices[k],origin),e=subtract(triangle.vertices[(k+1)%3],triangle.vertices[k]);
    const double ad=dot(a,direction)/dd,ed=dot(e,direction)/dd;
    const Vec3 ap{a.x-ad*direction.x,a.y-ad*direction.y,a.z-ad*direction.z};
    const Vec3 ep{e.x-ed*direction.x,e.y-ed*direction.y,e.z-ed*direction.z};
    const double denominator=dot(ep,ep);
    if(!finite(ap)||!finite(ep)||!std::isfinite(denominator)){counters.reason="measured_roundoff_arithmetic";return false;}
    if(denominator==0.) {
      const double begin=std::min(ad,ad+ed),end=std::max(ad,ad+ed);
      if(dot(ap,ap)<=gate&&end>kMinimumPositiveIntersection)
        ambiguous("measured_parallel_range_ambiguous",std::max(begin,kMinimumPositiveIntersection),end);
      continue;
    }
    const double fraction=std::clamp(-dot(ap,ep)/denominator,0.,1.);
    const Vec3 q{a.x+fraction*e.x,a.y+fraction*e.y,a.z+fraction*e.z};const double t=dot(q,direction)/dd;
    const Vec3 miss{q.x-t*direction.x,q.y-t*direction.y,q.z-t*direction.z};const double squared=dot(miss,miss);
    if(!finite(q)||!finite(miss)||!std::isfinite(t)||!std::isfinite(squared)){counters.reason="measured_roundoff_arithmetic";return false;}
    if(t>kMinimumPositiveIntersection&&squared<=gate){nearest=std::min(nearest,t);farthest=std::max(farthest,t);}
  }
  hit=std::isfinite(nearest);
  if(hit) {
    if((farthest-nearest)*std::sqrt(dd)>4*bound) {
      ambiguous("measured_roundoff_range_ambiguous",nearest,farthest);hit=false;
    } else lambda=nearest;
  }
  return true;
}
}  // namespace

double Mesh::origin_radius_bound() const noexcept {
  if(!ready_ || nodes_.empty())return std::numeric_limits<double>::infinity();
  const auto& bounds=nodes_[0].bounds;
  return std::nextafter(std::hypot(std::hypot(
    std::max(std::abs(bounds.minimum.x),std::abs(bounds.maximum.x)),
    std::max(std::abs(bounds.minimum.y),std::abs(bounds.maximum.y))),
    std::max(std::abs(bounds.minimum.z),std::abs(bounds.maximum.z))),
    std::numeric_limits<double>::infinity());
}
void Mesh::invalidate_revision() noexcept {
  if (revision_) revision_=revision_==std::numeric_limits<uint64_t>::max()?0:revision_+1;
}
Mesh::Mesh(Mesh&& other) noexcept : ready_(other.ready_), triangles_(std::move(other.triangles_)),
    order_(std::move(other.order_)), nodes_(std::move(other.nodes_)),
    boundary_edges_(std::move(other.boundary_edges_)), boundary_bounds_(other.boundary_bounds_),
    boundary_first_(other.boundary_first_), boundary_count_(other.boundary_count_), boundary_tolerance_m_(other.boundary_tolerance_m_),
    measured_first_(other.measured_first_),measured_count_(other.measured_count_),measured_bounds_(other.measured_bounds_) {
  other.ready_=false;other.invalidate_revision();
}
Mesh& Mesh::operator=(const Mesh& other) {
  if(this!=&other) {invalidate_revision();ready_=false;
    triangles_=other.triangles_;order_=other.order_;nodes_=other.nodes_;
    boundary_edges_=other.boundary_edges_;boundary_bounds_=other.boundary_bounds_;
    boundary_first_=other.boundary_first_;boundary_count_=other.boundary_count_;boundary_tolerance_m_=other.boundary_tolerance_m_;measured_first_=other.measured_first_;measured_count_=other.measured_count_;measured_bounds_=other.measured_bounds_;ready_=other.ready_;}
  return *this;
}
Mesh& Mesh::operator=(Mesh&& other) noexcept {
  if(this!=&other) {invalidate_revision();ready_=other.ready_;triangles_=std::move(other.triangles_);
    order_=std::move(other.order_);nodes_=std::move(other.nodes_);
    boundary_edges_=std::move(other.boundary_edges_);boundary_bounds_=other.boundary_bounds_;
    boundary_first_=other.boundary_first_;boundary_count_=other.boundary_count_;boundary_tolerance_m_=other.boundary_tolerance_m_;measured_first_=other.measured_first_;measured_count_=other.measured_count_;measured_bounds_=other.measured_bounds_;other.ready_=false;other.invalidate_revision();}
  return *this;
}

bool Mesh::build(const std::vector<Triangle>& triangles, const BuildLimits& limits,
                 std::string* error) {
  invalidate_revision();
  ready_=false;triangles_.clear();order_.clear();nodes_.clear();
  boundary_edges_.clear();boundary_first_=boundary_count_=0;boundary_tolerance_m_=0.;
  measured_first_=measured_count_=0;
  auto fail=[&](const char* why) { if(error)*error=why;return false; };
  if (!valid_build_limits(limits)) return fail("invalid_build_limits");
  if (triangles.empty() || triangles.size()>limits.max_triangles) return fail("triangle_capacity");
  for (const auto& t:triangles) {
    for (const auto& v:t.vertices) if (!finite(v)) return fail("nonfinite_triangle");
    const Vec3 n=cross(subtract(t.vertices[1],t.vertices[0]),subtract(t.vertices[2],t.vertices[0]));
    const double area=dot(n,n);
    if (!finite(n) || !std::isfinite(area) || area==0) return fail("degenerate_or_unbounded_triangle");
    if (!finite(centroid(t))) return fail("nonfinite_centroid");
  }
  try {
    triangles_=triangles;order_.resize(triangles_.size());std::iota(order_.begin(),order_.end(),0);
    nodes_.reserve(std::min(limits.max_nodes,triangles_.size()*2));
    build_node(0,triangles_.size(),1,limits);ready_=true;if(error)error->clear();return true;
  } catch (const std::exception& e) {
    triangles_.clear();order_.clear();nodes_.clear();if(error)*error=e.what();return false;
  }
}

bool Mesh::load_binary_stl(const std::uint8_t* bytes, std::size_t size,
                           std::size_t expected, const BuildLimits& limits,
                           std::string* error) {
  invalidate_revision();
  ready_=false;triangles_.clear();order_.clear();nodes_.clear();
  boundary_edges_.clear();boundary_first_=boundary_count_=0;boundary_tolerance_m_=0.;
  measured_first_=measured_count_=0;
  auto fail=[&](const char* why) {if(error)*error=why;return false;};
  if(!valid_build_limits(limits)) return fail("invalid_build_limits");
  if(!bytes || size<84 || expected==0 || expected>kMaximumTriangles ||
     expected>limits.max_triangles) return fail("stl_capacity_or_header");
  auto u32=[&](std::size_t offset) {
    return std::uint32_t(bytes[offset]) | (std::uint32_t(bytes[offset+1])<<8) |
           (std::uint32_t(bytes[offset+2])<<16) | (std::uint32_t(bytes[offset+3])<<24);
  };
  if(u32(80)!=expected || size!=84+expected*50) return fail("stl_count_or_size");
  static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559,
                "binary STL requires IEEE754 float32");
  try {
    std::vector<Triangle> triangles(expected);
    for(std::size_t i=0;i<expected;++i) for(std::size_t v=0;v<3;++v) {
      double values[3];
      for(std::size_t a=0;a<3;++a) {
        const auto bits=u32(84+i*50+12+v*12+a*4);float value;
        std::memcpy(&value,&bits,sizeof(value));values[a]=value;
      }
      triangles[i].vertices[v]={values[0],values[1],values[2]};
    }
    return build(triangles,limits,error);
  } catch(const std::exception& e) {if(error)*error=e.what();return false;}
}

std::size_t Mesh::build_node(std::size_t begin, std::size_t end,
                             std::size_t depth, const BuildLimits& limits) {
  if (nodes_.size()>=limits.max_nodes || depth>limits.max_depth) throw std::runtime_error("bvh_capacity");
  Node node;node.begin=begin;node.count=end-begin;
  node.bounds.minimum=node.bounds.maximum=triangles_[order_[begin]].vertices[0];
  for(std::size_t i=begin;i<end;++i) for(const auto& p:triangles_[order_[i]].vertices) {
    auto& a=node.bounds.minimum;auto& b=node.bounds.maximum;
    a={std::min(a.x,p.x),std::min(a.y,p.y),std::min(a.z,p.z)};
    b={std::max(b.x,p.x),std::max(b.y,p.y),std::max(b.z,p.z)};
  }
  const auto index=nodes_.size();nodes_.push_back(node);
  if (end-begin<=limits.leaf_triangles) return index;
  const Vec3 extent=subtract(node.bounds.maximum,node.bounds.minimum);
  if (!finite(extent)) throw std::runtime_error("nonfinite_bvh_extent");
  int split=extent.y>extent.x ? 1 : 0;if(extent.z>axis(extent,split))split=2;
  const auto middle=begin+(end-begin)/2;
  std::nth_element(order_.begin()+begin,order_.begin()+middle,order_.begin()+end,
    [&](std::size_t a,std::size_t b) {return axis(centroid(triangles_[a]),split)<axis(centroid(triangles_[b]),split);});
  const auto left=build_node(begin,middle,depth+1,limits),right=build_node(middle,end,depth+1,limits);
  nodes_[index].left=left;nodes_[index].right=right;nodes_[index].count=0;
  return index;
}

bool Mesh::configure_boundary_patch(std::size_t first,std::size_t count,double tolerance,std::string* error) {
  invalidate_revision();boundary_edges_.clear();boundary_first_=boundary_count_=0;boundary_tolerance_m_=0.;
  const auto fail=[&](const char* why){if(error)*error=why;return false;};
  if(!ready_ || !std::isfinite(tolerance) || tolerance<0. || tolerance>.003 ||
      first>triangles_.size() || count>triangles_.size()-first ||
      (!count && (first || tolerance!=0.)) || (count && !(tolerance>0.)))
    return fail("invalid_boundary_patch_authorization");
  if(!count)return true;
  try {
    using Point=std::array<double,3>;using Key=std::pair<Point,Point>;
    std::map<Key,std::pair<std::size_t,std::size_t>> edges;
    for(std::size_t i=first;i<first+count;++i)for(std::size_t j=0;j<3;++j) {
      const auto a=triangles_[i].vertices[j],b=triangles_[i].vertices[(j+1)%3];
      Point x{a.x,a.y,a.z},y{b.x,b.y,b.z};if(y<x)std::swap(x,y);
      auto& item=edges[{x,y}];if(++item.first>2)return fail("boundary_patch_nonmanifold");item.second=i;
    }
    for(const auto& entry:edges)if(entry.second.first==1) {
      const auto& a=entry.first.first;const auto& b=entry.first.second;
      boundary_edges_.push_back({{a[0],a[1],a[2]},{b[0],b[1],b[2]},entry.second.second});
    }
    if(boundary_edges_.empty())return fail("boundary_patch_has_no_open_rim");
    boundary_bounds_.minimum=boundary_bounds_.maximum=boundary_edges_[0].a;
    for(const auto& edge:boundary_edges_)for(const auto p:{edge.a,edge.b}) {
      auto& a=boundary_bounds_.minimum;auto& b=boundary_bounds_.maximum;
      a={std::min(a.x,p.x),std::min(a.y,p.y),std::min(a.z,p.z)};
      b={std::max(b.x,p.x),std::max(b.y,p.y),std::max(b.z,p.z)};
    }
    auto& a=boundary_bounds_.minimum;auto& b=boundary_bounds_.maximum;
    const auto infinity=std::numeric_limits<double>::infinity();
    a={std::nextafter(a.x-tolerance,-infinity),std::nextafter(a.y-tolerance,-infinity),std::nextafter(a.z-tolerance,-infinity)};
    b={std::nextafter(b.x+tolerance,infinity),std::nextafter(b.y+tolerance,infinity),std::nextafter(b.z+tolerance,infinity)};
    if(!finite(a)||!finite(b))return fail("boundary_bounds_arithmetic");
    boundary_first_=first;boundary_count_=count;boundary_tolerance_m_=tolerance;return true;
  } catch(const std::exception& e) {boundary_edges_.clear();if(error)*error=e.what();return false;}
}

bool Mesh::configure_measured_surface(std::size_t first,std::size_t count,std::string* error) {
  invalidate_revision();measured_first_=measured_count_=0;
  if(!ready_ || first>triangles_.size() || count>triangles_.size()-first || (!count && first)) {
    ready_=false;if(error)*error="invalid_measured_surface_authorization";return false;
  }
  if(!count){if(error)error->clear();return true;}
  measured_bounds_.minimum=measured_bounds_.maximum=triangles_[first].vertices[0];
  for(std::size_t i=first;i<first+count;++i)for(const auto& p:triangles_[i].vertices) {
    auto& a=measured_bounds_.minimum;auto& b=measured_bounds_.maximum;
    a={std::min(a.x,p.x),std::min(a.y,p.y),std::min(a.z,p.z)};
    b={std::max(b.x,p.x),std::max(b.y,p.y),std::max(b.z,p.z)};
  }
  measured_first_=first;measured_count_=count;if(error)error->clear();return true;
}

Mesh::MeasuredQuery Mesh::prepare_measured_query(const Vec3& origin) const {
  MeasuredQuery result;
  if(!measured_count_)return result;
  const auto& lo=measured_bounds_.minimum;const auto& hi=measured_bounds_.maximum;
  const double radius=std::hypot(std::hypot(std::max(std::abs(lo.x),std::abs(hi.x)),
      std::max(std::abs(lo.y),std::abs(hi.y))),std::max(std::abs(lo.z),std::abs(hi.z)));
  const double scale=std::hypot(std::hypot(origin.x,origin.y),origin.z)+radius;
  // 2 float eps covers vertex quantization plus FLOAT32 x/z,y/z ray
  // reconstruction, under a rigid rotation. Double arithmetic gets 64 eps.
  const double roundoff=2*std::numeric_limits<float>::epsilon()*scale+
      64*std::numeric_limits<double>::epsilon()*std::max(1.,scale);
  result.roundoff=roundoff;
  result.valid=std::isfinite(roundoff)&&roundoff<=1.e-6;
  if(!result.valid)return result;
  const double inf=std::numeric_limits<double>::infinity();
  const auto& b=measured_bounds_;
  result.root.minimum={std::nextafter(b.minimum.x-roundoff,-inf),std::nextafter(b.minimum.y-roundoff,-inf),std::nextafter(b.minimum.z-roundoff,-inf)};
  result.root.maximum={std::nextafter(b.maximum.x+roundoff,inf),std::nextafter(b.maximum.y+roundoff,inf),std::nextafter(b.maximum.z+roundoff,inf)};
  return result;
}

bool Mesh::measured_surface(const Vec3& origin,const Vec3& direction,
    const QueryBudget& budget,Result& counters,double& closest,std::size_t& triangle,
    const MeasuredQuery* cached) const {
  if(!measured_count_)return true;
  const auto local=cached ? MeasuredQuery{} : prepare_measured_query(origin);
  const auto& preparation=cached ? *cached : local;
  const double roundoff=preparation.roundoff;
  if(!preparation.valid){counters.reason="measured_roundoff_bound";return false;}
  const auto expanded=[&](const Bounds& b){
    const double inf=std::numeric_limits<double>::infinity();Bounds out=b;
    out.minimum={std::nextafter(b.minimum.x-roundoff,-inf),std::nextafter(b.minimum.y-roundoff,-inf),std::nextafter(b.minimum.z-roundoff,-inf)};
    out.maximum={std::nextafter(b.maximum.x+roundoff,inf),std::nextafter(b.maximum.y+roundoff,inf),std::nextafter(b.maximum.z+roundoff,inf)};
    return out;
  };
  const auto& measured_box=preparation.root;
  if(counters.node_visits>=budget.max_node_visits){counters.reason="node_budget";return false;}
  ++counters.node_visits;bool hit=false;double entry=0.;
  if(!box_entry(measured_box.minimum,measured_box.maximum,origin,direction,std::numeric_limits<double>::infinity(),hit,entry)) {
    counters.reason="measured_box_arithmetic";return false;
  }
  if(!hit)return true;
  std::array<std::size_t,kMaximumDepth+1> stack;std::size_t size=1;stack[0]=0;
  while(size) {
    if(counters.node_visits>=budget.max_node_visits){counters.reason="node_budget";return false;}
    const auto& node=nodes_[stack[--size]];++counters.node_visits;
    const auto box=expanded(node.bounds);
    if(!box_entry(box.minimum,box.maximum,origin,direction,std::numeric_limits<double>::infinity(),hit,entry)) {
      counters.reason="measured_box_arithmetic";return false;
    }
    if(!hit)continue;
    if(node.count) {
      for(std::size_t k=node.begin;k<node.begin+node.count;++k) {
        const auto i=order_[k];if(i<measured_first_ || i-measured_first_>=measured_count_)continue;
        if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
        ++counters.triangle_tests;double lambda=0.;bool seen=false;
        const double previous_min=counters.ambiguous_min_axial_depth_m;
        if(!measured_triangle_hit(triangles_[i],origin,direction,roundoff,budget,counters,seen,lambda))return false;
        if(seen) {
          if(!std::isfinite(counters.ambiguous_min_axial_depth_m)||lambda<counters.ambiguous_min_axial_depth_m)
            counters.ambiguous_min_axial_depth_m=lambda;
          if(!std::isfinite(counters.ambiguous_max_axial_depth_m)||lambda>counters.ambiguous_max_axial_depth_m)
            counters.ambiguous_max_axial_depth_m=lambda;
          if(lambda<closest){closest=lambda;triangle=i;}
        }
        if(std::isfinite(counters.ambiguous_min_axial_depth_m)&&
            (!std::isfinite(previous_min)||counters.ambiguous_min_axial_depth_m<previous_min))
          counters.ambiguous_measured_triangle_index=i;
      }
    } else {
      if(size+2>stack.size()){counters.reason="stack_budget";return false;}
      stack[size++]=node.right;stack[size++]=node.left;
    }
  }
  return true;
}

bool Mesh::pixel_surface(const NativePixelSupport& pixel,const RigidTransform& tf,
    const QueryBudget& budget,Result& counters,double& lower,double& upper,
    std::size_t& triangle) const {
  if(!measured_count_)return true;
  const std::array<Vec3,5> normals{{{1,0,-(double(pixel.u)-.5-pixel.cx)/pixel.fx},
    {-1,0,(double(pixel.u)+.5-pixel.cx)/pixel.fx},
    {0,1,-(double(pixel.v)-.5-pixel.cy)/pixel.fy},
    {0,-1,(double(pixel.v)+.5-pixel.cy)/pixel.fy},{0,0,1}}};
  std::array<Vec3,5> local;std::array<double,5> offsets;
  for(std::size_t i=0;i<normals.size();++i) {
    local[i]=inverse_rotate(tf,normals[i]);offsets[i]=dot(normals[i],tf.translation)-(i==4?kMinimumPositiveIntersection:0.);
    if(!finite(local[i])||!std::isfinite(offsets[i])){counters.reason="pixel_plane_arithmetic";return false;}
  }
  // These are DOUBLE arithmetic error bounds, not a physical cone/rim expansion.
  // The same conservative predicate is used by the BVH and finite polygon clip.
  std::array<double,5> plane_error;
  const auto& bounds=nodes_[0].bounds;
  const Vec3 extent{std::max(std::abs(bounds.minimum.x),std::abs(bounds.maximum.x)),
    std::max(std::abs(bounds.minimum.y),std::abs(bounds.maximum.y)),
    std::max(std::abs(bounds.minimum.z),std::abs(bounds.maximum.z))};
  for(std::size_t i=0;i<local.size();++i) {
    plane_error[i]=64*std::numeric_limits<double>::epsilon()*(1+std::abs(local[i].x)*extent.x+
      std::abs(local[i].y)*extent.y+std::abs(local[i].z)*extent.z+std::abs(offsets[i]));
    if(!std::isfinite(plane_error[i])){counters.reason="pixel_plane_arithmetic";return false;}
  }
  const auto outside=[&](const Bounds& bounds) {
    for(std::size_t i=0;i<local.size();++i) {
      const auto& n=local[i];const Vec3 p{n.x>=0?bounds.maximum.x:bounds.minimum.x,
        n.y>=0?bounds.maximum.y:bounds.minimum.y,n.z>=0?bounds.maximum.z:bounds.minimum.z};
      const double value=dot(n,p)+offsets[i];
      if(!std::isfinite(value))return -1;
      if(value< -plane_error[i])return 1;
    }
    return 0;
  };
  if(counters.node_visits>=budget.max_node_visits){counters.reason="node_budget";return false;}
  ++counters.node_visits;const int root=outside(measured_bounds_);
  if(root<0){counters.reason="pixel_box_arithmetic";return false;}if(root)return true;
  std::array<std::size_t,kMaximumDepth+1> stack;std::size_t size=1;stack[0]=0;
  while(size) {
    if(counters.node_visits>=budget.max_node_visits){counters.reason="node_budget";return false;}
    ++counters.node_visits;const auto& node=nodes_[stack[--size]];const int excluded=outside(node.bounds);
    if(excluded<0){counters.reason="pixel_box_arithmetic";return false;}if(excluded)continue;
    if(!node.count) {
      if(size+2>stack.size()){counters.reason="stack_budget";return false;}
      stack[size++]=node.right;stack[size++]=node.left;continue;
    }
    for(std::size_t k=node.begin;k<node.begin+node.count;++k) {
      const auto index=order_[k];if(index<measured_first_||index-measured_first_>=measured_count_)continue;
      if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
      ++counters.triangle_tests;
      std::array<Vec3,16> polygon,next;std::size_t count=3;
      for(std::size_t j=0;j<3;++j)polygon[j]=triangles_[index].vertices[j];
      for(std::size_t i=0;i<local.size()&&count;++i) {
        std::size_t output=0;
        const auto append=[&](const Vec3& p){
          if(!finite(p)||output>=next.size())return false;
          if(output&&p.x==next[output-1].x&&p.y==next[output-1].y&&p.z==next[output-1].z)return true;
          next[output++]=p;return true;
        };
        for(std::size_t j=0;j<count;++j) {
          // Clip-edge primitives consume the existing bounded surface budget.
          if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
          ++counters.triangle_tests;
          const auto& a=polygon[j];const auto& b=polygon[(j+1)%count];
          const double da=dot(local[i],a)+offsets[i]+plane_error[i];
          const double db=dot(local[i],b)+offsets[i]+plane_error[i];
          if(!std::isfinite(da)||!std::isfinite(db)){counters.reason="pixel_clip_arithmetic";return false;}
          if(da>=0&&!append(a)){counters.reason="pixel_clip_capacity";return false;}
          if((da>=0)!=(db>=0)) {
            const double fraction=da/(da-db);const auto d=subtract(b,a);
            if(!std::isfinite(fraction)||fraction<0||fraction>1||
              !append({a.x+fraction*d.x,a.y+fraction*d.y,a.z+fraction*d.z})) {
              counters.reason="pixel_clip_arithmetic";return false;
            }
          }
        }
        if(output>1&&next[0].x==next[output-1].x&&next[0].y==next[output-1].y&&next[0].z==next[output-1].z)--output;
        polygon=next;count=output;
      }
      for(std::size_t j=0;j<count;++j) {
        const auto& p=polygon[j];const auto& r=tf.rotation;
        const double z=r[6]*p.x+r[7]*p.y+r[8]*p.z+tf.translation.z;
        const double error=64*std::numeric_limits<double>::epsilon()*(1+std::abs(r[6]*p.x)+
          std::abs(r[7]*p.y)+std::abs(r[8]*p.z)+std::abs(tf.translation.z));
        if(!std::isfinite(z)||!std::isfinite(error)){counters.reason="pixel_range_arithmetic";return false;}
        const double lo=std::max(kMinimumPositiveIntersection,std::nextafter(z-error,-INFINITY));
        const double hi=std::nextafter(z+error,INFINITY);
        if(lo<lower){lower=lo;triangle=index;}upper=std::max(upper,hi);
      }
    }
  }
  return true;
}

bool Mesh::projection_edge_surface(const NativePixelSupport& pixel,const RigidTransform& tf,
    const QueryBudget& budget,Result& counters,double& upper) const {
  const std::array<Vec3,5> normals{{{1,0,-(double(pixel.u)-2.-pixel.cx)/pixel.fx},
    {-1,0,(double(pixel.u)+2.-pixel.cx)/pixel.fx},
    {0,1,-(double(pixel.v)-2.-pixel.cy)/pixel.fy},
    {0,-1,(double(pixel.v)+2.-pixel.cy)/pixel.fy},{0,0,1}}};
  std::array<Vec3,5> local;std::array<double,5> offsets;
  for(std::size_t i=0;i<normals.size();++i) {
    local[i]=inverse_rotate(tf,normals[i]);offsets[i]=dot(normals[i],tf.translation)-(i==4?kMinimumPositiveIntersection:0.);
    if(!finite(local[i])||!std::isfinite(offsets[i])){counters.reason="pixel_plane_arithmetic";return false;}
  }
  // These are DOUBLE arithmetic error bounds, not a physical cone/rim expansion.
  // The same conservative predicate is used by the BVH and finite polygon clip.
  std::array<double,5> plane_error;
  const auto& bounds=nodes_[0].bounds;
  const Vec3 extent{std::max(std::abs(bounds.minimum.x),std::abs(bounds.maximum.x)),
    std::max(std::abs(bounds.minimum.y),std::abs(bounds.maximum.y)),
    std::max(std::abs(bounds.minimum.z),std::abs(bounds.maximum.z))};
  for(std::size_t i=0;i<local.size();++i) {
    plane_error[i]=64*std::numeric_limits<double>::epsilon()*(1+std::abs(local[i].x)*extent.x+
      std::abs(local[i].y)*extent.y+std::abs(local[i].z)*extent.z+std::abs(offsets[i]));
    if(!std::isfinite(plane_error[i])){counters.reason="pixel_plane_arithmetic";return false;}
  }
  const auto outside=[&](const Bounds& bounds) {
    for(std::size_t i=0;i<local.size();++i) {
      const auto& n=local[i];const Vec3 p{n.x>=0?bounds.maximum.x:bounds.minimum.x,
        n.y>=0?bounds.maximum.y:bounds.minimum.y,n.z>=0?bounds.maximum.z:bounds.minimum.z};
      const double value=dot(n,p)+offsets[i];
      if(!std::isfinite(value))return -1;
      if(value< -plane_error[i])return 1;
    }
    return 0;
  };
  if(counters.node_visits>=budget.max_node_visits){counters.reason="node_budget";return false;}
  ++counters.node_visits;const int root=outside(nodes_[0].bounds);
  if(root<0){counters.reason="pixel_box_arithmetic";return false;}if(root)return true;
  std::array<std::size_t,kMaximumDepth+1> stack;std::size_t size=1;stack[0]=0;
  while(size) {
    if(counters.node_visits>=budget.max_node_visits){counters.reason="node_budget";return false;}
    ++counters.node_visits;const auto& node=nodes_[stack[--size]];const int excluded=outside(node.bounds);
    if(excluded<0){counters.reason="pixel_box_arithmetic";return false;}if(excluded)continue;
    if(!node.count) {
      if(size+2>stack.size()){counters.reason="stack_budget";return false;}
      stack[size++]=node.right;stack[size++]=node.left;continue;
    }
    for(std::size_t k=node.begin;k<node.begin+node.count;++k) {
      const auto index=order_[k];
      if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
      ++counters.triangle_tests;
      std::array<Vec3,16> polygon,next;std::size_t count=3;
      for(std::size_t j=0;j<3;++j)polygon[j]=triangles_[index].vertices[j];
      for(std::size_t i=0;i<local.size()&&count;++i) {
        std::size_t output=0;
        const auto append=[&](const Vec3& p){
          if(!finite(p)||output>=next.size())return false;
          if(output&&p.x==next[output-1].x&&p.y==next[output-1].y&&p.z==next[output-1].z)return true;
          next[output++]=p;return true;
        };
        for(std::size_t j=0;j<count;++j) {
          // Clip-edge primitives consume the existing bounded surface budget.
          if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
          ++counters.triangle_tests;
          const auto& a=polygon[j];const auto& b=polygon[(j+1)%count];
          const double da=dot(local[i],a)+offsets[i]+plane_error[i];
          const double db=dot(local[i],b)+offsets[i]+plane_error[i];
          if(!std::isfinite(da)||!std::isfinite(db)){counters.reason="pixel_clip_arithmetic";return false;}
          if(da>=0&&!append(a)){counters.reason="pixel_clip_capacity";return false;}
          if((da>=0)!=(db>=0)) {
            const double fraction=da/(da-db);const auto d=subtract(b,a);
            if(!std::isfinite(fraction)||fraction<0||fraction>1||
              !append({a.x+fraction*d.x,a.y+fraction*d.y,a.z+fraction*d.z})) {
              counters.reason="pixel_clip_arithmetic";return false;
            }
          }
        }
        if(output>1&&next[0].x==next[output-1].x&&next[0].y==next[output-1].y&&next[0].z==next[output-1].z)--output;
        polygon=next;count=output;
      }
      // The square above is only BVH/clip broad phase. The actual policy is
      // a radius-2 Euclidean disk about the native pixel CENTER (no +.5).
      std::array<Vec3,16> optical;
      bool positive=false,negative=false;double nearest_squared=INFINITY;
      for(std::size_t j=0;j<count;++j) {
        const auto& p=polygon[j];const auto& r=tf.rotation;
        optical[j]={r[0]*p.x+r[1]*p.y+r[2]*p.z+tf.translation.x,
          r[3]*p.x+r[4]*p.y+r[5]*p.z+tf.translation.y,
          r[6]*p.x+r[7]*p.y+r[8]*p.z+tf.translation.z};
        if(!finite(optical[j])||!(optical[j].z>0.)) {
          counters.reason="edge_projection_arithmetic";return false;
        }
      }
      for(std::size_t j=0;j<count;++j) {
        if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
        ++counters.triangle_tests;
        const auto& a=optical[j];const auto& b=optical[(j+1)%count];
        const double ax=pixel.fx*a.x/a.z+pixel.cx-double(pixel.u);
        const double ay=pixel.fy*a.y/a.z+pixel.cy-double(pixel.v);
        const double bx=pixel.fx*b.x/b.z+pixel.cx-double(pixel.u);
        const double by=pixel.fy*b.y/b.z+pixel.cy-double(pixel.v);
        const double dx=bx-ax,dy=by-ay,den=dx*dx+dy*dy,cross=ax*by-ay*bx;
        const double t=den>0.?std::clamp(-(ax*dx+ay*dy)/den,0.,1.):0.;
        const double x=ax+t*dx,y=ay+t*dy,squared=x*x+y*y;
        if(!std::isfinite(squared)||!std::isfinite(cross)||!std::isfinite(den)) {
          counters.reason="edge_projection_arithmetic";return false;
        }
        positive|=cross>0.;negative|=cross<0.;nearest_squared=std::min(nearest_squared,squared);
      }
      const bool contains_center=count>=3&&(positive!=negative);
      if(!contains_center&&nearest_squared>4.)continue;
      // The clipped-square envelope encloses this triangle's disk intersection;
      // using its upper endpoint cannot falsely discard a nearer foreground.
      for(std::size_t j=0;j<count;++j) {
        const auto& p=polygon[j];const auto& r=tf.rotation;
        const double z=r[6]*p.x+r[7]*p.y+r[8]*p.z+tf.translation.z;
        const double error=64*std::numeric_limits<double>::epsilon()*(1+std::abs(r[6]*p.x)+
          std::abs(r[7]*p.y)+std::abs(r[8]*p.z)+std::abs(tf.translation.z));
        if(!std::isfinite(z)||!std::isfinite(error)){counters.reason="pixel_range_arithmetic";return false;}
        const double lo=std::max(kMinimumPositiveIntersection,std::nextafter(z-error,-INFINITY));
        const double hi=std::nextafter(z+error,INFINITY);
        (void)lo;upper=std::max(upper,hi);
      }
    }
  }
  return true;
}

bool Mesh::within_surface(const Vec3& point,double radius,const QueryBudget& budget,
    Result& counters,bool& found,double& distance,std::size_t& triangle) const {
  found=false;
  std::array<std::size_t,kMaximumDepth+1> stack{};std::size_t count=1;stack[0]=0;
  const double gate=radius*radius;
  while(count) {
    if(counters.node_visits>=budget.max_node_visits){counters.reason="node_budget";return false;}
    ++counters.node_visits;const auto& node=nodes_[stack[--count]];
    double lower=0.;
    for(int i=0;i<3;++i) {
      const double delta=std::max({axis(node.bounds.minimum,i)-axis(point,i),0.,
          axis(point,i)-axis(node.bounds.maximum,i)});
      lower+=delta*delta;
    }
    if(!std::isfinite(lower)){counters.reason="surface_box_arithmetic";return false;}
    if(lower>gate)continue;
    if(node.left || node.right) {
      if(count+2>stack.size()){counters.reason="surface_stack_capacity";return false;}
      stack[count++]=node.right;stack[count++]=node.left;continue;
    }
    for(std::size_t k=node.begin;k<node.begin+node.count;++k) {
      if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
      ++counters.triangle_tests;const auto index=order_[k];double squared=0.;
      if(!triangle_distance_squared(triangles_[index],point,squared)) {
        counters.reason="surface_triangle_arithmetic";return false;
      }
      if(squared<=gate) {found=true;distance=std::sqrt(squared);triangle=index;return true;}
    }
  }
  return true;
}

Result match_finite_surface(const Vec3& point,double tolerance,const PreparedModel& model,Result result) {
  if(!result.complete || result.classification==Classification::Invalid)return result;
  const auto fail=[&](const char* reason) {
    result.complete=false;result.classification=Classification::Unobserved;result.reason=reason;return result;
  };
  if(!finite(point)||!std::isfinite(tolerance)||tolerance<0.)return fail("invalid_surface_point_or_tolerance");
  if(!model.ready())return fail(model.reason_);
  for(const auto& link:model.links_)if(!link.mesh->ready_||link.mesh->revision_!=link.revision)
    return fail("model_identity_or_source_tf_invalid");
  const double radius=tolerance;
  // An axial/pixel match alone cannot bypass the same metric surface bound.
  if(result.classification==Classification::SelfSurface) {
    result.classification=Classification::Unobserved;
    result.reason="outside_finite_surface_tolerance";
  }
  if(model.surface_bounds_valid_ && std::abs(point.x)<kSurfaceBoundsSafeCoordinate &&
      std::abs(point.y)<kSurfaceBoundsSafeCoordinate && std::abs(point.z)<kSurfaceBoundsSafeCoordinate) {
    const double expansion=tolerance*(1.+64.e-12)+model.surface_bounds_padding_;
    bool outside=false;
    for(int a=0;a<3;++a)outside|=axis(point,a)<axis(model.surface_bounds_min_,a)-expansion ||
        axis(point,a)>axis(model.surface_bounds_max_,a)+expansion;
    if(outside) {
      // The original traversal would visit exactly each link's root, then
      // reject it. Retain that accounting/failure boundary while omitting
      // twelve inverse transforms and twelve local distance tests.
      for(std::size_t i=0;i<model.links_.size();++i) {
        if(result.node_visits>=model.budget_.max_node_visits)return fail("node_budget");
        ++result.node_visits;
      }
      return result;
    }
  }
  for(std::size_t i=0;i<model.links_.size();++i) {
    const auto& link=model.links_[i];
    const Vec3 local=inverse_rotate(link.optical_from_link,subtract(point,link.optical_from_link.translation));
    if(!finite(local))return fail("nonfinite_local_surface_point");
    bool found=false;double distance=0.;std::size_t triangle=0;
    if(!link.mesh->within_surface(local,radius,model.budget_,result,found,distance,triangle))
      return fail(result.reason);
    if(found) {
      result.classification=Classification::SelfSurface;result.reason="matches_finite_model_surface";
      result.finite_surface_distance_m=distance;result.finite_surface_link_index=i;
      result.finite_surface_triangle_index=triangle;return result;
    }
  }
  return result;
}

bool PreparedModel::excludes_near_body_map_mark(const Vec3& point) const {
  Result query;
  query.complete=true;  // Standalone endpoint query, without a ray classification.
  const auto witness=match_finite_surface(point,0.15,*this,query);
  return witness.complete && witness.classification==Classification::SelfSurface;
}

bool Mesh::near_boundary(const Vec3& origin,const Vec3& direction,
    const QueryBudget& budget,Result& counters,double& best_squared,
    double& expected_depth,std::size_t& triangle) const {
  if(!(boundary_tolerance_m_>0.))return true;
  if(counters.node_visits>=budget.max_node_visits){counters.reason="node_budget";return false;}
  ++counters.node_visits;bool hit=false;double entry=0.;
  if(!box_entry(boundary_bounds_.minimum,boundary_bounds_.maximum,origin,direction,
      std::numeric_limits<double>::infinity(),hit,entry)){counters.reason="boundary_box_arithmetic";return false;}
  if(!hit)return true;
  const double dd=dot(direction,direction),gate=boundary_tolerance_m_*boundary_tolerance_m_;
  if(!std::isfinite(dd)||!(dd>0.)){counters.reason="boundary_ray_arithmetic";return false;}
  for(const auto& edge:boundary_edges_) {
    // The existing surface primitive budget also charges each finite rim edge.
    if(counters.triangle_tests>=budget.max_triangle_tests){counters.reason="triangle_budget";return false;}
    ++counters.triangle_tests;
    const auto a=subtract(edge.a,origin),e=subtract(edge.b,edge.a);
    const double ad=dot(a,direction)/dd,ed=dot(e,direction)/dd;
    const Vec3 ap{a.x-ad*direction.x,a.y-ad*direction.y,a.z-ad*direction.z};
    const Vec3 ep{e.x-ed*direction.x,e.y-ed*direction.y,e.z-ed*direction.z};
    const double denominator=dot(ep,ep);
    if(!finite(ap)||!finite(ep)||!std::isfinite(denominator)){counters.reason="boundary_edge_arithmetic";return false;}
    if(denominator==0.) {
      if(dot(ap,ap)<=gate){counters.reason="boundary_parallel_range_ambiguous";return false;}
      continue;
    }
    const double s=std::clamp(-dot(ap,ep)/denominator,0.,1.);
    const Vec3 q{a.x+s*e.x,a.y+s*e.y,a.z+s*e.z};const double t=dot(q,direction)/dd;
    const Vec3 miss{q.x-t*direction.x,q.y-t*direction.y,q.z-t*direction.z};const double squared=dot(miss,miss);
    if(!finite(q)||!finite(miss)||!std::isfinite(t)||!std::isfinite(squared)){counters.reason="boundary_edge_arithmetic";return false;}
    if(t>kMinimumPositiveIntersection && squared<=gate && squared<best_squared) {
      best_squared=squared;expected_depth=t;triangle=edge.triangle;
    }
  }
  return true;
}

bool Mesh::first_surface(const Vec3& origin, const Vec3& direction,
                         const QueryBudget& budget, Result& counters,
                         double& closest, std::size_t& triangle) const {
  // DFS on a median-split tree needs at most max_depth pending siblings.
  // Every slot is assigned before it is read: root below, descendants on push.
  std::array<std::size_t,kMaximumDepth+1> stack;std::size_t size=1;stack[0]=0;
  while(size) {
    if (counters.node_visits>=budget.max_node_visits) {counters.reason="node_budget";return false;}
    const auto& node=nodes_[stack[--size]];++counters.node_visits;
    bool hit=false;double entry=0;
    if(!box_entry(node.bounds.minimum,node.bounds.maximum,origin,direction,closest,hit,entry)) {counters.reason="box_arithmetic";return false;}
    if(!hit)continue;
    if(node.count) {
      for(std::size_t i=node.begin;i<node.begin+node.count;++i) {
        if(counters.triangle_tests>=budget.max_triangle_tests) {counters.reason="triangle_budget";return false;}
        ++counters.triangle_tests;double lambda=0;bool triangle_hit_seen=false;
        if(!triangle_hit(triangles_[order_[i]],origin,direction,triangle_hit_seen,lambda)) {counters.reason="triangle_arithmetic";return false;}
        if(triangle_hit_seen && lambda<closest) {closest=lambda;triangle=order_[i];}
      }
    } else {
      if(size+2>stack.size()) {counters.reason="stack_budget";return false;}
      // No allocation or traversal recursion during a query. Both siblings
      // must be completed; the nearest hit never licenses a partial result.
      stack[size++]=node.right;stack[size++]=node.left;
    }
  }
  return true;
}

Result classify(const Vec3& ray, double measured, double tolerance,
    const std::vector<LinkObservation>& links,const QueryBudget& budget) {
  return classify(ray,measured,tolerance,PreparedModel(links,budget));
}

PreparedModel::PreparedModel(const std::vector<LinkObservation>& links, const QueryBudget& budget)
    : budget_(budget) {
  if(links.empty() || budget.max_links>kMaximumLinks || links.size()>budget.max_links ||
      budget.max_node_visits==0 || budget.max_node_visits>kMaximumNodes ||
      budget.max_triangle_tests==0 || budget.max_triangle_tests>kMaximumTriangles) return;
  for(const auto& link:links) {
    if(!link.mesh || !link.mesh->ready_ || !link.mesh->revision_ || link.link_name.empty() || link.link_name.size()>128 ||
        !link.identity_valid || !link.tf_valid || !link.covers_source_time) {reason_="model_identity_or_source_tf_invalid";return;}
    if(!rigid(link.optical_from_link)) {reason_="invalid_rigid_transform";return;}
  }
  for(std::size_t i=0;i<links.size();++i) for(std::size_t j=0;j<i;++j)
    if(links[i].link_name==links[j].link_name) {reason_="duplicate_link_identity";return;}
  try {
    links_.reserve(links.size());
    for(const auto& link:links) {
      const auto& p=link.optical_from_link.translation;
      const auto origin=inverse_rotate(link.optical_from_link,{-p.x,-p.y,-p.z});
      links_.push_back({link.mesh,link.mesh->revision_,link.optical_from_link,
          origin,link.mesh->prepare_measured_query(origin)});
    }
    // Cache only high-precision rigid observations. A loose-but-valid input
    // keeps the original query path rather than assuming exact orthogonality.
    surface_bounds_min_={INFINITY,INFINITY,INFINITY};
    surface_bounds_max_={-INFINITY,-INFINITY,-INFINITY};
    surface_bounds_valid_=true;
    for(const auto& link:links_) {
      const auto& r=link.optical_from_link.rotation;
      for(int a=0;a<3;++a)for(int b=0;b<3;++b) {
        double d=0.;for(int k=0;k<3;++k)d+=r[3*k+a]*r[3*k+b];
        if(std::abs(d-(a==b?1.:0.))>1.e-12)surface_bounds_valid_=false;
      }
      const auto& bounds=link.mesh->nodes_[0].bounds;
      for(unsigned mask=0;mask<8;++mask) {
        const Vec3 p{mask&1?bounds.maximum.x:bounds.minimum.x,
          mask&2?bounds.maximum.y:bounds.minimum.y,
          mask&4?bounds.maximum.z:bounds.minimum.z};
        const auto& t=link.optical_from_link.translation;
        const Vec3 q{r[0]*p.x+r[1]*p.y+r[2]*p.z+t.x,
          r[3]*p.x+r[4]*p.y+r[5]*p.z+t.y,
          r[6]*p.x+r[7]*p.y+r[8]*p.z+t.z};
        if(!finite(q))surface_bounds_valid_=false;
        for(int a=0;a<3;++a)if(std::abs(axis(q,a))>=kSurfaceBoundsSafeCoordinate ||
            std::abs(axis(p,a))>=kSurfaceBoundsSafeCoordinate ||
            std::abs(axis(t,a))>=kSurfaceBoundsSafeCoordinate)surface_bounds_valid_=false;
        surface_bounds_min_={std::min(surface_bounds_min_.x,q.x),std::min(surface_bounds_min_.y,q.y),std::min(surface_bounds_min_.z,q.z)};
        surface_bounds_max_={std::max(surface_bounds_max_.x,q.x),std::max(surface_bounds_max_.y,q.y),std::max(surface_bounds_max_.z,q.z)};
        // Encloses inverse-vs-forward roundoff for the admitted <=1e-12
        // orthogonality error, plus floating-point corner arithmetic.
        surface_bounds_padding_=std::max(surface_bounds_padding_,
          64.e-12*(1.+std::abs(p.x)+std::abs(p.y)+std::abs(p.z)+std::abs(t.x)+std::abs(t.y)+std::abs(t.z)));
      }
    }
    reason_=nullptr;
  } catch(const std::exception&) {links_.clear();reason_="prepared_model_allocation_failure";}
}

Result classify(const Vec3& ray, double measured, double tolerance, const PreparedModel& model) {
  Result result;
  if(!std::isfinite(measured) || measured<=0) {
    result.classification=Classification::Invalid;result.complete=true;result.reason="invalid_measured_depth";return result;
  }
  if(!finite(ray) || ray.z!=1. || !std::isfinite(tolerance) || tolerance<0) {result.reason="invalid_ray_or_tolerance";return result;}
  if(!model.ready()) {result.reason=model.reason_;return result;}
  for(const auto& link:model.links_) if(!link.mesh->ready_ || link.mesh->revision_!=link.revision) {
    result.reason="model_identity_or_source_tf_invalid";return result;
  }
  const auto& budget=model.budget_;
  double closest=std::numeric_limits<double>::infinity();std::size_t closest_link=model.links_.size(),closest_triangle=0;
  // Reuse this exact immutable link/ray inverse rotation in all traversals of
  // one query. Nothing is cached across rays, models or source-time frames.
  // Primitive storage leaves unused maximum-link entries uninitialized.
  std::array<std::array<double,3>,kMaximumLinks> local_directions;
  for(std::size_t i=0;i<model.links_.size();++i) {
    const auto& link=model.links_[i];const Vec3 direction=inverse_rotate(link.optical_from_link,ray);
    local_directions[i]={direction.x,direction.y,direction.z};
    if(!finite(link.optical_origin_in_link) || !finite(direction)) {result.reason="nonfinite_local_ray";return result;}
    double local_closest=closest;std::size_t triangle=0;
    if(!link.mesh->first_surface(link.optical_origin_in_link,direction,budget,result,local_closest,triangle))return result;
    if(local_closest<closest) {closest=local_closest;closest_link=i;closest_triangle=triangle;}
  }
  double measured_closest=std::numeric_limits<double>::infinity();std::size_t measured_link=model.links_.size(),measured_triangle=0;
  for(std::size_t i=0;i<model.links_.size();++i) {
    const auto& link=model.links_[i];double local=measured_closest;std::size_t triangle=0;
    const double previous_min=result.ambiguous_min_axial_depth_m;
    const auto& d=local_directions[i];
    if(!link.mesh->measured_surface(link.optical_origin_in_link,{d[0],d[1],d[2]},
        budget,result,local,triangle,&link.measured_query))return result;
    if(std::isfinite(result.ambiguous_min_axial_depth_m)&&
        (!std::isfinite(previous_min)||result.ambiguous_min_axial_depth_m<previous_min))result.ambiguous_measured_link_index=i;
    if(local<measured_closest){measured_closest=local;measured_link=i;measured_triangle=triangle;}
  }
  if(result.measured_reference_ambiguous) {
    // Every candidate, including exact hits, contributes to this conservative
    // envelope. No guessed single range may decide a tolerance-boundary case.
    const double lower_delta=measured-result.ambiguous_max_axial_depth_m;
    const double upper_delta=measured-result.ambiguous_min_axial_depth_m;
    if(!std::isfinite(lower_delta)||!std::isfinite(upper_delta)) {
      result.reason="measured_interval_arithmetic";return result;
    }
    result.complete=true;result.classification=Classification::Unobserved;
    if(std::isfinite(closest)){result.has_first_surface=true;result.first_axial_depth_m=closest;result.link_index=closest_link;result.triangle_index=closest_triangle;}
    if(lower_delta>=-tolerance&&upper_delta<=tolerance) {
      result.classification=Classification::SelfSurface;result.reason="matches_measured_surface";
    } else if(upper_delta< -tolerance) {
      result.classification=Classification::ExternalSurface;result.reason="before_measured_surface";
    } else if(lower_delta>tolerance) {
      result.classification=Classification::ModelConflict;result.reason="behind_measured_surface";
    }
    if(result.classification!=Classification::Unobserved) {
      result.has_measured_surface=true;
      // Diagnostic endpoint of the interval, not a claim of unique intersection.
      result.measured_axial_depth_m=result.ambiguous_min_axial_depth_m;
      result.measured_link_index=result.ambiguous_measured_link_index;
      result.measured_triangle_index=result.ambiguous_measured_triangle_index;
    }
    return result;
  }
  if(std::isfinite(measured_closest)) {
    const double delta=measured-measured_closest;
    if(!std::isfinite(delta)){result.reason="depth_difference_arithmetic";return result;}
    result.complete=true;result.has_measured_surface=true;result.measured_axial_depth_m=measured_closest;
    result.measured_link_index=measured_link;result.measured_triangle_index=measured_triangle;
    if(std::isfinite(closest)){result.has_first_surface=true;result.first_axial_depth_m=closest;result.link_index=closest_link;result.triangle_index=closest_triangle;}
    if(std::abs(delta)<=tolerance){result.classification=Classification::SelfSurface;result.reason="matches_measured_surface";}
    else if(delta<0){result.classification=Classification::ExternalSurface;result.reason="before_measured_surface";}
    else {result.classification=Classification::ModelConflict;result.reason="behind_measured_surface";}
    return result;
  }
  // Without exact measured coverage, original first-hit semantics stay intact.
  // The finite rim query is permitted only after ALL exact mesh rays missed.
  bool boundary_match=false;
  if(!std::isfinite(closest)) {
    double nearest_squared=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<model.links_.size();++i) {
      const auto& link=model.links_[i];
      double predicted=std::numeric_limits<double>::infinity();std::size_t triangle=0;
      const auto before=nearest_squared;
      const auto& d=local_directions[i];
      if(!link.mesh->near_boundary(link.optical_origin_in_link,
          {d[0],d[1],d[2]},budget,result,nearest_squared,predicted,triangle))return result;
      if(nearest_squared<before) {closest=predicted;closest_link=i;closest_triangle=triangle;boundary_match=true;}
    }
  }
  // A near miss certifies only a matching shell return, never an occluder or
  // a clearing ray. Outside the unchanged range gate keep the original miss.
  if(boundary_match && std::abs(measured-closest)>tolerance)closest=std::numeric_limits<double>::infinity();
  result.complete=true;
  if(!std::isfinite(closest)) {result.classification=Classification::ExternalSurface;result.reason="no_self_intersection";return result;}
  result.has_first_surface=true;result.first_axial_depth_m=closest;result.link_index=closest_link;result.triangle_index=closest_triangle;
  const double delta=measured-closest;
  if(!std::isfinite(delta)) {result.complete=false;result.classification=Classification::Unobserved;result.reason="depth_difference_arithmetic";return result;}
  if(std::abs(delta)<=tolerance) {result.classification=Classification::SelfSurface;result.reason=boundary_match?"matches_authorized_finite_boundary":"matches_first_surface";}
  else if(delta<0) {result.classification=Classification::ExternalSurface;result.reason="before_model_surface";}
  else {result.classification=Classification::ModelConflict;result.reason="behind_model_surface";}
  return result;
}
Result classify(const Vec3& ray,double measured,double tolerance,const PreparedModel& model,
    const NativePixelSupport& pixel) {
  Result result=classify(ray,measured,tolerance,model);
  // Missing native raster proof never changes the legacy center-ray contract.
  if(!result.complete||result.classification==Classification::Invalid||
      !pixel.width||!pixel.height||pixel.u>=pixel.width||pixel.v>=pixel.height||
      !std::isfinite(pixel.fx)||!std::isfinite(pixel.fy)||!(pixel.fx>0)||!(pixel.fy>0)||
      !std::isfinite(pixel.cx)||!std::isfinite(pixel.cy))return result;
  const double rx=(double(pixel.u)-pixel.cx)/pixel.fx,ry=(double(pixel.v)-pixel.cy)/pixel.fy;
  if(!std::isfinite(rx)||!std::isfinite(ry))return result;
  const double ray_margin=8*std::numeric_limits<float>::epsilon()*std::max({1.,std::abs(rx),std::abs(ry)});
  if(std::abs(ray.x-rx)>ray_margin||std::abs(ray.y-ry)>ray_margin)return result;
  // The existing authorized center-ray Self gate is retained. Pure nominal Self
  // must NOT short-circuit a measured pixel interval (foreground counterexample).
  if(result.has_measured_surface&&result.classification==Classification::SelfSurface)return result;
  double lower=INFINITY,upper=-INFINITY;std::size_t owner=model.links_.size(),triangle=0;
  for(std::size_t i=0;i<model.links_.size();++i) {
    const auto& link=model.links_[i];const double before=lower;std::size_t local_triangle=0;
    if(!link.mesh->pixel_surface(pixel,link.optical_from_link,model.budget_,result,lower,upper,local_triangle)) {
      result.complete=false;result.classification=Classification::Unobserved;result.has_measured_surface=false;return result;
    }
    if(lower<before){owner=i;triangle=local_triangle;}
  }
  if(!std::isfinite(lower))return result;
  if(!std::isfinite(upper)||lower>upper){result.complete=false;result.classification=Classification::Unobserved;
    result.has_measured_surface=false;result.reason="pixel_interval_arithmetic";return result;}
  result.has_measured_surface=false;result.measured_axial_depth_m=NAN;
  result.measured_link_index=result.measured_triangle_index=std::numeric_limits<std::size_t>::max();
  result.measured_reference_ambiguous=true;result.ambiguous_min_axial_depth_m=lower;
  result.ambiguous_max_axial_depth_m=upper;result.ambiguous_measured_link_index=owner;
  result.ambiguous_measured_triangle_index=triangle;result.classification=Classification::Unobserved;
  result.reason="native_pixel_measured_interval_unobserved";
  const double margin=2*std::numeric_limits<float>::epsilon()*std::max({1.,std::abs(measured),std::abs(lower),std::abs(upper)});
  const double low_delta=measured-upper,high_delta=measured-lower;
  const double self_gate=tolerance;
  if(low_delta> -self_gate+margin&&high_delta<self_gate-margin) {
    result.classification=Classification::SelfSurface;result.reason="matches_measured_surface";
  } else if(high_delta< -tolerance-margin) {
    result.classification=Classification::ExternalSurface;result.reason="before_measured_surface";
  } else if(low_delta>tolerance+margin) {
    result.classification=Classification::ModelConflict;result.reason="behind_measured_surface";
  }
  if(result.classification!=Classification::Unobserved) {
    result.has_measured_surface=true;result.measured_axial_depth_m=lower;
    result.measured_link_index=owner;result.measured_triangle_index=triangle;
  }
  return result;
}

std::vector<PreparedModel::DiagnosticLink> PreparedModel::diagnostic_links() const {
  std::vector<DiagnosticLink> out;out.reserve(links_.size());
  for(std::size_t i=0;i<links_.size();++i){const auto& link=links_[i];
    out.push_back({i,link.mesh->triangle_count(),link.revision,link.optical_from_link,
      link.mesh->boundary_first_,link.mesh->boundary_count_,link.mesh->boundary_tolerance_m_,link.mesh->measured_first_,link.mesh->measured_count_});}
  return out;
}
Result classify_range(const Vec3& origin, const Vec3& ray, double measured, double tolerance, const PreparedModel& model) {
  return classify_range(origin,ray,measured,tolerance,model,Result{});
}
Result classify_range(const Vec3& origin,const Vec3& ray,double measured,double tolerance,const PreparedModel& model,Result counters) {
  Result result;result.node_visits=counters.node_visits;result.triangle_tests=counters.triangle_tests;
  if(!std::isfinite(measured) || measured<=0) {
    result.classification=Classification::Invalid;result.complete=true;result.reason="invalid_measured_depth";return result;
  }
  if(!finite(origin) || !finite(ray) || std::abs(dot(ray,ray)-1.)>1.e-12 || !std::isfinite(tolerance) || tolerance<0) {result.reason="invalid_ray_or_tolerance";return result;}
  if(!model.ready()) {result.reason=model.reason_;return result;}
  for(const auto& link:model.links_) if(!link.mesh->ready_ || link.mesh->revision_!=link.revision) {
    result.reason="model_identity_or_source_tf_invalid";return result;
  }
  const auto& budget=model.budget_;
  double closest=std::numeric_limits<double>::infinity();std::size_t closest_link=model.links_.size(),closest_triangle=0;
  for(std::size_t i=0;i<model.links_.size();++i) {
    const auto& link=model.links_[i];const Vec3 direction=inverse_rotate(link.optical_from_link,ray);
    if(!finite(link.optical_origin_in_link) || !finite(direction)) {result.reason="nonfinite_local_ray";return result;}
    double local_closest=closest;std::size_t triangle=0;
    const auto local_origin=inverse_rotate(link.optical_from_link,subtract(origin,link.optical_from_link.translation));
    if(!finite(local_origin)) {result.reason="nonfinite_local_ray";return result;}
    if(!link.mesh->first_surface(local_origin,direction,budget,result,local_closest,triangle))return result;
    if(local_closest<closest) {closest=local_closest;closest_link=i;closest_triangle=triangle;}
  }
  result.complete=true;
  if(!std::isfinite(closest)) {result.classification=Classification::ExternalSurface;result.reason="no_self_intersection";return result;}
  result.has_first_surface=true;result.first_axial_depth_m=closest;result.link_index=closest_link;result.triangle_index=closest_triangle;
  const double delta=measured-closest;
  if(!std::isfinite(delta)) {result.complete=false;result.classification=Classification::Unobserved;result.reason="depth_difference_arithmetic";return result;}
  if(std::abs(delta)<=tolerance) {result.classification=Classification::SelfSurface;result.reason="matches_first_surface";}
  else if(delta<0) {result.classification=Classification::ExternalSurface;result.reason="before_model_surface";}
  else {result.classification=Classification::ModelConflict;result.reason="behind_model_surface";}
  return result;
}
Result classify_body_surface(const Vec3& ray,double measured,double tolerance,const PreparedModel& model) {
  return match_finite_surface({ray.x*measured,ray.y*measured,measured},tolerance,model,
      classify(ray,measured,tolerance,model));
}
Result classify_body_surface(const Vec3& ray,double measured,double tolerance,
    const std::vector<LinkObservation>& links,const QueryBudget& budget) {
  return classify_body_surface(ray,measured,tolerance,PreparedModel(links,budget));
}
Result classify_body_surface(const Vec3& ray,double measured,double tolerance,const PreparedModel& model,
    const NativePixelSupport& pixel) {
  return match_finite_surface({ray.x*measured,ray.y*measured,measured},tolerance,model,
      classify(ray,measured,tolerance,model,pixel));
}
Result apply_native_projection_edge_policy(const Vec3& ray,double measured,double tolerance,
    const PreparedModel& model,const NativePixelSupport& pixel,Result result) {
  // Only the previously missed external center ray is an outside silhouette
  // candidate. Existing measured, Self, foreground and nominal conflict rules
  // remain the original geometric evidence, with the original metric shell.
  if(!result.complete||result.classification!=Classification::ExternalSurface||
      result.has_first_surface||result.has_measured_surface||
      !pixel.width||!pixel.height||pixel.u>=pixel.width||pixel.v>=pixel.height||
      !std::isfinite(pixel.fx)||!std::isfinite(pixel.fy)||!(pixel.fx>0)||!(pixel.fy>0)||
      !std::isfinite(pixel.cx)||!std::isfinite(pixel.cy))return result;
  const double rx=(double(pixel.u)-pixel.cx)/pixel.fx,ry=(double(pixel.v)-pixel.cy)/pixel.fy;
  const double margin=8*std::numeric_limits<float>::epsilon()*std::max({1.,std::abs(rx),std::abs(ry)});
  if(!std::isfinite(rx)||!std::isfinite(ry)||std::abs(ray.x-rx)>margin||std::abs(ray.y-ry)>margin)return result;
  if(!model.ready()) {result.complete=false;result.classification=Classification::Unobserved;result.reason=model.reason();return result;}
  for(const auto& link:model.links_)if(!link.mesh->ready_||link.mesh->revision_!=link.revision) {
    result.complete=false;result.classification=Classification::Unobserved;
    result.reason="model_identity_or_source_tf_invalid";return result;
  }
  double upper=-INFINITY;
  for(const auto& link:model.links_) {
    if(!link.mesh->projection_edge_surface(pixel,link.optical_from_link,model.budget_,result,upper)) {
      result.complete=false;result.classification=Classification::Unobserved;return result;
    }
  }
  const double depth_margin=2*std::numeric_limits<float>::epsilon()*
    std::max({1.,std::abs(measured),std::isfinite(upper)?std::abs(upper):0.});
  if(std::isfinite(upper)&&measured-upper>tolerance+depth_margin)
    result.reason="model_projection_edge_behind";
  return result;
}
Result apply_stereo_body_occlusion_policy(const Vec3& ray,double measured,double tolerance,
    const PreparedModel& model,const NativePixelSupport& pixel,Result result) {
  if(!result.complete || result.classification!=Classification::ExternalSurface ||
      result.has_first_surface || result.has_measured_surface || !finite(ray) || ray.z!=1. ||
      !std::isfinite(pixel.cx) || !std::isfinite(pixel.cy) ||
      !std::isfinite(pixel.stereo_baseline_m) || !(pixel.stereo_baseline_m>0.) ||
      !pixel.width || !pixel.height || pixel.u>=pixel.width || pixel.v>=pixel.height ||
      !std::isfinite(pixel.fx) || !std::isfinite(pixel.fy) || !(pixel.fx>0.) || !(pixel.fy>0.) ||
      !std::isfinite(measured) || !(measured>0.))return result;
  const double rx=(double(pixel.u)-pixel.cx)/pixel.fx,ry=(double(pixel.v)-pixel.cy)/pixel.fy;
  const double margin=8*std::numeric_limits<float>::epsilon()*std::max({1.,std::abs(rx),std::abs(ry)});
  const double right_u=double(pixel.u)-pixel.fx*pixel.stereo_baseline_m/measured;
  if(!std::isfinite(rx)||!std::isfinite(ry)||!std::isfinite(right_u)||right_u<0.||right_u>=double(pixel.width)||
      std::abs(ray.x-rx)>margin||std::abs(ray.y-ry)>margin)return result;
  const Vec3 origin{pixel.stereo_baseline_m,0.,0.};
  const Vec3 delta{ray.x*measured-origin.x,ray.y*measured,measured};
  const double range=std::sqrt(dot(delta,delta));
  if(!std::isfinite(range)||!(range>0.))return result;
  const Vec3 direction{delta.x/range,delta.y/range,delta.z/range};
  // Both eyes spend the SAME original query counters and model link capacity.
  const auto right=classify_range(origin,direction,range,tolerance,model,result);
  result.node_visits=right.node_visits;result.triangle_tests=right.triangle_tests;
  if(!right.complete){result.complete=false;result.classification=Classification::Unobserved;result.reason=right.reason;return result;}
  if(right.has_first_surface && right.classification==Classification::ModelConflict)
    result.reason="measurement_body_occluded";
  return result;
}
Result classify_native_body_filter(const Vec3& ray,double measured,double tolerance,
    const PreparedModel& model,const NativePixelSupport& pixel) {
  return apply_stereo_body_occlusion_policy(ray,measured,tolerance,model,pixel,
    apply_native_projection_edge_policy(ray,measured,tolerance,model,pixel,
      classify_body_surface(ray,measured,tolerance,model,pixel)));
}
Result classify_body_range(const Vec3& origin,const Vec3& ray,double measured,double tolerance,const PreparedModel& model) {
  return match_finite_surface({origin.x+ray.x*measured,origin.y+ray.y*measured,origin.z+ray.z*measured},
      tolerance,model,classify_range(origin,ray,measured,tolerance,model));
}
}  // namespace sunhorizon_obstacle_coverage::dynamic_self_geometry
