#!/usr/bin/env python3
"""Translate reviewed query arithmetic to device views; fail on source drift."""
import argparse,hashlib,pathlib,re
parser=argparse.ArgumentParser()
for name in ['header','source','arm-helper','output']:parser.add_argument('--'+name,type=pathlib.Path,required=True)
args=parser.parse_args();h=args.header.read_text();s=args.source.read_text()
expected={'header':'2997dc1b4ae7479c98bc99d2e419482f3728c6b5859502dc0ebbf139fe5718b3','source':'77ab6732180105f4bfb50d7f5a244e80cbfa1a98b5af0e5f9eb824562c9a9c41','arm_helper':'a79c827a36598b2beffea3ecd2f7b28c04e37c5b2413a5583792fdb4be125cbe'}
for name,content in [('header',h.encode()),('source',s.encode()),('arm_helper',args.arm_helper.read_bytes())]:
 if hashlib.sha256(content).hexdigest()!=expected[name]:raise SystemExit('unreviewed_camera_cuda_arithmetic_source:'+name)
g=h.replace('sunhorizon_obstacle_coverage::dynamic_self_geometry','gpu_geometry').replace(' private:',' public:').replace('std::vector<','View<')
# Device structs are borrowed views into immutable uploaded mesh buffers.
g=g.replace('  Mesh() = default;','  Mesh() = default;')
g=g.replace('  PreparedModel(const View<LinkObservation>& links, const QueryBudget& budget);','  PreparedModel() = default;')
methods=['ready','reason','query_budget','triangle_count','node_count','origin_radius_bound','prepare_measured_query','measured_surface','pixel_surface','projection_edge_surface','near_boundary','within_surface','first_surface','match_finite_surface','classify','classify_range','classify_body_surface','apply_native_projection_edge_policy','apply_stereo_body_occlusion_policy','classify_native_body_filter','classify_body_range','projection_edge_suppressed']
for name in methods:
 g=re.sub(r'(?m)^(\s*(?:friend )?)(Result|bool|double|std::size_t|MeasuredQuery|const char\*|const QueryBudget&)( '+name+r'\()',r'\1__device__ \2\3',g)
# Inline edge predicate may begin inline bool.
# projection_edge_suppressed is evaluated by the original host accumulator.
prefix='''
template<class T> struct View {
 T* pointer{}; std::size_t count{};
 __host__ __device__ std::size_t size() const {return count;}
 __host__ __device__ bool empty() const {return !count;}
 __host__ __device__ T& operator[](std::size_t i) const {return pointer[i];}
 __host__ __device__ T* begin() const {return pointer;}
 __host__ __device__ T* end() const {return pointer+count;}
};
template<class T> __device__ T dmin(T a,T b){return b<a?b:a;}
template<class T> __device__ T dmax(T a,T b){return a<b?b:a;}
template<class T> __device__ T dmin(std::initializer_list<T> a){T x=*a.begin();for(auto y:a)x=dmin(x,y);return x;}
template<class T> __device__ T dmax(std::initializer_list<T> a){T x=*a.begin();for(auto y:a)x=dmax(x,y);return x;}
template<class T> __device__ T dclamp(T v,T lo,T hi){return v<lo?lo:hi<v?hi:v;}
template<class T> __device__ void dswap(T& a,T& b){T t=a;a=b;b=t;}
'''
g=g.replace('namespace gpu_geometry {','namespace gpu_geometry {\n'+prefix)
# Select query functions by whole top-level definitions, never edit formulas.
body=s[s.index('namespace {'):s.index('}  // namespace\n')+len('}  // namespace\n')]
body+=s[s.index('Mesh::MeasuredQuery Mesh::prepare_measured_query'):s.index('Result classify(const Vec3& ray, double measured, double tolerance,\n')]
body+=s[s.index('Result classify(const Vec3& ray, double measured, double tolerance, const PreparedModel& model)'):s.index('std::vector<PreparedModel::DiagnosticLink>')]
body+=s[s.index('Result classify_range('):s.rindex('}  // namespace')]
start=body.index('Result classify_body_surface(const Vec3& ray,double measured,double tolerance,\n    const std::vector')
end=body.index('Result classify_body_surface(',start+10)
body=body[:start]+body[end:]
body=re.sub(r'(?m)^(bool|Vec3|double|Mesh::MeasuredQuery|Result) ',r'__device__ \1 ',body)
body=body.replace('const double kSurfaceBoundsSafeCoordinate=std::sqrt(std::numeric_limits<double>::max())/64.;','constexpr double kSurfaceBoundsSafeCoordinate=2.0949699890535306e152;')
for a,b in [('std::min','dmin'),('std::max','dmax'),('std::clamp','dclamp'),('std::swap','dswap')]:body=body.replace(a,b)
arm_nominal=True
if arm_nominal:
 override=args.arm_helper.read_text()
 # Preserve the original helper in generated evidence; explicitly use the
 # validated arithmetic for inverse rotation and nominal first-surface only.
 body=body.replace('__device__ Vec3 inverse_rotate(', '__device__ Vec3 original_inverse_rotate(',1)
 insert=body.index('__device__ bool rigid(')
 body=body[:insert]+override+'\n'+body[insert:]
 body=body.replace('inverse_rotate(', 'arm_inverse_rotate(').replace('original_arm_inverse_rotate(', 'original_inverse_rotate(').replace('arm_arm_inverse_rotate(', 'arm_inverse_rotate(')
 body=body.replace('if(!triangle_hit(triangles_[order_[i]],origin,direction,triangle_hit_seen,lambda))', 'if(!arm_nominal_triangle_hit(triangles_[order_[i]],origin,direction,triangle_hit_seen,lambda))')
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(g+'\nnamespace gpu_geometry {\n'+body+'\n}\n')
