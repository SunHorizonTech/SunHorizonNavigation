#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace sunhorizon_obstacle_coverage::dynamic_self_geometry {

// One body-surface matching tolerance for RGB-D, native pixels and LiDAR.
inline constexpr double kSelfSurfaceToleranceM = .03;

struct Vec3 { double x{0}, y{0}, z{0}; };
struct Triangle { std::array<Vec3, 3> vertices; };

// Rotation is row-major. No scale, shear or reflection is permitted.
struct RigidTransform {
  std::array<double, 9> rotation{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  Vec3 translation;
};

struct BuildLimits {
  std::size_t max_triangles{1000000};
  std::size_t max_nodes{2000000};
  std::size_t max_depth{32};
  std::size_t leaf_triangles{8};
};

struct QueryBudget {
  std::size_t max_links{32};
  std::size_t max_node_visits{65536};
  std::size_t max_triangle_tests{65536};
};

enum class Classification {
  SelfSurface, ExternalSurface, Invalid, ModelConflict, Unobserved
};

struct Result {
  Classification classification{Classification::Unobserved};
  bool complete{false};
  bool has_first_surface{false};
  // Legacy name: axial depth for classify(), Euclidean range for classify_range().
  double first_axial_depth_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t link_index{std::numeric_limits<std::size_t>::max()};
  std::size_t triangle_index{std::numeric_limits<std::size_t>::max()};
  // Explicit measured surface reference; original first-surface diagnostics stay nominal.
  // Nonunique finite references are resolved only when the complete candidate
  // range envelope has one classification. A tolerance-boundary overlap remains
  // single-point Unobserved (complete=true), never a nominal fallback. The
  // interval includes every exact/roundoff candidate; indices own its minimum.
  // These bounds may be populated for a unique query, but are used only if flagged.
  bool measured_reference_ambiguous{false};
  std::size_t ambiguous_measured_link_index{std::numeric_limits<std::size_t>::max()};
  std::size_t ambiguous_measured_triangle_index{std::numeric_limits<std::size_t>::max()};
  double ambiguous_min_axial_depth_m{std::numeric_limits<double>::quiet_NaN()};
  double ambiguous_max_axial_depth_m{std::numeric_limits<double>::quiet_NaN()};
  bool has_measured_surface{false};
  double measured_axial_depth_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t measured_link_index{std::numeric_limits<std::size_t>::max()};
  std::size_t measured_triangle_index{std::numeric_limits<std::size_t>::max()};
  std::size_t node_visits{0};
  // Surface primitive budget: triangles, finite rim edges and pixel clip edges.
  std::size_t triangle_tests{0};
  // A finite triangle witness, independent of the camera's first occluder.
  double finite_surface_distance_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t finite_surface_link_index{std::numeric_limits<std::size_t>::max()};
  std::size_t finite_surface_triangle_index{std::numeric_limits<std::size_t>::max()};
  const char* reason{"unobserved"};
};

// Original, rectified native raster cell; the image owner must prove exact
// source/header/K/metric identity before supplying this view. No stride widening.
struct NativePixelSupport {
  std::size_t width{0},height{0},u{0},v{0};
  double fx{0},fy{0},cx{0},cy{0};
  // Nonzero only with the original source-bound exact rectified stereo proof.
  double stereo_baseline_m{0};
};

struct LinkObservation;
class PreparedModel;
class CudaGeometryAccess; // Narrow read-only upload access; no class layout change.

class Mesh {
 public:
  Mesh() = default;
  Mesh(const Mesh&) = default;
  Mesh(Mesh&& other) noexcept;
  Mesh& operator=(const Mesh& other);
  Mesh& operator=(Mesh&& other) noexcept;
  // Atomic validity: a failed rebuild invalidates this mesh. Finite but
  // degenerate triangles are rejected, not silently used as surfaces.
  bool build(const std::vector<Triangle>& triangles, const BuildLimits& limits,
             std::string* error = nullptr);
  // Exact binary STL layout only. Owner supplies the expected count and proves
  // asset SHA separately. Normals/attribute bytes never define surface identity.
  bool load_binary_stl(const std::uint8_t* bytes, std::size_t size,
                       std::size_t expected_triangles, const BuildLimits& limits,
                       std::string* error = nullptr);
  // Default off. SHA-qualified owner authorizes only this finite triangle patch.
  // Exact shared edges cancel; only its actual open rim can support a near miss.
  bool configure_boundary_patch(std::size_t first, std::size_t count,
      double tolerance_m, std::string* error = nullptr);
  // Default off. Only SHA-qualified finite measured triangles may correct a
  // nominal CAD occluder. Center-ray coverage remains exact; only the explicitly
  // qualified native-raster overload adds finite pixel-cell coverage.
  bool configure_measured_surface(std::size_t first, std::size_t count,
      std::string* error = nullptr);
  bool ready() const noexcept { return ready_; }
  std::size_t triangle_count() const noexcept { return triangles_.size(); }
  std::size_t node_count() const noexcept { return nodes_.size(); }
  // Conservative link-origin sphere from the built CAD root bounds.
  double origin_radius_bound() const noexcept;

 private:
  friend class CudaGeometryAccess;
  struct Bounds { Vec3 minimum, maximum; };
  struct Node {
    Bounds bounds;
    std::size_t begin{0}, count{0}, left{0}, right{0};
  };
  bool ready_{false};
  uint64_t revision_{1};
  void invalidate_revision() noexcept;
  std::vector<Triangle> triangles_;
  std::vector<std::size_t> order_;
  std::vector<Node> nodes_;
  struct BoundaryEdge {Vec3 a,b;std::size_t triangle;};
  std::vector<BoundaryEdge> boundary_edges_;
  Bounds boundary_bounds_;
  std::size_t boundary_first_{0},boundary_count_{0};
  double boundary_tolerance_m_{0.};
  std::size_t measured_first_{0},measured_count_{0};
  Bounds measured_bounds_;
  // Same immutable source-time origin as PreparedModel; revision checks still
  // precede every query. This caches arithmetic, never geometry or lease credit.
  struct MeasuredQuery {double roundoff{0.};Bounds root;bool valid{true};};
  MeasuredQuery prepare_measured_query(const Vec3&) const;
  bool measured_surface(const Vec3&,const Vec3&,const QueryBudget&,Result&,
      double& closest,std::size_t& triangle,const MeasuredQuery* cached=nullptr) const;
  bool pixel_surface(const NativePixelSupport&,const RigidTransform&,
      const QueryBudget&,Result&,double& lower,double& upper,std::size_t& triangle) const;
  bool projection_edge_surface(const NativePixelSupport&,const RigidTransform&,
      const QueryBudget&,Result&,double& upper) const;
  bool near_boundary(const Vec3& origin, const Vec3& direction,
      const QueryBudget&, Result&, double& distance_squared,
      double& expected_depth, std::size_t& triangle) const;
  bool within_surface(const Vec3&,double,const QueryBudget&,Result&,
      bool&,double&,std::size_t&) const;
  std::size_t build_node(std::size_t begin, std::size_t end,
                         std::size_t depth, const BuildLimits& limits);
  bool first_surface(const Vec3& origin, const Vec3& direction,
                     const QueryBudget& budget, Result& counters,
                     double& closest, std::size_t& triangle) const;
  friend Result classify(const Vec3&, double, double,
                          const std::vector<LinkObservation>&,
                          const QueryBudget&);
  friend class PreparedModel;
  friend Result classify(const Vec3&, double, double, const PreparedModel&);
  friend Result classify(const Vec3&, double, double, const PreparedModel&,const NativePixelSupport&);
  friend Result classify_range(const Vec3&, const Vec3&, double, double, const PreparedModel&);
  friend Result classify_range(const Vec3&, const Vec3&, double, double, const PreparedModel&,Result);
  friend Result match_finite_surface(const Vec3&,double,const PreparedModel&,Result);
  friend Result apply_native_projection_edge_policy(const Vec3&,double,double,const PreparedModel&,const NativePixelSupport&,Result);
};

struct LinkObservation {
  const Mesh* mesh{nullptr};
  std::string link_name;
  RigidTransform optical_from_link;
  bool identity_valid{false};
  bool tf_valid{false};
  // Set by the owner only when the transform is qualified for the image's
  // exact source time. Stale, extrapolated or absent TF must set this false.
  bool covers_source_time{false};
};

// Per-frame immutable validation/transform snapshot. Mesh storage is borrowed:
// owners must keep it alive and must not rebuild/move it concurrently with a
// query. Sequential rebuild/assignment invalidates the snapshot. No TF, pose,
// identity or evidence may be refreshed through this object.
class PreparedModel {
 public:
  PreparedModel(const std::vector<LinkObservation>& links, const QueryBudget& budget);
  bool ready() const noexcept { return reason_ == nullptr; }
  const char* reason() const noexcept { return reason_; }
  const QueryBudget& query_budget() const noexcept { return budget_; }
  // A near-body endpoint exclusion needs a finite triangle witness, never a visibility
  // or clearing proof. Invalid geometry or exhausted work leaves the point eligible.
  bool excludes_near_body_map_mark(const Vec3& point) const;
  struct DiagnosticLink {
    std::size_t link_index, triangle_count;
    uint64_t mesh_revision;
    RigidTransform optical_from_link;
    std::size_t boundary_triangle_begin{0},boundary_triangle_count{0};
    double boundary_tolerance_m{0.};
    std::size_t measured_triangle_begin{0},measured_triangle_count{0};
  };
  // Explicit diagnostic copy from this exact prepared owner; never FK/raycast.
  // Index order is the query/model manifest order, not a new frame registry.
  std::vector<DiagnosticLink> diagnostic_links() const;
 private:
  friend class CudaGeometryAccess;
  struct Link {
    const Mesh* mesh;
    uint64_t revision;
    RigidTransform optical_from_link;
    Vec3 optical_origin_in_link;
    Mesh::MeasuredQuery measured_query;
  };
  std::vector<Link> links_;
  Vec3 surface_bounds_min_,surface_bounds_max_;
  double surface_bounds_padding_{0.};
  bool surface_bounds_valid_{false};
  QueryBudget budget_;
  const char* reason_{"invalid_query_capacity"};
  friend Result classify(const Vec3&, double, double, const PreparedModel&);
  friend Result classify(const Vec3&, double, double, const PreparedModel&,const NativePixelSupport&);
  friend Result classify_range(const Vec3&, const Vec3&, double, double, const PreparedModel&);
  friend Result classify_range(const Vec3&, const Vec3&, double, double, const PreparedModel&,Result);
  friend Result match_finite_surface(const Vec3&,double,const PreparedModel&,Result);
  friend Result apply_native_projection_edge_policy(const Vec3&,double,double,const PreparedModel&,const NativePixelSupport&,Result);
};

// A completed visibility query may additionally identify a return within the
// configured metric tolerance of an actual, finite model triangle. This does
// not fill mesh holes, test an AABB as body, or grant clearing behind the body.
// Incomplete input/model/budget remains incomplete; it is never rescued as Self.
Result match_finite_surface(const Vec3& point,double tolerance_m,
    const PreparedModel& model,Result visibility);

// Ray origin is the optical origin; direction MUST be unnormalized with z=1.
// Intersection lambda is axial optical Z, not Euclidean range. Both faces are
// tested, including a first forward exit if the optical origin is inside a mesh.
// Nominal CAD uses its globally nearest positive surface. An explicitly
// authorized finite measured patch, with COMPLETE exact-ray coverage, instead
// supplies the nearest measured surface reference: before=External, matching=Self,
// behind=Conflict at the unchanged tolerance. Uncovered rays retain nominal rules.
// No measured-patch override is applied by classify_range().
// Missing model/identity/TF, invalid rigid input or incomplete budget returns
// complete=false/Unobserved, never a partial SelfSurface. A SelfSurface grants
// no free-space evidence behind itself; ModelConflict retains the measurement.
Result classify(const Vec3& optical_ray_z1, double measured_axial_depth_m,
                double tolerance_m, const std::vector<LinkObservation>& links,
                const QueryBudget& budget);

// Same geometry, classification and per-ray counters as the legacy overload;
// only frame-invariant checks and inverse optical origins are reused.
Result classify(const Vec3& optical_ray_z1, double measured_axial_depth_m,
                double tolerance_m, const PreparedModel& model);

// Qualified native pixels may also cover an authorized measured facet inside
// their +/-0.5 pixel angular cell. Every clipped facet contributes to a complete
// conservative axial interval. Existing measured center-ray Self is unchanged;
// additional axial Self requires the WHOLE interval strictly inside +/-tol
// with a FLOAT32 margin. External/Conflict require the whole interval beyond the
// original tolerance. All other supported cases are local Unobserved, never a
// nominal fallback or ground evidence. Invalid raster proof uses the old query.
Result classify(const Vec3& optical_ray_z1, double measured_axial_depth_m,
                double tolerance_m, const PreparedModel& model,
                const NativePixelSupport& native_pixel);

// Omnidirectional LiDAR query. Origin and unit direction are in the same frame
// as PreparedModel transforms; no positive-Z/optical-axis assumption is made.
// Only the nearest positive CAD surface can match. A return behind that surface
// is retained as ModelConflict, and SelfSurface never grants clearing credit.
Result classify_range(const Vec3& origin, const Vec3& unit_direction,
    double measured_range_m, double tolerance_m, const PreparedModel& model);

Result classify_range(const Vec3&,const Vec3&,double,double,const PreparedModel&,Result counters);
Result apply_stereo_body_occlusion_policy(const Vec3&,double,double,const PreparedModel&,const NativePixelSupport&,Result);
inline bool measurement_body_occluded(const Result& result) {
  return result.complete && result.reason && std::strcmp(result.reason,"measurement_body_occluded")==0;
}

// Runtime body filtering uses one finite metric shell. Ray/pixel classification
// above supplies visibility diagnostics for retained returns; it never bypasses
// this surface-distance decision. Every removed Self has a finite triangle
// witness at <= tolerance_m. No free-space credit follows a removed body point.
Result classify_body_surface(const Vec3&,double,double,const PreparedModel&);
Result classify_body_surface(const Vec3&,double,double,const std::vector<LinkObservation>&,const QueryBudget&);
Result classify_body_surface(const Vec3&,double,double,const PreparedModel&,const NativePixelSupport&);
// BODY-only policy: retain the original geometry classification, but report
// model_projection_edge_behind when a missed center ray is within a TWO native
// pixel CENTER Euclidean radius of a finite model projection and the measured
// depth is behind its complete conservative depth envelope by tolerance_m.
// No extra half pixel, finite-surface Self witness, or clearing/Self support.
// Missing native proof keeps the original query. Work shares its query budget.
// Add only the policy to an already completed visibility/surface query. This
// preserves callers whose original coverage contract is center-ray-only.
Result apply_native_projection_edge_policy(const Vec3&,double,double,const PreparedModel&,const NativePixelSupport&,Result);
Result classify_native_body_filter(const Vec3&,double,double,const PreparedModel&,const NativePixelSupport&);
inline bool projection_edge_suppressed(const Result& result) {
  return result.complete && result.reason &&
    std::strcmp(result.reason,"model_projection_edge_behind")==0;
}
Result classify_body_range(const Vec3&,const Vec3&,double,double,const PreparedModel&);

}  // namespace sunhorizon_obstacle_coverage::dynamic_self_geometry
