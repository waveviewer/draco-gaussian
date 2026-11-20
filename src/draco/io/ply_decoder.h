#ifndef DRACO_IO_PLY_DECODER_H_
#define DRACO_IO_PLY_DECODER_H_

#include <string>

#include "draco/core/decoder_buffer.h"
#include "draco/core/status.h"
#include "draco/draco_features.h"
#include "draco/io/ply_reader.h"
#include "draco/mesh/mesh.h"

namespace draco {

// Decodes a PLY file into draco::Mesh (or draco::PointCloud if the
// connectivity data is not needed).
// TODO(b/34330853): The current implementation assumes that the input vertices
// are defined with x, y, z properties. The decoder also reads uint8 red, green,
// blue, alpha color information, float32 defined as nx, ny, nz properties, but
// all other attributes are ignored for now.
//
// DecodeFromFileGaussian() additionally reads the attributes written by 3D
// Gaussian splatting pipelines (opacity, scale, rotation, displacement and
// spherical harmonics coefficients).
class PlyDecoder {
 public:
  PlyDecoder();

  // Decodes an obj file stored in the input file.
  Status DecodeFromFile(const std::string &file_name, Mesh *out_mesh);
  Status DecodeFromFile(const std::string &file_name,
                        PointCloud *out_point_cloud);
  // Decodes a Gaussian splatting PLY file into a point cloud.
  Status DecodeFromFileGaussian(const std::string &filename,
                                PointCloud *out_point_cloud);

  Status DecodeFromBuffer(DecoderBuffer *buffer, Mesh *out_mesh);
  Status DecodeFromBuffer(DecoderBuffer *buffer, PointCloud *out_point_cloud);

 protected:
  Status DecodeInternal();
  DecoderBuffer *buffer() { return &buffer_; }

 private:
  Status DecodeFaceData(const PlyElement *face_element);
  Status DecodeVertexData(const PlyElement *vertex_element);
  bool HasProperties(const PlyElement *element,
                     const std::vector<std::string> &prop_names) const;

  void AddFloatAttribute(const PlyElement *element,
                         const std::vector<std::string> &prop_names,
                         GeometryAttribute::Type attribute_type,
                         int num_components, PointCloud *pc);
  void AddSphericalHarmonicsAttributes(const PlyElement *element,
                                       PointCloud *pc);
  void AddSphericalHarmonicsRestAttributes(const PlyElement *element,
                                           PointCloud *pc);
  void AddSingleFloatAttribute(const PlyElement *element,
                               const std::string &prop_name,
                               GeometryAttribute::Type attribute_type,
                               PointCloud *pc);

  template <typename DataTypeT>
  bool ReadPropertiesToAttribute(
      const std::vector<const PlyProperty *> &properties,
      PointAttribute *attribute, int num_vertices);

  DecoderBuffer buffer_;

  // Data structure that stores the decoded data. |out_point_cloud_| must be
  // always set but |out_mesh_| is optional.
  Mesh *out_mesh_;
  PointCloud *out_point_cloud_;
};

}  // namespace draco

#endif  // DRACO_IO_PLY_DECODER_H_
