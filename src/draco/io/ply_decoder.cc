#include "draco/io/ply_decoder.h"

#include "draco/core/macros.h"
#include "draco/core/status.h"
#include "draco/io/file_utils.h"
#include "draco/io/ply_property_reader.h"

namespace draco {
namespace {
int64_t CountNumTriangles(const PlyElement &face_element,
                          const PlyProperty &vertex_indices) {
  int64_t num_triangles = 0;
  for (int i = 0; i < face_element.num_entries(); ++i) {
    const int64_t list_size = vertex_indices.GetListEntryNumValues(i);
    if (list_size < 3) {
      // Correctly encoded ply files don't have less than three vertices.
      continue;
    }
    num_triangles += list_size - 2;
  }
  return num_triangles;
}
}  // namespace

PlyDecoder::PlyDecoder() : out_mesh_(nullptr), out_point_cloud_(nullptr) {}

Status PlyDecoder::DecodeFromFile(const std::string &file_name,
                                  Mesh *out_mesh) {
  out_mesh_ = out_mesh;
  return DecodeFromFile(file_name, static_cast<PointCloud *>(out_mesh));
}

Status PlyDecoder::DecodeFromFile(const std::string &file_name,
                                  PointCloud *out_point_cloud) {
  std::vector<char> data;
  if (!ReadFileToBuffer(file_name, &data)) {
    return Status(Status::DRACO_ERROR, "Unable to read input file.");
  }
  buffer_.Init(data.data(), data.size());
  return DecodeFromBuffer(&buffer_, out_point_cloud);
}

Status PlyDecoder::DecodeFromBuffer(DecoderBuffer *buffer, Mesh *out_mesh) {
  out_mesh_ = out_mesh;
  return DecodeFromBuffer(buffer, static_cast<PointCloud *>(out_mesh));
}

Status PlyDecoder::DecodeFromBuffer(DecoderBuffer *buffer,
                                    PointCloud *out_point_cloud) {
  out_point_cloud_ = out_point_cloud;
  buffer_.Init(buffer->data_head(), buffer->remaining_size());
  return DecodeInternal();
}

Status PlyDecoder::DecodeInternal() {
  PlyReader ply_reader;
  DRACO_RETURN_IF_ERROR(ply_reader.Read(buffer()));
  // First, decode the connectivity data.
  if (out_mesh_)
    DRACO_RETURN_IF_ERROR(DecodeFaceData(ply_reader.GetElementByName("face")));
  // Decode all attributes.
  DRACO_RETURN_IF_ERROR(
      DecodeVertexData(ply_reader.GetElementByName("vertex")));
  // In case there are no faces this is just a point cloud which does
  // not require deduplication.
  if (out_mesh_ && out_mesh_->num_faces() != 0) {
#ifdef DRACO_ATTRIBUTE_VALUES_DEDUPLICATION_SUPPORTED
    if (!out_point_cloud_->DeduplicateAttributeValues()) {
      return Status(Status::DRACO_ERROR,
                    "Could not deduplicate attribute values");
    }
#endif
#ifdef DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED
    out_point_cloud_->DeduplicatePointIds();
#endif
  }
  return OkStatus();
}

Status PlyDecoder::DecodeFaceData(const PlyElement *face_element) {
  // We accept point clouds now.
  if (face_element == nullptr) {
    return OkStatus();
  }
  const PlyProperty *vertex_indices =
      face_element->GetPropertyByName("vertex_indices");
  if (vertex_indices == nullptr) {
    // The property name may be named either "vertex_indices" or "vertex_index".
    vertex_indices = face_element->GetPropertyByName("vertex_index");
  }
  if (vertex_indices == nullptr || !vertex_indices->is_list()) {
    return Status(Status::DRACO_ERROR, "No faces defined");
  }

  // Allocate faces.
  out_mesh_->SetNumFaces(CountNumTriangles(*face_element, *vertex_indices));
  const int64_t num_polygons = face_element->num_entries();

  PlyPropertyReader<PointIndex::ValueType> vertex_index_reader(vertex_indices);
  Mesh::Face face;
  FaceIndex face_index(0);
  for (int i = 0; i < num_polygons; ++i) {
    const int64_t list_offset = vertex_indices->GetListEntryOffset(i);
    const int64_t list_size = vertex_indices->GetListEntryNumValues(i);
    if (list_size < 3) {
      continue;  // All invalid polygons are skipped.
    }

    // Triangulate polygon assuming the polygon is convex.
    const int64_t num_triangles = list_size - 2;
    face[0] = vertex_index_reader.ReadValue(static_cast<int>(list_offset));
    for (int64_t ti = 0; ti < num_triangles; ++ti) {
      for (int64_t c = 1; c < 3; ++c) {
        face[c] = vertex_index_reader.ReadValue(
            static_cast<int>(list_offset + ti + c));
      }
      out_mesh_->SetFace(face_index, face);
      face_index++;
    }
  }
  out_mesh_->SetNumFaces(face_index.value());
  return OkStatus();
}

template <typename DataTypeT>
bool PlyDecoder::ReadPropertiesToAttribute(
    const std::vector<const PlyProperty *> &properties,
    PointAttribute *attribute, int num_vertices) {
  std::vector<std::unique_ptr<PlyPropertyReader<DataTypeT>>> readers;
  readers.reserve(properties.size());
  for (int prop = 0; prop < properties.size(); ++prop) {
    readers.push_back(std::unique_ptr<PlyPropertyReader<DataTypeT>>(
        new PlyPropertyReader<DataTypeT>(properties[prop])));
  }
  std::vector<DataTypeT> memory(properties.size());
  for (PointIndex::ValueType i = 0; i < static_cast<uint32_t>(num_vertices);
       ++i) {
    for (int prop = 0; prop < properties.size(); ++prop) {
      memory[prop] = readers[prop]->ReadValue(i);
    }
    attribute->SetAttributeValue(AttributeValueIndex(i), memory.data());
  }
  return true;
}

Status PlyDecoder::DecodeVertexData(const PlyElement *vertex_element) {
  if (vertex_element == nullptr) {
    return Status(Status::INVALID_PARAMETER, "vertex_element is null");
  }
  // TODO(b/34330853): For now, try to load x,y,z vertices, red,green,blue,alpha
  // colors, and nx,ny,nz normals. We need to add other properties later.
  const PlyProperty *const x_prop = vertex_element->GetPropertyByName("x");
  const PlyProperty *const y_prop = vertex_element->GetPropertyByName("y");
  const PlyProperty *const z_prop = vertex_element->GetPropertyByName("z");
  if (!x_prop || !y_prop || !z_prop) {
    // Currently, we require 3 vertex coordinates (this should be generalized
    // later on).
    return Status(Status::INVALID_PARAMETER, "x, y, or z property is missing");
  }
  const PointIndex::ValueType num_vertices = vertex_element->num_entries();
  out_point_cloud_->set_num_points(num_vertices);
  // Decode vertex positions.
  {
    // All properties must have the same type.
    if (x_prop->data_type() != y_prop->data_type() ||
        y_prop->data_type() != z_prop->data_type()) {
      return Status(Status::INVALID_PARAMETER,
                    "x, y, and z properties must have the same type");
    }
    // TODO(ostava): For now assume the position types are float32 or int32.
    const DataType dt = x_prop->data_type();
    if (dt != DT_FLOAT32 && dt != DT_INT32) {
      return Status(Status::INVALID_PARAMETER,
                    "x, y, and z properties must be of type float32 or int32");
    }

    GeometryAttribute va;
    va.Init(GeometryAttribute::POSITION, nullptr, 3, dt, false,
            DataTypeLength(dt) * 3, 0);
    const int att_id = out_point_cloud_->AddAttribute(va, true, num_vertices);
    std::vector<const PlyProperty *> properties;
    properties.push_back(x_prop);
    properties.push_back(y_prop);
    properties.push_back(z_prop);
    if (dt == DT_FLOAT32) {
      ReadPropertiesToAttribute<float>(
          properties, out_point_cloud_->attribute(att_id), num_vertices);
    } else if (dt == DT_INT32) {
      ReadPropertiesToAttribute<int32_t>(
          properties, out_point_cloud_->attribute(att_id), num_vertices);
    }
  }

  // Decode normals if present.
  const PlyProperty *const n_x_prop = vertex_element->GetPropertyByName("nx");
  const PlyProperty *const n_y_prop = vertex_element->GetPropertyByName("ny");
  const PlyProperty *const n_z_prop = vertex_element->GetPropertyByName("nz");
  if (n_x_prop != nullptr && n_y_prop != nullptr && n_z_prop != nullptr) {
    // For now, all normal properties must be set and of type float32
    if (n_x_prop->data_type() == DT_FLOAT32 &&
        n_y_prop->data_type() == DT_FLOAT32 &&
        n_z_prop->data_type() == DT_FLOAT32) {
      PlyPropertyReader<float> x_reader(n_x_prop);
      PlyPropertyReader<float> y_reader(n_y_prop);
      PlyPropertyReader<float> z_reader(n_z_prop);
      GeometryAttribute va;
      va.Init(GeometryAttribute::NORMAL, nullptr, 3, DT_FLOAT32, false,
              sizeof(float) * 3, 0);
      const int att_id = out_point_cloud_->AddAttribute(va, true, num_vertices);
      for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
        std::array<float, 3> val;
        val[0] = x_reader.ReadValue(i);
        val[1] = y_reader.ReadValue(i);
        val[2] = z_reader.ReadValue(i);
        out_point_cloud_->attribute(att_id)->SetAttributeValue(
            AttributeValueIndex(i), &val[0]);
      }
    }
  }

  // Decode color data if present.
  int num_colors = 0;
  const PlyProperty *const r_prop = vertex_element->GetPropertyByName("red");
  const PlyProperty *const g_prop = vertex_element->GetPropertyByName("green");
  const PlyProperty *const b_prop = vertex_element->GetPropertyByName("blue");
  const PlyProperty *const a_prop = vertex_element->GetPropertyByName("alpha");
  if (r_prop) {
    ++num_colors;
  }
  if (g_prop) {
    ++num_colors;
  }
  if (b_prop) {
    ++num_colors;
  }
  if (a_prop) {
    ++num_colors;
  }

  if (num_colors) {
    std::vector<std::unique_ptr<PlyPropertyReader<uint8_t>>> color_readers;
    const PlyProperty *p;
    if (r_prop) {
      p = r_prop;
      // TODO(ostava): For now ensure the data type of all components is uint8.
      DRACO_DCHECK_EQ(true, p->data_type() == DT_UINT8);
      if (p->data_type() != DT_UINT8) {
        return Status(Status::INVALID_PARAMETER,
                      "Type of 'red' property must be uint8");
      }
      color_readers.push_back(std::unique_ptr<PlyPropertyReader<uint8_t>>(
          new PlyPropertyReader<uint8_t>(p)));
    }
    if (g_prop) {
      p = g_prop;
      // TODO(ostava): For now ensure the data type of all components is uint8.
      DRACO_DCHECK_EQ(true, p->data_type() == DT_UINT8);
      if (p->data_type() != DT_UINT8) {
        return Status(Status::INVALID_PARAMETER,
                      "Type of 'green' property must be uint8");
      }
      color_readers.push_back(std::unique_ptr<PlyPropertyReader<uint8_t>>(
          new PlyPropertyReader<uint8_t>(p)));
    }
    if (b_prop) {
      p = b_prop;
      // TODO(ostava): For now ensure the data type of all components is uint8.
      DRACO_DCHECK_EQ(true, p->data_type() == DT_UINT8);
      if (p->data_type() != DT_UINT8) {
        return Status(Status::INVALID_PARAMETER,
                      "Type of 'blue' property must be uint8");
      }
      color_readers.push_back(std::unique_ptr<PlyPropertyReader<uint8_t>>(
          new PlyPropertyReader<uint8_t>(p)));
    }
    if (a_prop) {
      p = a_prop;
      // TODO(ostava): For now ensure the data type of all components is uint8.
      DRACO_DCHECK_EQ(true, p->data_type() == DT_UINT8);
      if (p->data_type() != DT_UINT8) {
        return Status(Status::INVALID_PARAMETER,
                      "Type of 'alpha' property must be uint8");
      }
      color_readers.push_back(std::unique_ptr<PlyPropertyReader<uint8_t>>(
          new PlyPropertyReader<uint8_t>(p)));
    }

    GeometryAttribute va;
    va.Init(GeometryAttribute::COLOR, nullptr, num_colors, DT_UINT8, true,
            sizeof(uint8_t) * num_colors, 0);
    const int32_t att_id =
        out_point_cloud_->AddAttribute(va, true, num_vertices);
    for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
      std::array<uint8_t, 4> val;
      for (int j = 0; j < num_colors; j++) {
        val[j] = color_readers[j]->ReadValue(i);
      }
      out_point_cloud_->attribute(att_id)->SetAttributeValue(
          AttributeValueIndex(i), &val[0]);
    }
  }

  return OkStatus();
}

Status PlyDecoder::DecodeFromFileGaussian(const std::string &filename,
                                          PointCloud *out_point_cloud) {
  std::vector<char> data;
  if (!ReadFileToBuffer(filename, &data)) {
    return Status(Status::DRACO_ERROR, "Unable to read input file.");
  }
  DecoderBuffer buffer;
  buffer.Init(data.data(), data.size());

  PlyReader ply_reader;
  DRACO_RETURN_IF_ERROR(ply_reader.Read(&buffer));

  const PlyElement *vertex_element = ply_reader.GetElementByName("vertex");
  if (!vertex_element) {
    return Status(Status::DRACO_ERROR, "Missing vertex element.");
  }

  // Properties written by the common 3D Gaussian splatting exporters.
  const std::vector<std::string> required_properties = {
      "x",       "y",     "z",     "opacity", "scale_0", "scale_1",
      "scale_2", "rot_0", "rot_1", "rot_2",   "rot_3"};

  for (const auto &prop : required_properties) {
    if (!vertex_element->GetPropertyByName(prop)) {
      return Status(Status::DRACO_ERROR, "Missing required property: " + prop);
    }
  }

  const PointIndex::ValueType num_vertices = vertex_element->num_entries();
  out_point_cloud->set_num_points(num_vertices);

  // Position (x, y, z).
  AddFloatAttribute(vertex_element, {"x", "y", "z"},
                    GeometryAttribute::POSITION, 3, out_point_cloud);

  // Normals (nx, ny, nz), optional.
  if (HasProperties(vertex_element, {"nx", "ny", "nz"})) {
    AddFloatAttribute(vertex_element, {"nx", "ny", "nz"},
                      GeometryAttribute::NORMAL, 3, out_point_cloud);
  }

  // Color stored as the DC term of the spherical harmonics.
  if (HasProperties(vertex_element, {"f_dc_0", "f_dc_1", "f_dc_2"})) {
    AddSphericalHarmonicsAttributes(vertex_element, out_point_cloud);
  }

  // Gaussian splatting attributes.
  AddSingleFloatAttribute(vertex_element, "opacity",
                          GeometryAttribute::CUSTOM_ATTRIBUTE_OPACITY,
                          out_point_cloud);

  AddFloatAttribute(vertex_element, {"scale_0", "scale_1", "scale_2"},
                    GeometryAttribute::CUSTOM_ATTRIBUTE_SCALE, 3,
                    out_point_cloud);

  AddFloatAttribute(vertex_element, {"rot_0", "rot_1", "rot_2", "rot_3"},
                    GeometryAttribute::CUSTOM_ATTRIBUTE_ROTATION, 4,
                    out_point_cloud);

  // Per-point displacement, optional.
  if (HasProperties(vertex_element,
                    {"xyz_disp_0", "xyz_disp_1", "xyz_disp_2"})) {
    AddFloatAttribute(
        vertex_element, {"xyz_disp_0", "xyz_disp_1", "xyz_disp_2"},
        GeometryAttribute::CUSTOM_ATTRIBUTE_DISPLACEMENT, 3, out_point_cloud);
  }

  // Remaining spherical harmonics coefficients.
  if (HasProperties(vertex_element, {"f_rest_0"})) {
    AddSphericalHarmonicsRestAttributes(vertex_element, out_point_cloud);
  }
  return OkStatus();
}

// Adds a group of float properties as a single attribute.
void PlyDecoder::AddFloatAttribute(const PlyElement *element,
                                   const std::vector<std::string> &prop_names,
                                   GeometryAttribute::Type attribute_type,
                                   int num_components, PointCloud *pc) {
  std::vector<const PlyProperty *> properties;
  for (const auto &name : prop_names) {
    const PlyProperty *prop = element->GetPropertyByName(name);
    if (prop != nullptr) {
      properties.push_back(prop);
    }
  }

  if (properties.size() != num_components) return;

  GeometryAttribute attr;
  attr.Init(attribute_type, nullptr, num_components, DT_FLOAT32, false,
            sizeof(float) * num_components, 0);
  const int att_id = pc->AddAttribute(attr, true, pc->num_points());

  ReadPropertiesToAttribute<float>(properties, pc->attribute(att_id),
                                   pc->num_points());
}

// Adds the spherical harmonics DC term, which carries the base color.
void PlyDecoder::AddSphericalHarmonicsAttributes(const PlyElement *element,
                                                 PointCloud *pc) {
  GeometryAttribute attr;
  attr.Init(GeometryAttribute::CUSTOM_ATTRIBUTE_SH_DC, nullptr, 3, DT_FLOAT32,
            false, sizeof(float) * 3, 0);
  const int att_id = pc->AddAttribute(attr, true, pc->num_points());

  std::vector<const PlyProperty *> properties = {
      element->GetPropertyByName("f_dc_0"),
      element->GetPropertyByName("f_dc_1"),
      element->GetPropertyByName("f_dc_2")};

  ReadPropertiesToAttribute<float>(properties, pc->attribute(att_id),
                                   pc->num_points());
}

// Adds the higher order spherical harmonics coefficients.
void PlyDecoder::AddSphericalHarmonicsRestAttributes(const PlyElement *element,
                                                     PointCloud *pc) {
  const int kMaxRestComponents = 45;  // f_rest_0 through f_rest_44
  std::vector<const PlyProperty *> rest_properties;

  for (int i = 0; i < kMaxRestComponents; ++i) {
    std::string prop_name = "f_rest_" + std::to_string(i);
    const PlyProperty *prop = element->GetPropertyByName(prop_name);
    if (prop == nullptr) {
      break;
    }
    rest_properties.push_back(prop);
  }

  if (!rest_properties.empty()) {
    GeometryAttribute attr;
    attr.Init(GeometryAttribute::CUSTOM_ATTRIBUTE_SH_REST, nullptr,
              rest_properties.size(), DT_FLOAT32, false,
              sizeof(float) * rest_properties.size(), 0);
    const int att_id = pc->AddAttribute(attr, true, pc->num_points());
    ReadPropertiesToAttribute<float>(rest_properties, pc->attribute(att_id),
                                     pc->num_points());
  }
}

bool PlyDecoder::HasProperties(
    const PlyElement *element,
    const std::vector<std::string> &prop_names) const {
  for (const auto &name : prop_names) {
    if (!element->GetPropertyByName(name)) {
      return false;
    }
  }
  return true;
}

void PlyDecoder::AddSingleFloatAttribute(const PlyElement *element,
                                         const std::string &prop_name,
                                         GeometryAttribute::Type attribute_type,
                                         PointCloud *pc) {
  if (!element || !pc || pc->num_points() == 0) {
    return;
  }
  const PlyProperty *prop = element->GetPropertyByName(prop_name);
  if (!prop) {
    return;
  }

  GeometryAttribute attr;
  attr.Init(attribute_type, nullptr, 1, DT_FLOAT32, false, sizeof(float), 0);
  const int att_id = pc->AddAttribute(attr, true, pc->num_points());
  if (att_id < 0) {
    return;
  }

  std::vector<const PlyProperty *> properties{prop};
  ReadPropertiesToAttribute<float>(properties, pc->attribute(att_id),
                                   pc->num_points());
}

// Explicit instantiations of the template used above.

template bool PlyDecoder::ReadPropertiesToAttribute<float>(
    const std::vector<const PlyProperty *> &, PointAttribute *, int);

template bool PlyDecoder::ReadPropertiesToAttribute<int32_t>(
    const std::vector<const PlyProperty *> &, PointAttribute *, int);

template bool PlyDecoder::ReadPropertiesToAttribute<uint8_t>(
    const std::vector<const PlyProperty *> &, PointAttribute *, int);

}  // namespace draco
