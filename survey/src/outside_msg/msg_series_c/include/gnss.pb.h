// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: gnss.proto

#ifndef PROTOBUF_gnss_2eproto__INCLUDED
#define PROTOBUF_gnss_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "header.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_gnss_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsGnssImpl();
void InitDefaultsGnss();
inline void InitDefaults() {
  InitDefaultsGnss();
}
}  // namespace protobuf_gnss_2eproto
namespace phoenix {
namespace msg {
namespace localization {
class Gnss;
class GnssDefaultTypeInternal;
extern GnssDefaultTypeInternal _Gnss_default_instance_;
}  // namespace localization
}  // namespace msg
}  // namespace phoenix
namespace phoenix {
namespace msg {
namespace localization {

enum Gnss_Status {
  Gnss_Status_STATUS_INVALID = 0,
  Gnss_Status_STATUS_BAD = 1,
  Gnss_Status_STATUS_CONVERGING = 2,
  Gnss_Status_STATUS_GOOD = 3
};
bool Gnss_Status_IsValid(int value);
const Gnss_Status Gnss_Status_Status_MIN = Gnss_Status_STATUS_INVALID;
const Gnss_Status Gnss_Status_Status_MAX = Gnss_Status_STATUS_GOOD;
const int Gnss_Status_Status_ARRAYSIZE = Gnss_Status_Status_MAX + 1;

const ::google::protobuf::EnumDescriptor* Gnss_Status_descriptor();
inline const ::std::string& Gnss_Status_Name(Gnss_Status value) {
  return ::google::protobuf::internal::NameOfEnum(
    Gnss_Status_descriptor(), value);
}
inline bool Gnss_Status_Parse(
    const ::std::string& name, Gnss_Status* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Gnss_Status>(
    Gnss_Status_descriptor(), name, value);
}
// ===================================================================

class Gnss : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:phoenix.msg.localization.Gnss) */ {
 public:
  Gnss();
  virtual ~Gnss();

  Gnss(const Gnss& from);

  inline Gnss& operator=(const Gnss& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Gnss(Gnss&& from) noexcept
    : Gnss() {
    *this = ::std::move(from);
  }

  inline Gnss& operator=(Gnss&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Gnss& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Gnss* internal_default_instance() {
    return reinterpret_cast<const Gnss*>(
               &_Gnss_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(Gnss* other);
  friend void swap(Gnss& a, Gnss& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Gnss* New() const PROTOBUF_FINAL { return New(NULL); }

  Gnss* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Gnss& from);
  void MergeFrom(const Gnss& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(Gnss* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  typedef Gnss_Status Status;
  static const Status STATUS_INVALID =
    Gnss_Status_STATUS_INVALID;
  static const Status STATUS_BAD =
    Gnss_Status_STATUS_BAD;
  static const Status STATUS_CONVERGING =
    Gnss_Status_STATUS_CONVERGING;
  static const Status STATUS_GOOD =
    Gnss_Status_STATUS_GOOD;
  static inline bool Status_IsValid(int value) {
    return Gnss_Status_IsValid(value);
  }
  static const Status Status_MIN =
    Gnss_Status_Status_MIN;
  static const Status Status_MAX =
    Gnss_Status_Status_MAX;
  static const int Status_ARRAYSIZE =
    Gnss_Status_Status_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Status_descriptor() {
    return Gnss_Status_descriptor();
  }
  static inline const ::std::string& Status_Name(Status value) {
    return Gnss_Status_Name(value);
  }
  static inline bool Status_Parse(const ::std::string& name,
      Status* value) {
    return Gnss_Status_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // optional .phoenix.msg.common.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::phoenix::msg::common::Header& header() const;
  ::phoenix::msg::common::Header* release_header();
  ::phoenix::msg::common::Header* mutable_header();
  void set_allocated_header(::phoenix::msg::common::Header* header);

  // optional double latitude = 2 [default = 0];
  bool has_latitude() const;
  void clear_latitude();
  static const int kLatitudeFieldNumber = 2;
  double latitude() const;
  void set_latitude(double value);

  // optional double longitude = 3 [default = 0];
  bool has_longitude() const;
  void clear_longitude();
  static const int kLongitudeFieldNumber = 3;
  double longitude() const;
  void set_longitude(double value);

  // optional double altitude = 4 [default = 0];
  bool has_altitude() const;
  void clear_altitude();
  static const int kAltitudeFieldNumber = 4;
  double altitude() const;
  void set_altitude(double value);

  // optional double x_utm = 6 [default = 0];
  bool has_x_utm() const;
  void clear_x_utm();
  static const int kXUtmFieldNumber = 6;
  double x_utm() const;
  void set_x_utm(double value);

  // optional double y_utm = 7 [default = 0];
  bool has_y_utm() const;
  void clear_y_utm();
  static const int kYUtmFieldNumber = 7;
  double y_utm() const;
  void set_y_utm(double value);

  // optional float heading_gnss = 5 [default = 0];
  bool has_heading_gnss() const;
  void clear_heading_gnss();
  static const int kHeadingGnssFieldNumber = 5;
  float heading_gnss() const;
  void set_heading_gnss(float value);

  // optional float heading_utm = 9 [default = 0];
  bool has_heading_utm() const;
  void clear_heading_utm();
  static const int kHeadingUtmFieldNumber = 9;
  float heading_utm() const;
  void set_heading_utm(float value);

  // optional double z_utm = 8 [default = 0];
  bool has_z_utm() const;
  void clear_z_utm();
  static const int kZUtmFieldNumber = 8;
  double z_utm() const;
  void set_z_utm(double value);

  // optional double x_odom = 10 [default = 0];
  bool has_x_odom() const;
  void clear_x_odom();
  static const int kXOdomFieldNumber = 10;
  double x_odom() const;
  void set_x_odom(double value);

  // optional double y_odom = 11 [default = 0];
  bool has_y_odom() const;
  void clear_y_odom();
  static const int kYOdomFieldNumber = 11;
  double y_odom() const;
  void set_y_odom(double value);

  // optional double z_odom = 12 [default = 0];
  bool has_z_odom() const;
  void clear_z_odom();
  static const int kZOdomFieldNumber = 12;
  double z_odom() const;
  void set_z_odom(double value);

  // optional float heading_odom = 13 [default = 0];
  bool has_heading_odom() const;
  void clear_heading_odom();
  static const int kHeadingOdomFieldNumber = 13;
  float heading_odom() const;
  void set_heading_odom(float value);

  // optional float pitch = 14 [default = 0];
  bool has_pitch() const;
  void clear_pitch();
  static const int kPitchFieldNumber = 14;
  float pitch() const;
  void set_pitch(float value);

  // optional float roll = 15 [default = 0];
  bool has_roll() const;
  void clear_roll();
  static const int kRollFieldNumber = 15;
  float roll() const;
  void set_roll(float value);

  // optional float v_e = 16 [default = 0];
  bool has_v_e() const;
  void clear_v_e();
  static const int kVEFieldNumber = 16;
  float v_e() const;
  void set_v_e(float value);

  // optional float v_n = 17 [default = 0];
  bool has_v_n() const;
  void clear_v_n();
  static const int kVNFieldNumber = 17;
  float v_n() const;
  void set_v_n(float value);

  // optional float v_u = 18 [default = 0];
  bool has_v_u() const;
  void clear_v_u();
  static const int kVUFieldNumber = 18;
  float v_u() const;
  void set_v_u(float value);

  // optional float v_x_utm = 19 [default = 0];
  bool has_v_x_utm() const;
  void clear_v_x_utm();
  static const int kVXUtmFieldNumber = 19;
  float v_x_utm() const;
  void set_v_x_utm(float value);

  // optional float v_y_utm = 20 [default = 0];
  bool has_v_y_utm() const;
  void clear_v_y_utm();
  static const int kVYUtmFieldNumber = 20;
  float v_y_utm() const;
  void set_v_y_utm(float value);

  // optional float v_z_utm = 21 [default = 0];
  bool has_v_z_utm() const;
  void clear_v_z_utm();
  static const int kVZUtmFieldNumber = 21;
  float v_z_utm() const;
  void set_v_z_utm(float value);

  // optional float v_x_odom = 22 [default = 0];
  bool has_v_x_odom() const;
  void clear_v_x_odom();
  static const int kVXOdomFieldNumber = 22;
  float v_x_odom() const;
  void set_v_x_odom(float value);

  // optional float v_y_odom = 23 [default = 0];
  bool has_v_y_odom() const;
  void clear_v_y_odom();
  static const int kVYOdomFieldNumber = 23;
  float v_y_odom() const;
  void set_v_y_odom(float value);

  // optional float v_z_odom = 24 [default = 0];
  bool has_v_z_odom() const;
  void clear_v_z_odom();
  static const int kVZOdomFieldNumber = 24;
  float v_z_odom() const;
  void set_v_z_odom(float value);

  // optional .phoenix.msg.localization.Gnss.Status gnss_status = 25 [default = STATUS_INVALID];
  bool has_gnss_status() const;
  void clear_gnss_status();
  static const int kGnssStatusFieldNumber = 25;
  ::phoenix::msg::localization::Gnss_Status gnss_status() const;
  void set_gnss_status(::phoenix::msg::localization::Gnss_Status value);

  // optional .phoenix.msg.localization.Gnss.Status utm_status = 26 [default = STATUS_INVALID];
  bool has_utm_status() const;
  void clear_utm_status();
  static const int kUtmStatusFieldNumber = 26;
  ::phoenix::msg::localization::Gnss_Status utm_status() const;
  void set_utm_status(::phoenix::msg::localization::Gnss_Status value);

  // optional .phoenix.msg.localization.Gnss.Status odom_status = 27 [default = STATUS_INVALID];
  bool has_odom_status() const;
  void clear_odom_status();
  static const int kOdomStatusFieldNumber = 27;
  ::phoenix::msg::localization::Gnss_Status odom_status() const;
  void set_odom_status(::phoenix::msg::localization::Gnss_Status value);

  // @@protoc_insertion_point(class_scope:phoenix.msg.localization.Gnss)
 private:
  void set_has_header();
  void clear_has_header();
  void set_has_latitude();
  void clear_has_latitude();
  void set_has_longitude();
  void clear_has_longitude();
  void set_has_altitude();
  void clear_has_altitude();
  void set_has_heading_gnss();
  void clear_has_heading_gnss();
  void set_has_x_utm();
  void clear_has_x_utm();
  void set_has_y_utm();
  void clear_has_y_utm();
  void set_has_z_utm();
  void clear_has_z_utm();
  void set_has_heading_utm();
  void clear_has_heading_utm();
  void set_has_x_odom();
  void clear_has_x_odom();
  void set_has_y_odom();
  void clear_has_y_odom();
  void set_has_z_odom();
  void clear_has_z_odom();
  void set_has_heading_odom();
  void clear_has_heading_odom();
  void set_has_pitch();
  void clear_has_pitch();
  void set_has_roll();
  void clear_has_roll();
  void set_has_v_e();
  void clear_has_v_e();
  void set_has_v_n();
  void clear_has_v_n();
  void set_has_v_u();
  void clear_has_v_u();
  void set_has_v_x_utm();
  void clear_has_v_x_utm();
  void set_has_v_y_utm();
  void clear_has_v_y_utm();
  void set_has_v_z_utm();
  void clear_has_v_z_utm();
  void set_has_v_x_odom();
  void clear_has_v_x_odom();
  void set_has_v_y_odom();
  void clear_has_v_y_odom();
  void set_has_v_z_odom();
  void clear_has_v_z_odom();
  void set_has_gnss_status();
  void clear_has_gnss_status();
  void set_has_utm_status();
  void clear_has_utm_status();
  void set_has_odom_status();
  void clear_has_odom_status();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::phoenix::msg::common::Header* header_;
  double latitude_;
  double longitude_;
  double altitude_;
  double x_utm_;
  double y_utm_;
  float heading_gnss_;
  float heading_utm_;
  double z_utm_;
  double x_odom_;
  double y_odom_;
  double z_odom_;
  float heading_odom_;
  float pitch_;
  float roll_;
  float v_e_;
  float v_n_;
  float v_u_;
  float v_x_utm_;
  float v_y_utm_;
  float v_z_utm_;
  float v_x_odom_;
  float v_y_odom_;
  float v_z_odom_;
  int gnss_status_;
  int utm_status_;
  int odom_status_;
  friend struct ::protobuf_gnss_2eproto::TableStruct;
  friend void ::protobuf_gnss_2eproto::InitDefaultsGnssImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Gnss

// optional .phoenix.msg.common.Header header = 1;
inline bool Gnss::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Gnss::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Gnss::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::phoenix::msg::common::Header& Gnss::header() const {
  const ::phoenix::msg::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.header)
  return p != NULL ? *p : *reinterpret_cast<const ::phoenix::msg::common::Header*>(
      &::phoenix::msg::common::_Header_default_instance_);
}
inline ::phoenix::msg::common::Header* Gnss::release_header() {
  // @@protoc_insertion_point(field_release:phoenix.msg.localization.Gnss.header)
  clear_has_header();
  ::phoenix::msg::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::phoenix::msg::common::Header* Gnss::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    header_ = new ::phoenix::msg::common::Header;
  }
  // @@protoc_insertion_point(field_mutable:phoenix.msg.localization.Gnss.header)
  return header_;
}
inline void Gnss::set_allocated_header(::phoenix::msg::common::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    set_has_header();
  } else {
    clear_has_header();
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:phoenix.msg.localization.Gnss.header)
}

// optional double latitude = 2 [default = 0];
inline bool Gnss::has_latitude() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Gnss::set_has_latitude() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Gnss::clear_has_latitude() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Gnss::clear_latitude() {
  latitude_ = 0;
  clear_has_latitude();
}
inline double Gnss::latitude() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.latitude)
  return latitude_;
}
inline void Gnss::set_latitude(double value) {
  set_has_latitude();
  latitude_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.latitude)
}

// optional double longitude = 3 [default = 0];
inline bool Gnss::has_longitude() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Gnss::set_has_longitude() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Gnss::clear_has_longitude() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Gnss::clear_longitude() {
  longitude_ = 0;
  clear_has_longitude();
}
inline double Gnss::longitude() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.longitude)
  return longitude_;
}
inline void Gnss::set_longitude(double value) {
  set_has_longitude();
  longitude_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.longitude)
}

// optional double altitude = 4 [default = 0];
inline bool Gnss::has_altitude() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Gnss::set_has_altitude() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Gnss::clear_has_altitude() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Gnss::clear_altitude() {
  altitude_ = 0;
  clear_has_altitude();
}
inline double Gnss::altitude() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.altitude)
  return altitude_;
}
inline void Gnss::set_altitude(double value) {
  set_has_altitude();
  altitude_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.altitude)
}

// optional float heading_gnss = 5 [default = 0];
inline bool Gnss::has_heading_gnss() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Gnss::set_has_heading_gnss() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Gnss::clear_has_heading_gnss() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Gnss::clear_heading_gnss() {
  heading_gnss_ = 0;
  clear_has_heading_gnss();
}
inline float Gnss::heading_gnss() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.heading_gnss)
  return heading_gnss_;
}
inline void Gnss::set_heading_gnss(float value) {
  set_has_heading_gnss();
  heading_gnss_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.heading_gnss)
}

// optional double x_utm = 6 [default = 0];
inline bool Gnss::has_x_utm() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Gnss::set_has_x_utm() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Gnss::clear_has_x_utm() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Gnss::clear_x_utm() {
  x_utm_ = 0;
  clear_has_x_utm();
}
inline double Gnss::x_utm() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.x_utm)
  return x_utm_;
}
inline void Gnss::set_x_utm(double value) {
  set_has_x_utm();
  x_utm_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.x_utm)
}

// optional double y_utm = 7 [default = 0];
inline bool Gnss::has_y_utm() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Gnss::set_has_y_utm() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Gnss::clear_has_y_utm() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Gnss::clear_y_utm() {
  y_utm_ = 0;
  clear_has_y_utm();
}
inline double Gnss::y_utm() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.y_utm)
  return y_utm_;
}
inline void Gnss::set_y_utm(double value) {
  set_has_y_utm();
  y_utm_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.y_utm)
}

// optional double z_utm = 8 [default = 0];
inline bool Gnss::has_z_utm() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void Gnss::set_has_z_utm() {
  _has_bits_[0] |= 0x00000100u;
}
inline void Gnss::clear_has_z_utm() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void Gnss::clear_z_utm() {
  z_utm_ = 0;
  clear_has_z_utm();
}
inline double Gnss::z_utm() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.z_utm)
  return z_utm_;
}
inline void Gnss::set_z_utm(double value) {
  set_has_z_utm();
  z_utm_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.z_utm)
}

// optional float heading_utm = 9 [default = 0];
inline bool Gnss::has_heading_utm() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Gnss::set_has_heading_utm() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Gnss::clear_has_heading_utm() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Gnss::clear_heading_utm() {
  heading_utm_ = 0;
  clear_has_heading_utm();
}
inline float Gnss::heading_utm() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.heading_utm)
  return heading_utm_;
}
inline void Gnss::set_heading_utm(float value) {
  set_has_heading_utm();
  heading_utm_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.heading_utm)
}

// optional double x_odom = 10 [default = 0];
inline bool Gnss::has_x_odom() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void Gnss::set_has_x_odom() {
  _has_bits_[0] |= 0x00000200u;
}
inline void Gnss::clear_has_x_odom() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void Gnss::clear_x_odom() {
  x_odom_ = 0;
  clear_has_x_odom();
}
inline double Gnss::x_odom() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.x_odom)
  return x_odom_;
}
inline void Gnss::set_x_odom(double value) {
  set_has_x_odom();
  x_odom_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.x_odom)
}

// optional double y_odom = 11 [default = 0];
inline bool Gnss::has_y_odom() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void Gnss::set_has_y_odom() {
  _has_bits_[0] |= 0x00000400u;
}
inline void Gnss::clear_has_y_odom() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void Gnss::clear_y_odom() {
  y_odom_ = 0;
  clear_has_y_odom();
}
inline double Gnss::y_odom() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.y_odom)
  return y_odom_;
}
inline void Gnss::set_y_odom(double value) {
  set_has_y_odom();
  y_odom_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.y_odom)
}

// optional double z_odom = 12 [default = 0];
inline bool Gnss::has_z_odom() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void Gnss::set_has_z_odom() {
  _has_bits_[0] |= 0x00000800u;
}
inline void Gnss::clear_has_z_odom() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void Gnss::clear_z_odom() {
  z_odom_ = 0;
  clear_has_z_odom();
}
inline double Gnss::z_odom() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.z_odom)
  return z_odom_;
}
inline void Gnss::set_z_odom(double value) {
  set_has_z_odom();
  z_odom_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.z_odom)
}

// optional float heading_odom = 13 [default = 0];
inline bool Gnss::has_heading_odom() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void Gnss::set_has_heading_odom() {
  _has_bits_[0] |= 0x00001000u;
}
inline void Gnss::clear_has_heading_odom() {
  _has_bits_[0] &= ~0x00001000u;
}
inline void Gnss::clear_heading_odom() {
  heading_odom_ = 0;
  clear_has_heading_odom();
}
inline float Gnss::heading_odom() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.heading_odom)
  return heading_odom_;
}
inline void Gnss::set_heading_odom(float value) {
  set_has_heading_odom();
  heading_odom_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.heading_odom)
}

// optional float pitch = 14 [default = 0];
inline bool Gnss::has_pitch() const {
  return (_has_bits_[0] & 0x00002000u) != 0;
}
inline void Gnss::set_has_pitch() {
  _has_bits_[0] |= 0x00002000u;
}
inline void Gnss::clear_has_pitch() {
  _has_bits_[0] &= ~0x00002000u;
}
inline void Gnss::clear_pitch() {
  pitch_ = 0;
  clear_has_pitch();
}
inline float Gnss::pitch() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.pitch)
  return pitch_;
}
inline void Gnss::set_pitch(float value) {
  set_has_pitch();
  pitch_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.pitch)
}

// optional float roll = 15 [default = 0];
inline bool Gnss::has_roll() const {
  return (_has_bits_[0] & 0x00004000u) != 0;
}
inline void Gnss::set_has_roll() {
  _has_bits_[0] |= 0x00004000u;
}
inline void Gnss::clear_has_roll() {
  _has_bits_[0] &= ~0x00004000u;
}
inline void Gnss::clear_roll() {
  roll_ = 0;
  clear_has_roll();
}
inline float Gnss::roll() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.roll)
  return roll_;
}
inline void Gnss::set_roll(float value) {
  set_has_roll();
  roll_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.roll)
}

// optional float v_e = 16 [default = 0];
inline bool Gnss::has_v_e() const {
  return (_has_bits_[0] & 0x00008000u) != 0;
}
inline void Gnss::set_has_v_e() {
  _has_bits_[0] |= 0x00008000u;
}
inline void Gnss::clear_has_v_e() {
  _has_bits_[0] &= ~0x00008000u;
}
inline void Gnss::clear_v_e() {
  v_e_ = 0;
  clear_has_v_e();
}
inline float Gnss::v_e() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_e)
  return v_e_;
}
inline void Gnss::set_v_e(float value) {
  set_has_v_e();
  v_e_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_e)
}

// optional float v_n = 17 [default = 0];
inline bool Gnss::has_v_n() const {
  return (_has_bits_[0] & 0x00010000u) != 0;
}
inline void Gnss::set_has_v_n() {
  _has_bits_[0] |= 0x00010000u;
}
inline void Gnss::clear_has_v_n() {
  _has_bits_[0] &= ~0x00010000u;
}
inline void Gnss::clear_v_n() {
  v_n_ = 0;
  clear_has_v_n();
}
inline float Gnss::v_n() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_n)
  return v_n_;
}
inline void Gnss::set_v_n(float value) {
  set_has_v_n();
  v_n_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_n)
}

// optional float v_u = 18 [default = 0];
inline bool Gnss::has_v_u() const {
  return (_has_bits_[0] & 0x00020000u) != 0;
}
inline void Gnss::set_has_v_u() {
  _has_bits_[0] |= 0x00020000u;
}
inline void Gnss::clear_has_v_u() {
  _has_bits_[0] &= ~0x00020000u;
}
inline void Gnss::clear_v_u() {
  v_u_ = 0;
  clear_has_v_u();
}
inline float Gnss::v_u() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_u)
  return v_u_;
}
inline void Gnss::set_v_u(float value) {
  set_has_v_u();
  v_u_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_u)
}

// optional float v_x_utm = 19 [default = 0];
inline bool Gnss::has_v_x_utm() const {
  return (_has_bits_[0] & 0x00040000u) != 0;
}
inline void Gnss::set_has_v_x_utm() {
  _has_bits_[0] |= 0x00040000u;
}
inline void Gnss::clear_has_v_x_utm() {
  _has_bits_[0] &= ~0x00040000u;
}
inline void Gnss::clear_v_x_utm() {
  v_x_utm_ = 0;
  clear_has_v_x_utm();
}
inline float Gnss::v_x_utm() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_x_utm)
  return v_x_utm_;
}
inline void Gnss::set_v_x_utm(float value) {
  set_has_v_x_utm();
  v_x_utm_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_x_utm)
}

// optional float v_y_utm = 20 [default = 0];
inline bool Gnss::has_v_y_utm() const {
  return (_has_bits_[0] & 0x00080000u) != 0;
}
inline void Gnss::set_has_v_y_utm() {
  _has_bits_[0] |= 0x00080000u;
}
inline void Gnss::clear_has_v_y_utm() {
  _has_bits_[0] &= ~0x00080000u;
}
inline void Gnss::clear_v_y_utm() {
  v_y_utm_ = 0;
  clear_has_v_y_utm();
}
inline float Gnss::v_y_utm() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_y_utm)
  return v_y_utm_;
}
inline void Gnss::set_v_y_utm(float value) {
  set_has_v_y_utm();
  v_y_utm_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_y_utm)
}

// optional float v_z_utm = 21 [default = 0];
inline bool Gnss::has_v_z_utm() const {
  return (_has_bits_[0] & 0x00100000u) != 0;
}
inline void Gnss::set_has_v_z_utm() {
  _has_bits_[0] |= 0x00100000u;
}
inline void Gnss::clear_has_v_z_utm() {
  _has_bits_[0] &= ~0x00100000u;
}
inline void Gnss::clear_v_z_utm() {
  v_z_utm_ = 0;
  clear_has_v_z_utm();
}
inline float Gnss::v_z_utm() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_z_utm)
  return v_z_utm_;
}
inline void Gnss::set_v_z_utm(float value) {
  set_has_v_z_utm();
  v_z_utm_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_z_utm)
}

// optional float v_x_odom = 22 [default = 0];
inline bool Gnss::has_v_x_odom() const {
  return (_has_bits_[0] & 0x00200000u) != 0;
}
inline void Gnss::set_has_v_x_odom() {
  _has_bits_[0] |= 0x00200000u;
}
inline void Gnss::clear_has_v_x_odom() {
  _has_bits_[0] &= ~0x00200000u;
}
inline void Gnss::clear_v_x_odom() {
  v_x_odom_ = 0;
  clear_has_v_x_odom();
}
inline float Gnss::v_x_odom() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_x_odom)
  return v_x_odom_;
}
inline void Gnss::set_v_x_odom(float value) {
  set_has_v_x_odom();
  v_x_odom_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_x_odom)
}

// optional float v_y_odom = 23 [default = 0];
inline bool Gnss::has_v_y_odom() const {
  return (_has_bits_[0] & 0x00400000u) != 0;
}
inline void Gnss::set_has_v_y_odom() {
  _has_bits_[0] |= 0x00400000u;
}
inline void Gnss::clear_has_v_y_odom() {
  _has_bits_[0] &= ~0x00400000u;
}
inline void Gnss::clear_v_y_odom() {
  v_y_odom_ = 0;
  clear_has_v_y_odom();
}
inline float Gnss::v_y_odom() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_y_odom)
  return v_y_odom_;
}
inline void Gnss::set_v_y_odom(float value) {
  set_has_v_y_odom();
  v_y_odom_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_y_odom)
}

// optional float v_z_odom = 24 [default = 0];
inline bool Gnss::has_v_z_odom() const {
  return (_has_bits_[0] & 0x00800000u) != 0;
}
inline void Gnss::set_has_v_z_odom() {
  _has_bits_[0] |= 0x00800000u;
}
inline void Gnss::clear_has_v_z_odom() {
  _has_bits_[0] &= ~0x00800000u;
}
inline void Gnss::clear_v_z_odom() {
  v_z_odom_ = 0;
  clear_has_v_z_odom();
}
inline float Gnss::v_z_odom() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.v_z_odom)
  return v_z_odom_;
}
inline void Gnss::set_v_z_odom(float value) {
  set_has_v_z_odom();
  v_z_odom_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.v_z_odom)
}

// optional .phoenix.msg.localization.Gnss.Status gnss_status = 25 [default = STATUS_INVALID];
inline bool Gnss::has_gnss_status() const {
  return (_has_bits_[0] & 0x01000000u) != 0;
}
inline void Gnss::set_has_gnss_status() {
  _has_bits_[0] |= 0x01000000u;
}
inline void Gnss::clear_has_gnss_status() {
  _has_bits_[0] &= ~0x01000000u;
}
inline void Gnss::clear_gnss_status() {
  gnss_status_ = 0;
  clear_has_gnss_status();
}
inline ::phoenix::msg::localization::Gnss_Status Gnss::gnss_status() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.gnss_status)
  return static_cast< ::phoenix::msg::localization::Gnss_Status >(gnss_status_);
}
inline void Gnss::set_gnss_status(::phoenix::msg::localization::Gnss_Status value) {
  assert(::phoenix::msg::localization::Gnss_Status_IsValid(value));
  set_has_gnss_status();
  gnss_status_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.gnss_status)
}

// optional .phoenix.msg.localization.Gnss.Status utm_status = 26 [default = STATUS_INVALID];
inline bool Gnss::has_utm_status() const {
  return (_has_bits_[0] & 0x02000000u) != 0;
}
inline void Gnss::set_has_utm_status() {
  _has_bits_[0] |= 0x02000000u;
}
inline void Gnss::clear_has_utm_status() {
  _has_bits_[0] &= ~0x02000000u;
}
inline void Gnss::clear_utm_status() {
  utm_status_ = 0;
  clear_has_utm_status();
}
inline ::phoenix::msg::localization::Gnss_Status Gnss::utm_status() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.utm_status)
  return static_cast< ::phoenix::msg::localization::Gnss_Status >(utm_status_);
}
inline void Gnss::set_utm_status(::phoenix::msg::localization::Gnss_Status value) {
  assert(::phoenix::msg::localization::Gnss_Status_IsValid(value));
  set_has_utm_status();
  utm_status_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.utm_status)
}

// optional .phoenix.msg.localization.Gnss.Status odom_status = 27 [default = STATUS_INVALID];
inline bool Gnss::has_odom_status() const {
  return (_has_bits_[0] & 0x04000000u) != 0;
}
inline void Gnss::set_has_odom_status() {
  _has_bits_[0] |= 0x04000000u;
}
inline void Gnss::clear_has_odom_status() {
  _has_bits_[0] &= ~0x04000000u;
}
inline void Gnss::clear_odom_status() {
  odom_status_ = 0;
  clear_has_odom_status();
}
inline ::phoenix::msg::localization::Gnss_Status Gnss::odom_status() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.localization.Gnss.odom_status)
  return static_cast< ::phoenix::msg::localization::Gnss_Status >(odom_status_);
}
inline void Gnss::set_odom_status(::phoenix::msg::localization::Gnss_Status value) {
  assert(::phoenix::msg::localization::Gnss_Status_IsValid(value));
  set_has_odom_status();
  odom_status_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.localization.Gnss.odom_status)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace localization
}  // namespace msg
}  // namespace phoenix

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::phoenix::msg::localization::Gnss_Status> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::phoenix::msg::localization::Gnss_Status>() {
  return ::phoenix::msg::localization::Gnss_Status_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_gnss_2eproto__INCLUDED
