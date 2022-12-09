// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: chassis_ft_auman.proto

#ifndef PROTOBUF_chassis_5fft_5fauman_2eproto__INCLUDED
#define PROTOBUF_chassis_5fft_5fauman_2eproto__INCLUDED

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
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace protobuf_chassis_5fft_5fauman_2eproto {
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
void InitDefaultsChassisFtAumanImpl();
void InitDefaultsChassisFtAuman();
inline void InitDefaults() {
  InitDefaultsChassisFtAuman();
}
}  // namespace protobuf_chassis_5fft_5fauman_2eproto
namespace phoenix {
namespace msg {
namespace control {
class ChassisFtAuman;
class ChassisFtAumanDefaultTypeInternal;
extern ChassisFtAumanDefaultTypeInternal _ChassisFtAuman_default_instance_;
}  // namespace control
}  // namespace msg
}  // namespace phoenix
namespace phoenix {
namespace msg {
namespace control {

// ===================================================================

class ChassisFtAuman : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:phoenix.msg.control.ChassisFtAuman) */ {
 public:
  ChassisFtAuman();
  virtual ~ChassisFtAuman();

  ChassisFtAuman(const ChassisFtAuman& from);

  inline ChassisFtAuman& operator=(const ChassisFtAuman& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ChassisFtAuman(ChassisFtAuman&& from) noexcept
    : ChassisFtAuman() {
    *this = ::std::move(from);
  }

  inline ChassisFtAuman& operator=(ChassisFtAuman&& from) noexcept {
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
  static const ChassisFtAuman& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ChassisFtAuman* internal_default_instance() {
    return reinterpret_cast<const ChassisFtAuman*>(
               &_ChassisFtAuman_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(ChassisFtAuman* other);
  friend void swap(ChassisFtAuman& a, ChassisFtAuman& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ChassisFtAuman* New() const PROTOBUF_FINAL { return New(NULL); }

  ChassisFtAuman* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ChassisFtAuman& from);
  void MergeFrom(const ChassisFtAuman& from);
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
  void InternalSwap(ChassisFtAuman* other);
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

  // accessors -------------------------------------------------------

  // optional int32 switch_tja = 1 [default = 0];
  bool has_switch_tja() const;
  void clear_switch_tja();
  static const int kSwitchTjaFieldNumber = 1;
  ::google::protobuf::int32 switch_tja() const;
  void set_switch_tja(::google::protobuf::int32 value);

  // optional int32 switch_hwa = 2 [default = 0];
  bool has_switch_hwa() const;
  void clear_switch_hwa();
  static const int kSwitchHwaFieldNumber = 2;
  ::google::protobuf::int32 switch_hwa() const;
  void set_switch_hwa(::google::protobuf::int32 value);

  // optional int32 switch_i_drive = 3 [default = 0];
  bool has_switch_i_drive() const;
  void clear_switch_i_drive();
  static const int kSwitchIDriveFieldNumber = 3;
  ::google::protobuf::int32 switch_i_drive() const;
  void set_switch_i_drive(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:phoenix.msg.control.ChassisFtAuman)
 private:
  void set_has_switch_tja();
  void clear_has_switch_tja();
  void set_has_switch_hwa();
  void clear_has_switch_hwa();
  void set_has_switch_i_drive();
  void clear_has_switch_i_drive();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::int32 switch_tja_;
  ::google::protobuf::int32 switch_hwa_;
  ::google::protobuf::int32 switch_i_drive_;
  friend struct ::protobuf_chassis_5fft_5fauman_2eproto::TableStruct;
  friend void ::protobuf_chassis_5fft_5fauman_2eproto::InitDefaultsChassisFtAumanImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ChassisFtAuman

// optional int32 switch_tja = 1 [default = 0];
inline bool ChassisFtAuman::has_switch_tja() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ChassisFtAuman::set_has_switch_tja() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ChassisFtAuman::clear_has_switch_tja() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ChassisFtAuman::clear_switch_tja() {
  switch_tja_ = 0;
  clear_has_switch_tja();
}
inline ::google::protobuf::int32 ChassisFtAuman::switch_tja() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.ChassisFtAuman.switch_tja)
  return switch_tja_;
}
inline void ChassisFtAuman::set_switch_tja(::google::protobuf::int32 value) {
  set_has_switch_tja();
  switch_tja_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.ChassisFtAuman.switch_tja)
}

// optional int32 switch_hwa = 2 [default = 0];
inline bool ChassisFtAuman::has_switch_hwa() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ChassisFtAuman::set_has_switch_hwa() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ChassisFtAuman::clear_has_switch_hwa() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ChassisFtAuman::clear_switch_hwa() {
  switch_hwa_ = 0;
  clear_has_switch_hwa();
}
inline ::google::protobuf::int32 ChassisFtAuman::switch_hwa() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.ChassisFtAuman.switch_hwa)
  return switch_hwa_;
}
inline void ChassisFtAuman::set_switch_hwa(::google::protobuf::int32 value) {
  set_has_switch_hwa();
  switch_hwa_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.ChassisFtAuman.switch_hwa)
}

// optional int32 switch_i_drive = 3 [default = 0];
inline bool ChassisFtAuman::has_switch_i_drive() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ChassisFtAuman::set_has_switch_i_drive() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ChassisFtAuman::clear_has_switch_i_drive() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ChassisFtAuman::clear_switch_i_drive() {
  switch_i_drive_ = 0;
  clear_has_switch_i_drive();
}
inline ::google::protobuf::int32 ChassisFtAuman::switch_i_drive() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.ChassisFtAuman.switch_i_drive)
  return switch_i_drive_;
}
inline void ChassisFtAuman::set_switch_i_drive(::google::protobuf::int32 value) {
  set_has_switch_i_drive();
  switch_i_drive_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.ChassisFtAuman.switch_i_drive)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace control
}  // namespace msg
}  // namespace phoenix

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_chassis_5fft_5fauman_2eproto__INCLUDED
