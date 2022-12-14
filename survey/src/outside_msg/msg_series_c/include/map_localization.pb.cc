// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: map_localization.proto

#include "map_localization.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)
namespace phoenix {
namespace msg {
namespace routing {
class PosDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Pos>
      _instance;
} _Pos_default_instance_;
class MapLocalizationDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<MapLocalization>
      _instance;
} _MapLocalization_default_instance_;
}  // namespace routing
}  // namespace msg
}  // namespace phoenix
namespace protobuf_map_5flocalization_2eproto {
void InitDefaultsPosImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  {
    void* ptr = &::phoenix::msg::routing::_Pos_default_instance_;
    new (ptr) ::phoenix::msg::routing::Pos();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::phoenix::msg::routing::Pos::InitAsDefaultInstance();
}

void InitDefaultsPos() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsPosImpl);
}

void InitDefaultsMapLocalizationImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  protobuf_header_2eproto::InitDefaultsHeader();
  protobuf_map_5flocalization_2eproto::InitDefaultsPos();
  {
    void* ptr = &::phoenix::msg::routing::_MapLocalization_default_instance_;
    new (ptr) ::phoenix::msg::routing::MapLocalization();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::phoenix::msg::routing::MapLocalization::InitAsDefaultInstance();
}

void InitDefaultsMapLocalization() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsMapLocalizationImpl);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::Pos, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::Pos, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::Pos, nearest_lane_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::Pos, s_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::Pos, l_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::Pos, heading_),
  0,
  1,
  2,
  3,
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::MapLocalization, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::MapLocalization, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::MapLocalization, header_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::phoenix::msg::routing::MapLocalization, point_),
  0,
  1,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::phoenix::msg::routing::Pos)},
  { 13, 20, sizeof(::phoenix::msg::routing::MapLocalization)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::phoenix::msg::routing::_Pos_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::phoenix::msg::routing::_MapLocalization_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "map_localization.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\026map_localization.proto\022\023phoenix.msg.ro"
      "uting\032\014header.proto\"N\n\003Pos\022\027\n\017nearest_la"
      "ne_id\030\001 \001(\t\022\014\n\001s\030\002 \001(\001:\0010\022\014\n\001l\030\003 \001(\001:\0010\022"
      "\022\n\007heading\030\004 \001(\001:\0010\"f\n\017MapLocalization\022*"
      "\n\006header\030\001 \001(\0132\032.phoenix.msg.common.Head"
      "er\022\'\n\005point\030\002 \001(\0132\030.phoenix.msg.routing."
      "Pos"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 243);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "map_localization.proto", &protobuf_RegisterTypes);
  ::protobuf_header_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_map_5flocalization_2eproto
namespace phoenix {
namespace msg {
namespace routing {

// ===================================================================

void Pos::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Pos::kNearestLaneIdFieldNumber;
const int Pos::kSFieldNumber;
const int Pos::kLFieldNumber;
const int Pos::kHeadingFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Pos::Pos()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_map_5flocalization_2eproto::InitDefaultsPos();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:phoenix.msg.routing.Pos)
}
Pos::Pos(const Pos& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  nearest_lane_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_nearest_lane_id()) {
    nearest_lane_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.nearest_lane_id_);
  }
  ::memcpy(&s_, &from.s_,
    static_cast<size_t>(reinterpret_cast<char*>(&heading_) -
    reinterpret_cast<char*>(&s_)) + sizeof(heading_));
  // @@protoc_insertion_point(copy_constructor:phoenix.msg.routing.Pos)
}

void Pos::SharedCtor() {
  _cached_size_ = 0;
  nearest_lane_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&s_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&heading_) -
      reinterpret_cast<char*>(&s_)) + sizeof(heading_));
}

Pos::~Pos() {
  // @@protoc_insertion_point(destructor:phoenix.msg.routing.Pos)
  SharedDtor();
}

void Pos::SharedDtor() {
  nearest_lane_id_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void Pos::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Pos::descriptor() {
  ::protobuf_map_5flocalization_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_map_5flocalization_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Pos& Pos::default_instance() {
  ::protobuf_map_5flocalization_2eproto::InitDefaultsPos();
  return *internal_default_instance();
}

Pos* Pos::New(::google::protobuf::Arena* arena) const {
  Pos* n = new Pos;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Pos::Clear() {
// @@protoc_insertion_point(message_clear_start:phoenix.msg.routing.Pos)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(!nearest_lane_id_.IsDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited()));
    (*nearest_lane_id_.UnsafeRawStringPointer())->clear();
  }
  if (cached_has_bits & 14u) {
    ::memset(&s_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&heading_) -
        reinterpret_cast<char*>(&s_)) + sizeof(heading_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Pos::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:phoenix.msg.routing.Pos)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional string nearest_lane_id = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_nearest_lane_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->nearest_lane_id().data(), static_cast<int>(this->nearest_lane_id().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "phoenix.msg.routing.Pos.nearest_lane_id");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double s = 2 [default = 0];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_s();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &s_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double l = 3 [default = 0];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {
          set_has_l();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &l_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double heading = 4 [default = 0];
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {
          set_has_heading();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &heading_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:phoenix.msg.routing.Pos)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:phoenix.msg.routing.Pos)
  return false;
#undef DO_
}

void Pos::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:phoenix.msg.routing.Pos)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string nearest_lane_id = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->nearest_lane_id().data(), static_cast<int>(this->nearest_lane_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "phoenix.msg.routing.Pos.nearest_lane_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->nearest_lane_id(), output);
  }

  // optional double s = 2 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->s(), output);
  }

  // optional double l = 3 [default = 0];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->l(), output);
  }

  // optional double heading = 4 [default = 0];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->heading(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:phoenix.msg.routing.Pos)
}

::google::protobuf::uint8* Pos::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:phoenix.msg.routing.Pos)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string nearest_lane_id = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->nearest_lane_id().data(), static_cast<int>(this->nearest_lane_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "phoenix.msg.routing.Pos.nearest_lane_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->nearest_lane_id(), target);
  }

  // optional double s = 2 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->s(), target);
  }

  // optional double l = 3 [default = 0];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->l(), target);
  }

  // optional double heading = 4 [default = 0];
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->heading(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:phoenix.msg.routing.Pos)
  return target;
}

size_t Pos::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:phoenix.msg.routing.Pos)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 15u) {
    // optional string nearest_lane_id = 1;
    if (has_nearest_lane_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->nearest_lane_id());
    }

    // optional double s = 2 [default = 0];
    if (has_s()) {
      total_size += 1 + 8;
    }

    // optional double l = 3 [default = 0];
    if (has_l()) {
      total_size += 1 + 8;
    }

    // optional double heading = 4 [default = 0];
    if (has_heading()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Pos::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:phoenix.msg.routing.Pos)
  GOOGLE_DCHECK_NE(&from, this);
  const Pos* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Pos>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:phoenix.msg.routing.Pos)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:phoenix.msg.routing.Pos)
    MergeFrom(*source);
  }
}

void Pos::MergeFrom(const Pos& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:phoenix.msg.routing.Pos)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_nearest_lane_id();
      nearest_lane_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.nearest_lane_id_);
    }
    if (cached_has_bits & 0x00000002u) {
      s_ = from.s_;
    }
    if (cached_has_bits & 0x00000004u) {
      l_ = from.l_;
    }
    if (cached_has_bits & 0x00000008u) {
      heading_ = from.heading_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void Pos::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:phoenix.msg.routing.Pos)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Pos::CopyFrom(const Pos& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:phoenix.msg.routing.Pos)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Pos::IsInitialized() const {
  return true;
}

void Pos::Swap(Pos* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Pos::InternalSwap(Pos* other) {
  using std::swap;
  nearest_lane_id_.Swap(&other->nearest_lane_id_);
  swap(s_, other->s_);
  swap(l_, other->l_);
  swap(heading_, other->heading_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Pos::GetMetadata() const {
  protobuf_map_5flocalization_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_map_5flocalization_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void MapLocalization::InitAsDefaultInstance() {
  ::phoenix::msg::routing::_MapLocalization_default_instance_._instance.get_mutable()->header_ = const_cast< ::phoenix::msg::common::Header*>(
      ::phoenix::msg::common::Header::internal_default_instance());
  ::phoenix::msg::routing::_MapLocalization_default_instance_._instance.get_mutable()->point_ = const_cast< ::phoenix::msg::routing::Pos*>(
      ::phoenix::msg::routing::Pos::internal_default_instance());
}
void MapLocalization::clear_header() {
  if (header_ != NULL) header_->Clear();
  clear_has_header();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int MapLocalization::kHeaderFieldNumber;
const int MapLocalization::kPointFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

MapLocalization::MapLocalization()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_map_5flocalization_2eproto::InitDefaultsMapLocalization();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:phoenix.msg.routing.MapLocalization)
}
MapLocalization::MapLocalization(const MapLocalization& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_header()) {
    header_ = new ::phoenix::msg::common::Header(*from.header_);
  } else {
    header_ = NULL;
  }
  if (from.has_point()) {
    point_ = new ::phoenix::msg::routing::Pos(*from.point_);
  } else {
    point_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:phoenix.msg.routing.MapLocalization)
}

void MapLocalization::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&header_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&point_) -
      reinterpret_cast<char*>(&header_)) + sizeof(point_));
}

MapLocalization::~MapLocalization() {
  // @@protoc_insertion_point(destructor:phoenix.msg.routing.MapLocalization)
  SharedDtor();
}

void MapLocalization::SharedDtor() {
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete point_;
}

void MapLocalization::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* MapLocalization::descriptor() {
  ::protobuf_map_5flocalization_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_map_5flocalization_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const MapLocalization& MapLocalization::default_instance() {
  ::protobuf_map_5flocalization_2eproto::InitDefaultsMapLocalization();
  return *internal_default_instance();
}

MapLocalization* MapLocalization::New(::google::protobuf::Arena* arena) const {
  MapLocalization* n = new MapLocalization;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void MapLocalization::Clear() {
// @@protoc_insertion_point(message_clear_start:phoenix.msg.routing.MapLocalization)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(header_ != NULL);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(point_ != NULL);
      point_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool MapLocalization::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:phoenix.msg.routing.MapLocalization)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .phoenix.msg.common.Header header = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .phoenix.msg.routing.Pos point = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_point()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:phoenix.msg.routing.MapLocalization)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:phoenix.msg.routing.MapLocalization)
  return false;
#undef DO_
}

void MapLocalization::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:phoenix.msg.routing.MapLocalization)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .phoenix.msg.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->header_, output);
  }

  // optional .phoenix.msg.routing.Pos point = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, *this->point_, output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:phoenix.msg.routing.MapLocalization)
}

::google::protobuf::uint8* MapLocalization::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:phoenix.msg.routing.MapLocalization)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .phoenix.msg.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, *this->header_, deterministic, target);
  }

  // optional .phoenix.msg.routing.Pos point = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, *this->point_, deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:phoenix.msg.routing.MapLocalization)
  return target;
}

size_t MapLocalization::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:phoenix.msg.routing.MapLocalization)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 3u) {
    // optional .phoenix.msg.common.Header header = 1;
    if (has_header()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *this->header_);
    }

    // optional .phoenix.msg.routing.Pos point = 2;
    if (has_point()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *this->point_);
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void MapLocalization::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:phoenix.msg.routing.MapLocalization)
  GOOGLE_DCHECK_NE(&from, this);
  const MapLocalization* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const MapLocalization>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:phoenix.msg.routing.MapLocalization)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:phoenix.msg.routing.MapLocalization)
    MergeFrom(*source);
  }
}

void MapLocalization::MergeFrom(const MapLocalization& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:phoenix.msg.routing.MapLocalization)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_header()->::phoenix::msg::common::Header::MergeFrom(from.header());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_point()->::phoenix::msg::routing::Pos::MergeFrom(from.point());
    }
  }
}

void MapLocalization::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:phoenix.msg.routing.MapLocalization)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MapLocalization::CopyFrom(const MapLocalization& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:phoenix.msg.routing.MapLocalization)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MapLocalization::IsInitialized() const {
  return true;
}

void MapLocalization::Swap(MapLocalization* other) {
  if (other == this) return;
  InternalSwap(other);
}
void MapLocalization::InternalSwap(MapLocalization* other) {
  using std::swap;
  swap(header_, other->header_);
  swap(point_, other->point_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata MapLocalization::GetMetadata() const {
  protobuf_map_5flocalization_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_map_5flocalization_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace routing
}  // namespace msg
}  // namespace phoenix

// @@protoc_insertion_point(global_scope)
